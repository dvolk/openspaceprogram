#!/usr/bin/env python3
"""#682's band as a BUILD refusal: an air ceiling must sit where orbiting air pushes.

An atmosphere top is not a rendering number. top() sets shellEdge = max(100 km,
1.2 top), so one key moves the rotating frame, the LowOrbit/HighOrbit cuts, every
spawn bed and the atlas disk; and drag.h bills NOTHING at or above it, so the top is
also the altitude where a decaying orbit stops feeling air at all. #682's rule is that
the ceiling should sit where a circular orbit still meets 10..1000 Pa of q -- and the
band is anchored on the game rather than on a plucked constant, because Kerbin's own
edge reads 97.8 Pa. Out of band OVER means a wall of air at the ceiling; UNDER means a
cliff drawn in vacuum, which buys the bigger rot frame for nothing.

Until now this verdict was only PRINTED, by utils/atmo_profile.py, so a top could walk
out of band in a commit whose report nobody read. Six of fourteen air bodies were out
of band when this gate was written -- Venus 99.8 kPa, Jupiter 6.8 k, Uranus 3.3 k,
Neptune 3.0 k, Jool 2.9 k, Saturn 2.4 k -- and five of them are now in band because
they author a `height` (#682 step 3). This file is what keeps them there.

Why it IMPORTS atmo_profile instead of re-reading the JSON: that file is a second,
quadrature spelling of atmo.h that shares no code with the loader, and it lands on the
C++ pins to seven digits. A gate built on the loader's own closed form could only ever
catch a mistyped authoring number; this one can also catch the LAW drifting. The band
is 100x wide, so a 1e-7 disagreement between the two spellings cannot flip a verdict.
atmo_profile itself stays print-only -- "a second spelling of a law is evidence, not a
gate" -- and this is its consumer.

Waivers are VALUE-bound, exactly like make_solar_system.py's FACT_AIR_KNOWN: a waiver
carries the authored top it was written against, so if the file's number moves by more
than WAIVER_TOL the guard refuses again rather than sheltering a new value under an
old argument.

THE FRAME, because it is not the one the sim bills. The band is INERTIAL circular q,
1/2 rho v_circ^2, and the verdict is that number for every body -- Kerbin's 97.8 Pa
anchor is measured the same way. What the aero path charges is AIR-RELATIVE q in the
rotating frame (vehicle.cpp co-rotates the flow; bodylimits.h keeps the shell inside
the rot frame, which is what makes that assumption hold), and on a ~10 h rotator the
two differ by up to 2.6x: Jupiter's 390 km ceiling reads 83.3 Pa inertial, 41.8 Pa
for a prograde hull, 138.8 Pa retrograde -- retrograde is NOT inertial + prograde,
the co-rotating ground adds to v, not to q. Both print per row so nobody quotes the
wrong one, and every shipped body lands in band in BOTH frames: the largest shift in
the catalogue is Saturn prograde at 35.8 Pa, still 3.6x above the lower lip. Kerbin's
6 h day moves its own edge 97.8 -> 81.4 Pa, so this is not a giants-only caveat --
the giants are just where it is biggest.

Usage:  python3 utils/check_atmo_band.py [system.json ...]
        python3 utils/check_atmo_band.py --self-test
"""
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import atmo_profile as ap

Q_LO, Q_HI = ap.Q_BAND
WAIVER_TOL = 0.02

# body -> (the authored top this waiver was written against, why it is defensible).
KNOWN = {
    'Venus': (
        160000.0,
        "99,829 Pa of orbital q at its ceiling (10 H, since Venus authors no "
        "height), 100x the band's upper lip. This is the band measuring the wrong "
        "thing, and #700 said so about Laythe: the verdict speaks only AT the "
        "ceiling. Authoring 238 km would put the edge at 1.0 kPa and satisfy this "
        "gate, and a circular orbit at 150 km would still meet 181 kPa of q there -- "
        "the column (65 kg/m3 at the datum, H 16 km) is the problem, not the cut. "
        "Fixing it means a shorter H or a cooler Venus, which is a datum decision "
        "check_fact_air argues about, not a top. Waived BY VALUE: change the 160 km "
        "by more than 2% and this excuse no longer applies."),
}

DEFAULTS = ['res/systems/ksp_system.json', 'res/systems/solar_system.json']


def ceiling_q(pr):
    """The limit from BELOW the top: drag.h bills nothing at or above `height`, so
    this is the last q a decaying orbit is subject to, not one it is charged at the
    top itself. Same number atmo_profile prints."""
    ptop = pr.p(pr.top)
    return 0.5 * (ptop / (pr.Rs * pr.T(pr.top))) * pr.vcirc(pr.top) ** 2


def q_at(pr, z, steps=6000):
    """q of a circular orbit at z, by the same quadrature. Explicit because the advice
    a refusal prints wants a coarser column than the verdict does."""
    return 0.5 * (pr.p(z, steps=steps) / (pr.Rs * pr.T(z))) * pr.vcirc(z) ** 2


def air_q(pr, omega, z=None):
    """q of a PROGRADE circular orbit measured against the air, which is what the
    aero path charges (the flow co-rotates). The band verdict uses the inertial
    number; this is printed beside it so a ~10 h rotator's row is not misread."""
    z = pr.top if z is None else z
    rho = pr.p(z) / (pr.Rs * pr.T(z))
    v = pr.vcirc(z) - omega * (pr.radius + z)
    return 0.5 * rho * v * v


def in_band_q(pr, target):
    """Where a circular orbit here would feel `target` Pa -- the fix the refusal
    offers. q falls with altitude, so keep the bracket q(lo) > target > q(hi).
    Advisory text, so it runs only on a refusal and on a coarse column."""
    lo, hi = 1.0, pr.radius * 0.2
    # Both ends of the bracket, or a body with almost no air gets "10 Pa sits at 1 m"
    # printed at it as if that were a fix.
    if q_at(pr, hi, 400) > target or q_at(pr, lo, 400) < target:
        return None
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if q_at(pr, mid, 400) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def decide(name, top, q, known, suggest=None):
    """One row, and whether it refuses. Split out so --self-test can drive it with
    numbers no shipped file contains."""
    w = known.get(name)
    if w and abs(w[0] - top) > WAIVER_TOL * top:
        return False, 'STALE WAIVER', (
            '%s: top is %.0f m but the waiver was written against %.0f m -- re-argue '
            'it or drop it' % (name, top, w[0]))
    if Q_LO <= q <= Q_HI:
        return True, 'in band', '%s: q %.1f Pa at %.0f m' % (name, q, top)
    def say(target):
        z = suggest(target) if suggest else None
        return '%.0f m' % z if z else 'not reachable below 0.2 R'
    if q > Q_HI:
        msg = ('%s: a circular orbit at its own ceiling feels %.0f Pa of q, %.0fx '
               'over the band (Kerbin reads 97.8 at its edge). %.0f Pa sits at %s'
               % (name, q, q / Q_HI, Q_HI, say(Q_HI)))
    else:
        msg = ('%s: a circular orbit at its own ceiling feels only %.3g Pa -- the '
               'cliff is drawn in vacuum, and the bigger rot frame is bought for '
               'nothing. %.0f Pa sits at %s'
               % (name, q, Q_LO, say(Q_LO)))
    if w:
        return True, 'WAIVED', msg + '\n    waived: ' + w[1]
    return False, 'OUT OF BAND', msg


def check_docs(docs, known=KNOWN, verbose=True):
    """docs: [(label, [body dict, ...])]. Split from the file reading so --self-test
    can drive the REAL refusal path -- the one that counts and decides the exit code
    -- with a document no shipped file contains."""
    bad = 0
    weighed = 0
    seen = set()
    for label, bodies in docs:
        for b in bodies:
            su = b.get('surface') or {}
            at = su.get('atmosphere')
            if not at or 'sea_level_density' not in at:
                continue
            pr = ap.Profile(b)
            # The catalogue generates the same columns into several system files; a
            # column is weighed once. mass and M are in the key because vcirc comes
            # from one and a layered datum pressure from the other: two files that
            # agree on every authored air field but not on mass are NOT the same
            # column, and skipping the second would hide a real disagreement.
            key = (pr.name, pr.radius, pr.g, pr.mu, pr.M, pr.rho0, pr.H, pr.top,
                   tuple((l['alt'], l['T'], l['lapse']) for l in pr.levels))
            if key in seen:
                continue
            seen.add(key)
            weighed += 1
            q = ceiling_q(pr)
            omega = float(((b.get('rotating') or {}).get('rot_ang_speed')) or 0.0)
            ok, tag, msg = decide(pr.name, pr.top, q, known,
                                  lambda target: in_band_q(pr, target))
            if verbose:
                shape = 'table' if pr.levels else ('H %6.0f m' % pr.H)
                print('  %-9s top %8.0f m  %-12s q %10.1f Pa  air %9.1f Pa  %s'
                      % (pr.name, pr.top, shape, q, air_q(pr, omega), tag))
                if tag == 'WAIVED':
                    # The argument, not just the fact that one was filed.
                    print('    ' + msg.split('\n', 1)[1])
            if not ok:
                if verbose:
                    print('  REFUSE %s' % msg)
                bad += 1
    return bad, weighed


def check(paths, known=KNOWN, verbose=True):
    return check_docs([(p, ap.read_system(p)['bodies']) for p in paths], known, verbose)


def self_test():
    """Every gate needs a mutation that turns it red, or it is decoration."""
    fails = 0

    def expect(want, got, what):
        nonlocal fails
        if want != got:
            print('  SELF-TEST FAIL: %s (wanted %s, got %s)' % (what, want, got))
            fails += 1

    # In band passes with no waiver on offer.
    expect(True, decide('Kerbin', 85000.0, 97.8, {})[0], 'in-band body')
    # Over the wall refuses.
    expect(False, decide('Joolish', 200000.0, 2856.0, {})[0], 'a wall at the ceiling')
    # A cliff drawn in vacuum refuses too -- the opposite failure, same gate.
    expect(False, decide('Airless', 400000.0, 1e-4, {})[0], 'a cliff in vacuum')
    # A waiver shelters its own number...
    w = {'Venus': (160000.0, 'the column, not the cut')}
    expect(True, decide('Venus', 160000.0, 99829.0, w)[0], 'Venus waived')
    # ...and stops sheltering the moment the number moves.
    expect(False, decide('Venus', 238000.0, 998.0, w)[0], 'a waiver out of date')
    # A body whose q lands at the band's lip is IN, not waived.
    expect(True, decide('Edge', 275000.0, 1000.0, {})[0], 'the upper lip is in band')
    # The gate end to end, on documents no shipped file contains: decide() being right
    # is not enough, because the lines that weigh a column and the line that counts a
    # refusal together decide a build. One layered and one exponential body, so a filter
    # that quietly started skipping a family would show up as the wrong count.
    fat = {'name': 'Fatty', 'radius': 600000.0, 'mass': 5.29e22, 'g': 9.81,
           'surface': {'atmosphere': {
               'sea_level_density': 1.225, 'height': 1000.0,
               'layers': [{'alt': 0.0, 'T': 288.15, 'lapse': -0.0065}]}}}
    thin = {'name': 'Thinny', 'radius': 600000.0, 'mass': 5.29e22, 'g': 9.81,
            'surface': {'atmosphere': {'sea_level_density': 1e-6,
                                       'scale_height': 20000.0, 'height': 400000.0}}}
    bad, weighed = check_docs([('selftest', [fat, thin])], known={}, verbose=False)
    expect(2, weighed, 'both air families reach the scale')
    expect(2, bad, 'both refusals reach the count')
    print('check_atmo_band: %s' % ('OK' if fails == 0 else '%d SELF-TEST FAILURES' % fails))
    return 1 if fails else 0


def main(argv):
    args = argv[1:]
    if '--help' in args:
        print(__doc__)
        return 0
    if '--self-test' in args:
        return self_test()
    paths = [p for p in args if not p.startswith('-')] or DEFAULTS
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paths = [p if os.path.isabs(p) else os.path.join(root, p) for p in paths]
    bad, weighed = check(paths)
    if bad:
        print('check_atmo_band: %d body/ bodies out of #682\'s %g..%g Pa band. A top is '
              'a decision (shellEdge, spawn beds, the atlas disk); author one where the '
              'air still pushes, or waive it BY VALUE with the reason in KNOWN.'
              % (bad, Q_LO, Q_HI))
        return 1
    print('check_atmo_band: %d air columns weighed, every one inside %g..%g Pa of '
          'orbital q at its own ceiling (%d waiver/s filed)'
          % (weighed, Q_LO, Q_HI, len(KNOWN)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
