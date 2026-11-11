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
rotating frame: vehicle.cpp co-rotates the flow, and bodylimits.h is what licenses
that -- its floor and 1.2x fraction keep the AIR inside the shell, and the shell is
the rot frame's edge. On a ~10 h rotator the two numbers differ a lot: Saturn reads
93.5 Pa inertial against 35.8 for a prograde hull (2.6x, the largest shift in the
catalogue), Jupiter 83.3 against 41.8 (2.0x), and 138.8 retrograde -- retrograde is
NOT inertial + prograde, since the co-rotating ground adds to v, not to q. Both air
columns print per row, at the equatorial co-rotation speed: any other heading lands
between them. They are ADVISORY, not gated. The verdict stays inertial on purpose --
#682's band asks where an orbit meets air, not which way a hull points, and a
near-co-rotating sky is a world rather than a bug. As it happens every shipped body
is inside the band in all three columns -- every shipped column, with no waiver left
since #625 gave Venus a profile instead of a scale height. The lowest air-relative
reading is Pluto's 13.6 Pa, 1.4x above the lower lip; the widest spread between the
two headings is Saturn's 99.7 -> 488.2 Pa, because there its co-rotation (9.5 km/s at the
equator) is a large fraction of orbital speed. Kerbin's own 6 h day moves its edge 97.8 ->
81.4 Pa, so this is not a giants-only caveat.

Usage:  python3 utils/check_atmo_band.py [system.json ...]
        python3 utils/check_atmo_band.py --self-test
"""
import contextlib, io, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import atmo_profile as ap

Q_LO, Q_HI = ap.Q_BAND
WAIVER_TOL = 0.02

# body -> (the authored top this waiver was written against, why it is defensible).
KNOWN = {}
# Empty, and it was not. The one waiver here covered Venus's 160 km ceiling at
# 99,829 Pa of orbital q -- 100x the band's upper lip -- and its stated reason was
# "the column (65 kg/m3 at the datum, H 16 km) is the problem, not the cut. Fixing it
# means a shorter H or a cooler Venus". #625 found the third way the waiver did not
# imagine: give Venus the profile its own sheet states (737 K at the surface, 230 K at
# the 65 km tropopause) and the aloft air is 5x thinner, so a ceiling of 116 km sits at
# 275 Pa. The waiver expired with the profile that needed it, which is what WAIVER_TOL
# is for -- it refused at exactly this change and named the number it had pinned.

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


def air_q(pr, omega, z=None, retro=False):
    """q of a circular orbit measured against the AIR, which is what the aero path
    charges (the flow co-rotates). `retro` mirrors the heading. Both print beside the
    inertial number, so a ~10 h rotator's row is not misread; neither decides the
    verdict, and the two do not add: the co-rotating ground adds to v, not to q."""
    z = pr.top if z is None else z
    rho = pr.p(z) / (pr.Rs * pr.T(z))
    v = pr.vcirc(z) - (omega if not retro else -omega) * (pr.radius + z)
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
    waived = 0
    seen = set()
    for label, bodies in docs:
        for b in bodies:
            su = b.get('surface') or {}
            at = su.get('atmosphere')
            # #739: the authored anchor is the datum pressure; atmo_profile resolves
            # the density from it, and a body with no pressure has no column to weigh.
            if not at or 'sea_level_pressure' not in at:
                continue
            pr = ap.Profile(b)
            # The catalogue generates the same columns into several system files; a
            # column is weighed once. mass and M are in the key because vcirc comes
            # from one and a layered datum pressure from the other: two files that
            # agree on every authored air field but not on mass are NOT the same
            # column, and skipping the second would hide a real disagreement.
            key = (pr.name, pr.radius, pr.g, pr.mu, pr.M, pr.p0, pr.H, pr.top,
                   tuple((l['alt'], l['T'], l['lapse']) for l in pr.levels))
            if key in seen:
                continue
            seen.add(key)
            weighed += 1
            q = ceiling_q(pr)
            omega = float(((b.get('rotating') or {}).get('rot_ang_speed')) or 0.0)
            ok, tag, msg = decide(pr.name, pr.top, q, known,
                                  lambda target: in_band_q(pr, target))
            if tag == 'WAIVED':
                waived += 1
            if verbose:
                shape = 'table' if pr.levels else ('H %6.0f m' % pr.H)
                # air is prograde/retrograde q against the co-rotating flow: advisory
                # columns, and they bracket any other heading (THE FRAME, above).
                print('  %-9s top %8.0f m  %-12s q %10.1f Pa  air %8.1f/%-8.1f  %s'
                      % (pr.name, pr.top, shape, q, air_q(pr, omega),
                         air_q(pr, omega, retro=True), tag))
                if tag == 'WAIVED':
                    # The argument, not just the fact that one was filed. --self-test
                    # pins that decide() and this line agree about the newline.
                    print('    ' + msg.split('\n', 1)[1])
            if not ok:
                if verbose:
                    # Name the document: several system files can hold the same body
                    # differently, and a refusal has to say which one it is holding.
                    print('  REFUSE [%s] %s' % (label, msg))
                bad += 1
    return bad, weighed, waived


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
    # refusal together decide a build. Two SCALES on purpose -- a Kerbin-sized pair and
    # a giant-sized one -- because a filter that quietly started skipping bodies by size
    # would otherwise pass, and the giants are the family this gate exists for.
    fat = {'name': 'Fatty', 'radius': 600000.0, 'mass': 5.29e22,
           'surface': {'atmosphere': {
               # 101,325 Pa at 288.15 K is the 1.225 kg/m3 this fixture used to type
               'sea_level_pressure': 101325.0, 'height': 1000.0,
               'layers': [{'alt': 0.0, 'T': 288.15, 'lapse': -0.0065}]}}}
    thin = {'name': 'Thinny', 'radius': 600000.0, 'mass': 5.29e22,
            # 0.1962 Pa on a 20 km H at the 9.81 its own mass and radius give is
            # 1e-6 kg/m3 at the datum
            'surface': {'atmosphere': {'sea_level_pressure': 0.1962,
                                       'scale_height': 20000.0, 'height': 400000.0}}}
    giant = {'name': 'Biggy', 'radius': 7.0e7, 'mass': 1.9e27,
             # 107,092.8 Pa on a 27 km H at the 25.88 its own mass and radius give is
             # 0.153 kg/m3 at the datum
             'surface': {'atmosphere': {'sea_level_pressure': 107092.8,
                                        'scale_height': 27000.0, 'height': 390000.0}}}
    # The dedupe key is a list of fields, so it gets a list of tests: for each field, a
    # TWIN that differs in nothing else. Drop that field from the key and the twin
    # collapses into its original -- which is the silence the key exists to prevent,
    # since shipped system files repeat columns and a real disagreement must get through.
    # `name` is in the key to LABEL a row, not to tell columns apart (same physics under
    # two names is one column), so it deliberately gets no twin.
    # No `g` twin: a file states gravity once, as mass and radius, and g follows from
    # them (#708) -- the two columns that move it already have twins of their own.
    def twin(field, value, base=None):
        base = base or thin
        if field in ('mass', 'radius'):
            return dict(base, **{field: value})
        return dict(base, surface={'atmosphere': dict(
            base['surface']['atmosphere'], **{field: value})})
    twins = [twin('mass', 1.06e23), twin('radius', 8.0e5),
             twin('height', 500000.0), twin('scale_height', 30000.0),
             # 0.4 Pa on the same column is 2.039e-6 kg/m3, so the twin is still a twin
             twin('sea_level_pressure', 0.4),
             # M comes from the mix, and the level rows are the last key field; both
             # need a layered base, since an exponential body has no rows to differ in.
             twin('composition', [{'gas': 'CO2', 'fraction': 1.0}], base=fat),
             twin('layers', [{'alt': 0.0, 'T': 300.0, 'lapse': -0.0065}], base=fat)]
    bad, weighed, waived = check_docs(
        [('selftest', [fat, thin, giant] + twins)], known={}, verbose=False)
    expect(3 + len(twins), weighed, 'every scale, and every column the key can tell apart')
    expect(0, waived, 'nothing is waived when no waiver is filed')
    expect(2 + len(twins), bad, 'every out-of-band body reaches the count')
    # And a waiver run end to end, VERBOSE, so the line that prints a waiver's ARGUMENT
    # executes: it splits decide()'s message on a newline the two sides must agree
    # about, and nothing else here would notice them drifting apart.
    waivy = dict(fat, name='Waivy')
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        bad2, weighed2, waived2 = check_docs(
            [('waived', [waivy])],
            known={'Waivy': (1000.0, 'a column, not a cut')}, verbose=True)
    expect(0, bad2, 'a value waiver shelters its own number end to end')
    expect(1, waived2, 'a waiver is counted, not just skipped')
    expect(True, 'a column, not a cut' in out.getvalue(), 'the waiver argument is printed')
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
    bad, weighed, waived = check(paths)
    if bad:
        # Say how many were weighed too: a gate that goes quiet about its own coverage
        # is how a body stops being checked without anything looking red.
        print('check_atmo_band: %d of %d air columns out of #682\'s %g..%g Pa band (%d '
              'waived). A top is a decision (shellEdge, spawn beds, the atlas disk); '
              'author one where the air still pushes, or waive it BY VALUE with the '
              'reason in KNOWN.' % (bad, weighed, Q_LO, Q_HI, waived))
        return 1
    print('check_atmo_band: %d air columns weighed -- %d inside %g..%g Pa of orbital q '
          'at their own ceiling, %d waived, 0 out'
          % (weighed, weighed - waived, Q_LO, Q_HI, waived))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
