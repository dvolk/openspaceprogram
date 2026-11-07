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
        "gate, and would leave a hull in a 150 km orbit inside 77 kPa of q all the "
        "same -- the column (65 kg/m3 at the datum, H 16 km) is the problem, not the "
        "cut. Fixing it means a shorter H or a cooler Venus, which is a datum "
        "decision check_fact_air argues about, not a top. Waived BY VALUE: change "
        "the 160 km by more than 2% and this excuse no longer applies."),
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


def in_band_q(pr, target):
    """Where a circular orbit here would feel `target` Pa -- the fix the refusal
    offers. q falls with altitude, so keep the bracket q(lo) > target > q(hi).
    Advisory text, so it runs only on a refusal and on a coarse column."""
    lo, hi = 1.0, pr.radius * 0.2
    if q_at(pr, hi, 400) > target:
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


def check(paths, known=KNOWN, verbose=True):
    bad = 0
    seen = set()
    for path in paths:
        for b in ap.read_system(path)['bodies']:
            su = b.get('surface') or {}
            at = su.get('atmosphere')
            if not at or 'sea_level_density' not in at:
                continue
            pr = ap.Profile(b)
            # The catalogue generates the same columns into several system files; a
            # column is weighed once.
            key = (pr.name, pr.radius, pr.g, pr.rho0, pr.H, pr.top,
                   tuple((l['alt'], l['T'], l['lapse']) for l in pr.levels))
            if key in seen:
                continue
            seen.add(key)
            q = ceiling_q(pr)
            ok, tag, msg = decide(pr.name, pr.top, q, known,
                                  lambda target: in_band_q(pr, target))
            if verbose:
                shape = 'table' if pr.levels else ('H %6.0f m' % pr.H)
                print('  %-9s top %8.0f m  %-12s q %11.1f Pa   %s'
                      % (pr.name, pr.top, shape, q, tag))
            if not ok:
                print('  REFUSE %s' % msg)
                bad += 1
    return bad


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
    bad = check(paths)
    if bad:
        print('check_atmo_band: %d body/ bodies out of #682\'s %g..%g Pa band. A top is '
              'a decision (shellEdge, spawn beds, the atlas disk); author one where the '
              'air still pushes, or waive it BY VALUE with the reason in KNOWN.'
              % (bad, Q_LO, Q_HI))
        return 1
    print('check_atmo_band: every air body lands inside %g..%g Pa of orbital q at its own '
          'ceiling (%d waiver/s filed)' % (Q_LO, Q_HI, len(KNOWN)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
