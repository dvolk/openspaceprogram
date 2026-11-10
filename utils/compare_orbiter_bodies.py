#!/usr/bin/env python3
"""compare_orbiter_bodies.py -- cross-check our solar system against a second authored corpus.

ORBITER below is transcribed from Orbiter's real-solar-system configs
(``../references/orbiter/Src/Celbody/Vsop87/*/Config/*.cfg``, plus the ``Config/Titan.cfg`` that
ships inside the Titan texture pack) on 2026-11-10, and kept INLINE so the check runs without that
tree present and without a download. Two rules about these figures:

  * They are a CROSS-CHECK ONLY. Nothing here is read by the game, nothing is written into
    ``res/``, and no value here is ever copied into our data. A mismatch is a prompt to go find a
    primary source of our own and fix OUR number on our own reasoning -- not a number to adopt.
  * They are frozen as transcribed. If their corpus changes upstream, this table goes stale; that is
    fine, the point is a stable second opinion, not a live dependency.

Why the check is worth running: it is human-authored numbers from a different lineage than the RSS/
fact-sheet corpus our generator reads, so an agreement is corroboration and a disagreement is a
question. Run it after touching atmosphere authoring (#740) or any body's spin (#758).

    python3 utils/compare_orbiter_bodies.py
"""
import json
import math
import re
import sys

K_GAS_R = 8.314462618   # [J/(mol K)], the same constant src/constants.h carries

ORBITER = {
 "Earth": {
  "Size": "6.37101e6",
  "Mass": "5.973698968e+24",
  "SidRotPeriod": "86164.10132",
  "Obliquity": "0.4090928023",
  "AtmGasConstant": "286.91",
  "AtmGamma": "1.4",
  "AtmDensity0": "1.293"
 },
 "Jupiter": {
  "Size": "6.9911e+7",
  "Mass": "1.8986111e+27",
  "SidRotPeriod": "13500.3",
  "Obliquity": "0.05443758224",
  "AtmGasConstant": "194.92",
  "AtmGamma": "1.3333",
  "AtmAltLimit": "3200e3",
  "AtmPressure0": "201.4e4",
  "AtmDensity0": "1.3293"
 },
 "Mars": {
  "Size": "3.39e6",
  "Mass": "6.418542e+23",
  "SidRotPeriod": "88642.66435",
  "Obliquity": "0.4397415938",
  "AtmGasConstant": "188.92",
  "AtmGamma": "1.2941",
  "AtmAltLimit": "280e3",
  "AtmPressure0": "0.61e3",
  "AtmDensity0": "0.02"
 },
 "Mercury": {
  "Size": "2.4394e6",
  "Mass": "3.301880e+23",
  "SidRotPeriod": "5067034.512",
  "Obliquity": "0.0005931973805"
 },
 "Neptune": {
  "Size": "2.4624e7",
  "Mass": "1.024569e+26",
  "SidRotPeriod": "57996",
  "Obliquity": "0.49712071",
  "AtmGasConstant": "144.92",
  "AtmGamma": "1.3333",
  "AtmAltLimit": "1800e3",
  "AtmPressure0": "41.4e4",
  "AtmDensity0": "0.5093"
 },
 "Saturn": {
  "Size": "5.8232e7",
  "Mass": "5.6846272e+26",
  "SidRotPeriod": "38362.40004",
  "Obliquity": "0.4665349048",
  "AtmGasConstant": "174.92",
  "AtmGamma": "1.3333",
  "AtmAltLimit": "2900e3",
  "AtmPressure0": "75.4e4",
  "AtmDensity0": "0.7293"
 },
 "Sun": {
  "Size": "6.96e8",
  "Mass": "1.9889194444e+30"
 },
 "Uranus": {
  "Size": "2.5362e7",
  "Mass": "8.6832054e+25",
  "SidRotPeriod": "-62064",
  "Obliquity": "1.436025521",
  "AtmGasConstant": "194.92",
  "AtmGamma": "1.3333",
  "AtmAltLimit": "2600e3",
  "AtmPressure0": "45.4e4",
  "AtmDensity0": "0.5293"
 },
 "Venus": {
  "Size": "6.05184e6",
  "Mass": "4.86855374e+24",
  "SidRotPeriod": "-20996797.02",
  "Obliquity": "0.021624781",
  "AtmGasConstant": "188.92",
  "AtmGamma": "1.2857",
  "AtmAltLimit": "600e3",
  "AtmPressure0": "9200e3",
  "AtmDensity0": "65"
 },
 "Titan": {
  "Size": "2.575e6",
  "Mass": "1.35e23",
  "SidRotPeriod": "1377686.736",
  "Obliquity": "0.005036124126",
  "AtmGasConstant": "285.92",
  "AtmGamma": "1.3941",
  "AtmAltLimit": "600e3",
  "AtmPressure0": "146e3",
  "AtmDensity0": "6.131"
 }
}

SYSTEM = 'res/systems/solar_system.json'
SPECIES_HEADER = 'src/atmo.h'


def our_species_table():
    """{{name: (molar_mass_kg_per_mol, gamma)}} straight from src/atmo.h's GasSpecies table.

    Read from the header rather than restated, so this script cannot drift from the one place our
    species properties live (atmo.h:406).
    """
    txt = open(SPECIES_HEADER, encoding='utf-8').read()
    out = {}
    for m in re.finditer(r'\{\s*"(\w+)",\s*([0-9.eE+-]+),\s*([0-9.]+),', txt):
        out[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    if not out:
        raise SystemExit('no GasSpecies rows parsed from %s -- the table moved, fix this reader'
                         % SPECIES_HEADER)
    return out


def mix_properties(comp, species):
    """[(molar_mass, R_specific, gamma)] for an authored mix, mole fractions as written.

    gamma for the mixture follows the ideal-gas route src/atmo.h:495 takes for cp:
    cp_mix = sum(x * R * g/(g-1)), then g_mix = cp/(cp - R).
    """
    M = sum(c['fraction'] * species[c['gas']][0] for c in comp)
    cp = sum(c['fraction'] * K_GAS_R * species[c['gas']][1] / (species[c['gas']][1] - 1.0)
             for c in comp)
    return M, K_GAS_R / M, cp / (cp - K_GAS_R)


def main():
    species = our_species_table()
    ours = {b['name']: b for b in json.load(open(SYSTEM))['bodies']}
    flags = []
    print('%-8s  %8s  %8s  %8s  %17s  %9s  %9s  %9s  %7s  %s'
          % ('body', 'R ours/th', 'M ours/th', 'spin our/th', 'tilt ours/theirs', 'ceil m',
              'their ceil', 'Rspec o/t', 'gamma t', 'flags'))
    for name, t in ORBITER.items():
        o = ours.get(name)
        if o is None:
            flags.append('%s: not in our system file' % name)
            continue
        f = lambda k: float(t[k]) if k in t else None
        rot = o.get('rotating', {})
        per = 2 * math.pi / rot['rot_ang_speed'] if rot.get('rot_ang_speed') else None
        tilt = math.degrees(rot['axial_tilt']) if rot.get('axial_tilt') is not None else None
        ttilt = math.degrees(f('Obliquity')) if f('Obliquity') is not None else None
        at = o.get('surface', {}).get('atmosphere', {})

        def rat(a, b):
            return a / b if a and b else None

        rr, mr, pr = rat(o['radius'], f('Size')), rat(o['mass'], f('Mass')), rat(per, f('SidRotPeriod'))
        cells = ['%-8s' % name]
        for v in (rr, mr, pr):
            cells.append('%8s' % ('-' if v is None else '%.4f' % v))
        # Retrograde is spelled two ways: we put the tilt past 90 deg and spin prograde; they keep
        # the tilt under 90 and give a NEGATIVE SidRotPeriod. Both describe the same rotation, so
        # compare the tilt to whichever reading applies.
        flip = (tilt is not None and ttilt is not None and f('SidRotPeriod') is not None
                and f('SidRotPeriod') < 0)
        ttilt_cmp = 180.0 - ttilt if (flip or (tilt is not None and tilt > 90.0 and ttilt is not None
                                               and ttilt < 90.0)) else ttilt
        cells.append('%9.2f/%-7.2f' % (tilt if tilt is not None else -1,
                                          ttilt_cmp if ttilt_cmp is not None else -1))
        ceil, tceil = at.get('height'), f('AtmAltLimit')
        cells.append('%9s' % ('-' if not ceil else '%.0f' % ceil))
        cells.append('%9s' % ('-' if not tceil else '%.0f' % tceil))
        gspec = gtheirs = None
        if at.get('composition'):
            M, Rs, g = mix_properties(at['composition'], species)
            gspec, gtheirs = Rs, f('AtmGasConstant')
            cells.append('%9s' % ('-' if not gtheirs else '%.3f' % (Rs / gtheirs)))
            cells.append('%7s' % ('-' if not f('AtmGamma') else '%.3f' % f('AtmGamma')))
        else:
            cells.append('%9s%7s' % ('-', '-'))
        note = []
        if rr and abs(rr - 1) > 1e-3: note.append('R')
        if mr and abs(mr - 1) > 1e-3: note.append('M')
        if pr and abs(abs(pr) - 1) > 0.01: note.append('spin')
        if tilt is not None and ttilt_cmp is not None and abs(tilt - ttilt_cmp) > 0.05:
            note.append('tilt')
        if ceil and tceil and abs(ceil / tceil - 1) > 0.05: note.append('ceiling')
        if gspec and gtheirs and abs(gspec / gtheirs - 1) > 0.01: note.append('Rspec')
        flags.append('%s: %s' % (name, ','.join(note)) if note else '')
        cells.append(' '.join(note) if note else 'ok')
        print('  '.join(str(c) for c in cells))
    print('')
    print('Definitional mismatches, NOT disagreements -- do not "fix" these:')
    print('  * Venus and Uranus: they keep obliquity < 90 deg and NEGATE SidRotPeriod; we put the')
    print('    tilt past 90 deg and spin prograde. The table compares against the equivalent reading.')
    print("  * Jupiter SidRotPeriod = 13500.3 s (3.75 h) matches none of Jupiter's measured rotation")
    print('    systems (~9.9 h); their own file notes it is a cloud-layer figure. Ours is the better')
    print('    authority here, so this row is expected to flag.')
    print('  * Their AtmPressure0 for the giants (e.g. Jupiter 201.4e4 Pa) is a deeper datum than our')
    print('    authored 1 bar, so pressures are deliberately not compared.')
    bad = [x for x in flags if x]
    print('')
    print('FLAGS %d' % len(bad))
    for x in bad:
        print('  ' + x)
    print('A flag is a question to answer from a primary source of OUR OWN, not a value to copy.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
