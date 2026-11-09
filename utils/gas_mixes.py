#!/usr/bin/env python3
"""The named gas mixes the system generators author with (#636).

A body's air is ONE authored statement -- a mix of gases -- and the game's loader
derives everything else from it: molar mass, gamma, and the oxidizer share every
air-breathing engine is gated on (src/atmo.h:resolveMix). That argument only holds
while the mix is written once. Until now it was written twice, in
utils/gen_systems.py and utils/make_solar_system.py, and each generator compares a
file against its OWN output, so editing CO2_HEAVY in one of them leaves both
--check passes green while ksp_system.json and solar_system.json quietly disagree
about what "a CO2 world" means -- and with it, whether a jet runs there.

ONE RULE FOR NAMES: one name is one sky. Two mixes with different fractions must
not share a name, which is how this file earned its suffixes -- both generators
used to carry an H2_HE_CH4 with different numbers (Jool's and Uranus's), and
neither could see the other.

Fractions are by MOLE and need not sum to 1: the loader normalizes them by their
own total. The species names must exist in the game's gas table
(src/atmo.h:gasSpecies); tests/test_scenarios.cpp refuses a name the game does not
know, so a typo here fails at load rather than becoming a silent default. Python
names gases and fractions only -- the species PROPERTIES (mass, gamma, oxidizer)
stay in C++, which is the split that test pins.

Not every authored mix lives here: a mix that one catalogue measures for one body
stays in the generator that measured it (Titan's N2_METHANE, Venus's near-pure CO2).
What belongs here is what two generators could each invent under the same name: the
shared skies, and the ones that share species but not a sky.
"""

# Kerbin (gen_systems) and Earth (make_solar_system). This is the reference air:
# the jet gate's rating intake is standard sea-level density, and dry air's
# oxidizer share (0.2314 by mass) is what that gate divides by -- which is why a
# body that authors no mix at all gets atmo.h:dryAirMix, and why the authored mix
# here has to be the same gas rather than a near miss.
DRY_AIR = [{"gas": "N2", "fraction": 0.78084},
           {"gas": "O2", "fraction": 0.20946},
           {"gas": "Ar", "fraction": 0.00934},
           {"gas": "CO2", "fraction": 0.00033}]

# Eve and Duna (gen_systems) and Mars (make_solar_system). Dense enough to breathe
# for drag purposes, and only a 0.13% oxygen trace -- the reason a jet on any of
# them is a hole in the hull rather than a weak engine.
CO2_HEAVY = [{"gas": "CO2", "fraction": 0.9532},
             {"gas": "N2", "fraction": 0.027},
             {"gas": "Ar", "fraction": 0.016},
             {"gas": "O2", "fraction": 0.0013}]

# Jupiter (make_solar_system). Hydrogen and helium, nothing to burn anything with.
H2_HE = [{"gas": "H2", "fraction": 0.898},
         {"gas": "He", "fraction": 0.102}]

# Uranus (make_solar_system), from the same RSS tables as Jupiter's mix.
H2_HE_CH4_SOLAR = [{"gas": "H2", "fraction": 0.825},
                   {"gas": "He", "fraction": 0.152},
                   {"gas": "CH4", "fraction": 0.023}]

# Jool (gen_systems). The deepest column in that system -- a 1 bar datum under a
# 600 km ceiling -- and 0% oxidizer: a fuel store, not a sky a jet can cross. Its
# datum is only 0.19 kg/m3 (#625 put it at the 1 bar level the giants' published rows
# are stated at); Eve's compressed 1.96 atm of CO2 at 1.7 g is the densest air in that
# file at 3.6 kg/m3.
H2_HE_CH4_KSP = [{"gas": "H2", "fraction": 0.855},
                 {"gas": "He", "fraction": 0.14},
                 {"gas": "CH4", "fraction": 0.005}]
