# Comets: what the data supports, and a roster to pick from

2026-11-12. **This is a proposal, not a change.** Nothing in `res/systems/` mentions a
comet; which comets fly is the decision left for the morning.

What is already committed (`e720e4f9`): the MPC's `allcometels.json.gz` (200 KB, tracked
unlike the 175 MB MPCORB blob), `utils/rss/asteroids/comets.py` deriving `comets.json`,
and the source wired into `fetch.py` / `urls.txt`.

## The gate: a comet only enters the corpus with a *stated* nucleus radius

MPC's `H` for a comet is the brightness of the **coma** — how much gas the nucleus is
throwing off — and `G` (usually 4.0) is the slope that turns that into a
heliocentric-distance law. The asteroid trick used over 419 rows,
`D = 1329/√pV · 10^(−H/5)`, is not imprecise here, it is meaningless: Halley sizes to
~20 km at pV 0.04 against a flyby-measured 5.75 km, and an active and a dormant nucleus of
the same size differ by orders of magnitude in brightness.

So a radius has to be stated somewhere. Three places, all Wikipedia, all already cached:

| source | what it states | rows |
|---|---|---|
| `raw/objects_by_size.html` (already parsed by `sizes.py`) | nucleus **radius** + sometimes mass | 12 comets |
| Wikipedia `List_of_periodic_comets` | a `Nucleus radii` column | 35 of ~206 rows |
| Wikipedia `Great comet` | a `Dimensions (km)` column — a **diameter**, not a radius | 100 rows, ~20 usable |

Together: **57 identities with a stated radius, 51 of them with a closed MPC orbit.**
The MPC file itself has 4,666 rows over 4,619 identities, and **2,694 of those identities
have no closed orbit at all** (2,351 rows state `e` exactly 1.0 — single-sighting ancient
comets, where a parabola *is* the whole solution — and 354 are hyperbolic). There is no
comet census to be had from orbits alone.

## The nine the derivation already produces

| comet | family | radius m | q AU | e | period y | apoapsis AU | r at J2000 AU | i deg |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Halley's Comet (1P/Halley) | halley-type | 5750 | 0.573 | 0.96794 | 75.5 | 35.1 | 25.42 | 162.2 |
| Schwassmann–Wachmann 1 (29P) | jupiter-family | 30200 | 5.798 | 0.04289 | 14.9 | 6.3 | 6.14 | 9.4 |
| Tempel 1 (9P/Tempel) | jupiter-family | 3000 | 1.751 | 0.46893 | 6.0 | 4.8 | 4.29 | 10.6 |
| Borrelly (19P/Borrelly) | jupiter-family | 2660 | 1.308 | 0.63752 | 6.9 | 5.9 | 4.52 | 29.3 |
| Wild 2 (81P/Wild) | jupiter-family | 2133 | 1.595 | 0.53783 | 6.4 | 5.3 | 5.21 | 3.2 |
| Churyumov–Gerasimenko (67P) | jupiter-family | 2000 | 1.212 | 0.64956 | 6.4 | 5.7 | 5.52 | 3.9 |
| Hartley 2 (103P/Hartley) | jupiter-family | 570 | 1.066 | 0.69336 | 6.5 | 5.9 | 5.40 | 13.6 |
| LINEAR (252P/LINEAR) | jupiter-family | 300 | 1.004 | 0.67115 | 5.3 | 5.1 | 1.39 | 10.4 |
| Great Comet of 1811 (C/1811 F1) | long-period | 17500 | 1.063 | 0.99483 | 2944 | 409.8 | 253.72 | 104.8 |

Mass is stated for only three: 67P 9.98e12 kg, Halley 2.20e14 kg, Hartley 2 3.00e11 kg.
The other six would need a density rule, i.e. an invented mass stamped `INVENTED:`.

Refused by name and printed, never clamped: Bernardinelli–Bernstein (the MPC's own
solution is `e = 1.00421`), 3I/ATLAS (`e = 6.12`, interstellar), Tsuchinshan–ATLAS
(`e = 1.00017`).

## The rest of the pool, by family (beyond the nine)

| family | notable with a stated radius and a closed rail |
|---|---|
| jupiter-family (32) | P/2012 B1 (R 37 km), P/2015 PD229 (21 km), P/2021 N2 (14 km), P/2004 FY140 (10 km), P/1999 XN120 (9 km), P/2014 L2 (7.3 km) — plus ~20 sub-3 km objects with discovery-style names only |
| halley-type (6) | C/2014 OG392 (10 km), C/2011 P2 (8 km), C/2023 RS61 (4.7 km), P/2011 S1 (4 km) |
| long-period (6) | **Hale–Bopp C/1995 O1 (R 30 km, 10.2 AU at J2000)**, McNaught C/2006 P1 (12.5 km, 20.5 AU), Bennett C/1969 Y1 (3.8 km, 51 AU), NEOWISE C/2020 F3 (2.5 km, 41 AU) |
| sungrazer (5) | Great Comet of 1882 (R 31 km, q 0.012 AU), Great March Comet C/1843 D1 (7.9 km, q 0.006), Ikeya–Seki C/1965 S1 (4.3 km, q 0.010), Lovejoy C/2011 W3 (0.1–0.25 km, q 0.004), P/1999 J6 (20 m, q 0.045) |
| disintegrated (2) | D/1766 G1 (3 km, lost — this is Messier 2's comet), D/1978 R1 |

Famous names with **no** radius anywhere we hold: 2P/Encke (3.3 y period, q 0.34 — the
shortest-period comet there is), 109P/Swift–Tuttle, 55P/Tempel–Tuttle, 96P/Machholz,
D/1770 L1 Lexell, D/1993 F2 Shoemaker–Levy 9 (19 fragment rows), 12P/Pons–Brooks,
1I/\`Oumuamua. Adding any of them means inventing a size.

## What I would add, and the variety each one buys

- **29P/Schwassmann–Wachmann 1** — `e = 0.043`, a nearly circular 5.8–6.3 AU orbit. It is
  the only comet in the sky you can reach without a transfer window, and it is famous for
  being *always on* (repeated outbursts). Best "first comet" in the game.
- **252P/LINEAR** — 300 m, `q = 1.004` AU, 1.39 AU from the Sun at J2000. The intercept and
  landing target; small enough that the lander-vs-rock contrast is the whole story.
- **67P/Churyumov–Gerasimenko** — stated mass *and* radius, low-inclination, 5.5 AU at
  J2000. The one body where the density is a measurement rather than an assumption.
- **1P/Halley** — retrograde, `i = 162°`, the only comet that makes the player care about
  orbital inclination. Also the cheapest real-world analogue for a "return" plot.
- **C/1995 O1 Hale–Bopp** — the biggest reachable nucleus (30 km) at 10 AU at the epoch,
  on a 2,400-year orbit. A deep-space destination that is not absurdly far.
- **One Kreutz sungrazer (C/1965 S1 Ikeya–Seki)** — `q = 0.010` AU is 1.5 Gm, about two
  solar radii. That is a "dive close to the Sun" target the rest of the corpus cannot
  offer. If we ever model sublimation or destruction near perihelion, this is where it
  pays off.
- **C/1811 F1** — 254 AU at J2000, apoapsis 410 AU. A pure deep-space curiosity; keeps the
  long-period family represented and tests the outer-universe bound.

That is seven. Tempel 1, Wild 2 and Borrelly (the flyby comets) are cheap to include and
worth having in `named`/`full` rather than `core`.

## Consequences worth deciding about before any of it ships

1. **A comet with no activity model is a grey rock.** Everything above is a body with an
   orbit and a radius. Minimum viable "comet-ness", cheapest first: (a) accept it and let
   the dossier say what it is; (b) an anti-sunward tail billboard sized by heliocentric
   distance; (c) sublimation-driven outgassing — mass loss, a tiny non-gravitational kick,
   and meteor showers when a planet crosses an old trail. (c) uses the *same* insolation
   law as the surface-temperature work, so the temp model should land first and comets
   borrow it rather than inventing a second flux calculation.
2. **`type_` needs a real Branch.** `src/terrain.h`'s `BodyType` and `src/system.cpp`'s
   whitelist are where `Asteroid`/`DwarfPlanet` went; `comet` wants the same (enum value,
   whitelist entry, shader case, dossier word) rather than riding in as an asteroid,
   because the surface material, the tail and the science readout all want to branch on it.
3. **The epoch position is a placement, not an ephemeris.** Rails are osculating at the
   apparition nearest J2000, and `r at J2000` propagates that with two-body motion. For
   1811 that is 189 years of extrapolation on a 2,944-year orbit, and real long-period
   comets get non-gravitational kicks from their own outgassing. Fine for a game; it
   belongs in the provenance string, not hidden.
4. **Interstellar bodies cannot be authored at all** — the loader writes closed Keplerian
   rails. A visitor like \`Oumuamua (`e = 1.20`) needs a hyperbolic rail, which is adjacent
   to the Kepler-solver cleanup in #784.
5. **Names collide.** The shipped `full` file already contains bodies named `LINEAR`,
   `Chiron` and `Echeclus`. Chiron (2060) and Echeclus (60558) are dual-registered: they
   are *also* comets 95P and 174P. Adding them as comets would put two bodies with one name
   at two different orbits in the same system. Needs a rule before it happens — probably:
   the corpus keeps the asteroid registration, and a comet lists its designation
   (`252P/LINEAR`) instead of a bare name.
6. **#785 bites comets hardest.** A 300 m nucleus inherits the metre-rock problem: a tiny
   SoI and a very long period, so "land on the comet" has the same physics wall as the
   minimoons.
7. **A wiki join can attach the wrong orbit.** The `Great comet` table gives Hyakutake as
   `C/1996 B1`; the MPC's `C/1996 B1` is a different comet (Hyakutake is `C/1996 B2`). Any
   parser for that page needs the same identity refusal `parse.sbdb_identity_ok()` already
   implements, or a wrong orbit arrives silently.

## Decisions

1. Roster: the seven above, more, or fewer?
2. Comet-ness: does anything beyond (1a) get built before the roster grows past ~10?
3. Famous but size-less (Encke, Swift–Tuttle, Lexell, Shoemaker–Levy 9): invent and stamp
   `INVENTED:`, or leave them out?
4. Is one interstellar visitor worth a hyperbolic-rail change to the loader?
5. Dual-registered names (`Chiron`/95P, `Echeclus`/174P): rename, skip, or allow two
   bodies with the same name?

Scratch for all of the above: `tmp/mpc/comet_probe*.py`, `comet_join.py`,
`comet_candidates.py`, `comet_pool.py` (the 57-identity merge), `comet_pool.txt`. The two
comet wiki pages are cached in `tmp/mpc/` only — adopting their radii means moving them to
`raw/` and giving them a parse step.
