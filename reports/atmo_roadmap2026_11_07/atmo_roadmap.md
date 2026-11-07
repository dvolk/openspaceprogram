# Where the atmosphere is going

`reports/atmo_roadmap2026_11_07/atmo_roadmap.md`

A statement of position, not a work order. The question it answers: **if a planetary
atmosphere scientist or a spaceflight engineer booted this game and read its
instruments, what would they say — and what would we have to build to stop them saying
the bad things?** Written for future reference, so it states numbers as they measured on
2026_11_07 and says where each came from. `support/datum_table.py` in this directory
resolves both shipped systems through `atmo.h`'s arithmetic and scrapes the fact sheets,
and its output is committed beside it as `datum_table.out`: **every datum, delta and
per-body multiplier in §3, §3b and §4 is that output.** What the script does *not* produce
is stated where it appears — the wind and diurnal figures §3 quotes are read from the
sheet prose by hand (the script scrapes them for Venus/Earth/Mars only), the "~24 % high
Mach" and "84×" ratios are one-line derivations from numbers it does print, and the option
B/C comparison in §4 is a separate calculation in the script's section 3. Nothing is
quoted from memory.

Companion snapshots: `reports/atmo_references2026_11_06/` (what Orbiter and Pioneer
author, and our three findings #682/#683/#684 against them).

---

## 1. The acceptance test, written so it can actually be applied

The stated bar is: an expert should not say **"that's nonsense"**, and should not say
**"they're not simulating this really important thing that changes things
significantly."**

Those are two different refusals and they cost completely different things, so the first
job is to separate them.

| | what fails | how it fails | cost to fix | does it ever end |
|---|---|---|---|---|
| **A. form** | the equations | right physics, self-contradictory or wrong physics | cheap, one-time, permanent | yes |
| **B. constants** | the numbers | right physics, wrong world | cheap per body, bounded by real data | yes, body by body |
| **C. phenomena** | the ontology | a thing that exists in the world has no home in the code | expensive, unbounded | **no** |

Everything on the wishlist ("climate, wind, seasons, deserts, summers, humidity, rain")
is C. C is where projects die, because for every phenomenon you ship there is another
one. So the strategic claim of this document is:

> **A and B are nearly done and cheap to finish. The defensibility of this game rests on
> choosing, in advance and out loud, which C items we own and which we disclaim.**

An expert forgives a simplification that is *named*. They do not forgive one that is
invisible — because an invisible simplification means the model is claiming to be the
thing it isn't, and then every readout downstream is untrustworthy.

---

## 2. Where we are, in one paragraph

We have a **per-body, vertical, steady** atmosphere: for each body, density,
temperature, pressure, sound speed and composition as functions of altitude above that
body's datum, derived from a small set of authored numbers by hydrostatics and the ideal
gas law, with a loader that refuses profiles that contradict themselves. It is
**internally consistent to the point of being airtight** — that is the trio lock, the
seam check, the overturn refusal, the composition resolution. It has **no horizontal
structure and no time dependence at all**: the air at Kerbin's equator at noon is the
same air as at its pole at midnight, and always has been. Its datums for the **real
terrestrial** worlds (Venus, Earth, Mars) are within 0.0–4.9 % of published values on
temperature, 0.0–4.5 % on scale height and under 0.1 % on molecular weight, and its Mars
pressure and density sit inside the range the source itself publishes — but its **giant**
densities are off by −47 %, +43 % and +100 % against their own fact sheets (§3b), so "the
real worlds" is not one verdict. Its datums for the **fictional** worlds were chosen as
scale heights and have never been read back as surface temperatures — and one of them,
Kerbin's, implies a world at 188 K.

---

## 3. What the game asserts today (measured)

From `res/systems/solar_system.json` and `res/systems/ksp_system.json` resolved through
the loader's own arithmetic (`support/datum_table.py`). The game boots
**ksp_system.json** by default (`src/cli.h`), so the fictional column is the one players
breathe.

| system | body | form | g | rho0 | H | M g/mol | gamma | **T0 K** | **p0** | **a m/s** | air top |
|---|---|---|---|---|---|---|---|---|---|---|---|
| solar | Venus | exponential | 8.87 | 65 | 16 km | 43.45 | 1.3026 | 741.6 | 9.22 MPa | 430.0 | 160 km |
| solar | Earth | **7 layers** | 9.82 | 1.225 | — | 28.965 | 1.4015 | 288.15 | 101.3 kPa | 340.5 | 85 km |
| solar | Mars | exponential | 3.73 | 0.02 | 11.5 km | 43.496 | 1.3049 | 224.4 | 858 Pa | 236.6 | 115 km |
| solar | Jupiter | exponential | 25.92 | 0.16 | 27 km | 2.219 | 1.4170 | 186.7 | 112 kPa | 995.8 | 270 km |
| solar | Saturn | exponential | 11.19 | 0.1 | 60 km | 2.150 | 1.4047 | 173.5 | 67 kPa | 970.9 | 600 km |
| solar | Uranus | exponential | 9.01 | 0.6 | 20 km | 2.640 | 1.4233 | 57.2 | 108 kPa | 506.4 | 200 km |
| solar | Neptune | exponential | 11.27 | 0.9 | 20 km | 2.601 | 1.4310 | 70.5 | 203 kPa | 568.0 | 200 km |
| solar | Titan | exponential | 1.354 | 5.3 | 20 km | 27.794 | 1.3986 | 90.5 | 144 kPa | 194.6 | 200 km |
| **ksp** | **Kerbin** | exponential | 9.81 | 1.225 | **5.5 km** | 28.965 | 1.4015 | **188.0** | **66.1 kPa** | **275.0** | 70 km |
| ksp | Eve | exponential | 16.68 | 1.7 | 7 km | 43.496 | 1.3049 | 610.7 | 198 kPa | 390.3 | 90 km |
| ksp | Shay | exponential | 8.99 | 1.225 | 6 km | 28.965 | 1.4015 | 188.0 | 66.1 kPa | 275.0 | 60 km |
| ksp | Duna | exponential | 2.943 | 0.12 | 4 km | 43.496 | 1.3049 | 61.6 | 1.41 kPa | 123.9 | 50 km |
| ksp | Jool | exponential | 7.848 | 2.0 | 20 km | 2.364 | 1.4232 | 44.6 | 314 kPa | 472.6 | 200 km |
| ksp | Laythe | exponential | 7.848 | 1.225 | 5.5 km | 28.965 | 1.4015 | **150.4** | 52.9 kPa | 246.0 | 50 km |

**Against the source we already ship.** `utils/rss/html/<body>fact.html` is in the repo,
and `utils/make_solar_system.py:parse_planet()` already scrapes its **Bulk parameters**
section for mass, radius and gravity (`section_values(h, 'Bulk parameters')` at line 318,
alongside `Orbital parameters` and `Mean Orbital Elements`). **It does not scrape the air
— not yet.** Every `sea_level_density`, `scale_height` and `composition` in the shipped
systems is a hardcoded literal in the generator's own planet table (lines 621, 682, 728,
746, 764, 772, 784, 794), so #685 is asking for a *second* scrape of a section already
being read, not for a comparison of two things already in hand. The same sheet text does
carry the air:

| body | our T0 | sheet | off | our p0 | sheet | off | our M | sheet | our rho0 | sheet |
|---|---|---|---|---|---|---|---|---|---|---|
| Venus | 741.6 | 737 K | **+0.6 %** | 9.225 MPa | 92 bars | **+0.3 %** | 43.450 | 43.45 | 65 | ~65 |
| Earth | 288.15 | 288 K | **+0.1 %** | 101,325 Pa | 1014 mb | **−0.1 %** | 28.965 | 28.97 | 1.225 | 1.217 |
| Mars | 224.4 | ~214 K | **+4.9 %** | 858 Pa | 6.36 mb | **+35 %** | 43.496 | 43.49 | 0.02 | ~0.016 ± 0.006 |

So: **on the terrestrial worlds, axis B is essentially finished** at the datum —
0.0–4.9 % on temperature, under 0.1 % on molecular weight, and Mars's +35 % pressure sits
inside the range the sheet itself publishes ("variable from 4.0 to 8.7 mb depending on
season" = 400–870 Pa; our 858 Pa is *in* the real world's own seasonal swing, and its
density is inside the sheet's own ±0.006 error bar). **§3b shows the giants are a separate
and much worse story**, and they are the same scrape away from being fixed.

**And the same block gives us targets we have never used.** The sheets state, in prose we
do not parse: scale height (Venus 15.9 km — ours 16 km; Mars 11.0 — ours 11.5; Earth 8.5 —
our layered profile's effective datum scale height is 8.42 km), mean temperature,
**diurnal temperature range** (Venus ~0; Earth 283–293 K; Mars 184–242 K) and **wind
speeds** (Venus 0.3–1.0 m/s at the surface; Earth 0–100 m/s; Mars 2–7 m/s summer, 17–30
in a dust storm; and on the giants Jupiter up to 150 m/s, Saturn up to 400, Neptune up to
580 — the fastest winds in the solar system, in a file we already download, in a model with
nowhere to put them). Those last are the published size of the two phenomena we do not
simulate.

### 3b. Why the terrestrial worlds land within a percent — and why the giants cannot

Ask each fact sheet whether **its own** numbers form one hydrostatic isothermal column:
take the M, T and gravity it states, compute the scale height they imply
(`H = Rs T / g`), and compare against the scale height the same sheet states. Note the
**T column changes what it reads per family**: the rocky sheets state an "Average
temperature", the giant sheets state no such row, so for the giants this is their
"Temperature at 1 bar" — which is the right comparison anyway, since a giant's datum *is*
its 1-bar level.

| body | M g/mol | T K | g | H implied by its own M, T, g | H stated | agree |
|---|---|---|---|---|---|---|
| Venus | 43.450 | 737 | 8.870 | 15,900 m | 15.9 km | **−0.0 %** |
| Mars | 43.490 | 214 | 3.730 | 10,969 m | 11.0 km | **−0.3 %** |
| Earth | 28.970 | 288 | 9.820 | 8,417 m | 8.5 km | **−1.0 %** |
| Jupiter | 2.220 | 165 | 25.920 | 23,841 m | 27 km | **−11.7 %** |
| Saturn | 2.070 | 134 | 11.190 | 48,099 m | 59.5 km | **−19.2 %** |
| Uranus | 2.640 | 76 | 9.010 | 26,566 m | 27.7 km | **−4.1 %** |
| Neptune | 2.530 | 72 | 11.274 | 20,995 m | 19.1 km (states 19.1–20.3) | **+9.9 %** at its low end; inside the range at M = 2.69 |

That table is the answer to the obvious question — *why are our terrestrial datums within a
few percent while three of our four giant temperatures are 13–30 % off (Neptune, the
fourth, is −2 %)?*

> **The terrestrial sheets publish a self-consistent isothermal column at their datum. The
> giant sheets do not** — a giant is not isothermal over the depth a quoted scale height
> averages, so its stated scale height and its stated 1-bar temperature cannot be one
> constant-g column. **Our loader's trio lock is a stricter standard than NASA's own
> summary for four of its planets.**

Two laws fall out of that, and both belong in #685 rather than in code comments:

- **`H == Rs T / g` may never be an assertion** — not against our files and not against the
  source. The four giant sheets disagree with themselves by **4–19 %** (Jupiter and Saturn
  worst at 12–19 %), while the three terrestrial ones agree to 1 %. (Same meta-law as #684,
  seen from the other side: those three numbers are not independent in the data either.)
- **A single exponential is adequate for the terrestrial worlds and structurally wrong for
  the giants.** On a giant, one of {rho, H, T} must be sacrificed, and today the one we
  sacrifice is always T, silently. The giants want layer tables for a different reason than
  Kerbin does (§4): Kerbin wants one so an authored guess stops silently setting its
  climate; the giants want one because the published data cannot be written in one column
  at all.

And one plain authoring error, distinct from the structural point above. The giant sheets
state their air **at the 1 bar level**, which *is* our datum — their
`gravitymean1barms2` matches our authored `g` to four digits (Jupiter 25.92 vs 25.9192, so
the `G·mass/radius²` fallback at `make_solar_system.py:490` is already producing the
sheet's own value; that turned out **not** to be a bug, and was checked before being
claimed). Comparing density-at-1-bar against our `sea_level_density`:

| body | sheet rho at 1 bar | ours | off | sheet H | ours | off |
|---|---|---|---|---|---|---|
| Jupiter | 0.16 | 0.16 | **+0 %** | 27 km | 27 km | **+0 %** |
| Saturn | 0.19 | 0.1 | **−47 %** | 59.5 km | 60 km | +1 % |
| Uranus | 0.42 | 0.6 | **+43 %** | 27.7 km | 20 km | **−28 %** |
| Neptune | 0.45 | 0.9 | **+100 %** | 19.1–20.3 km | 20 km | inside its range (+4.7 % off the low end) |

Jupiter's density (0.16), scale height (27 km) and M (2.219 vs 2.22) matching their sheet
exactly cannot be coincidence, while Saturn 0.10, Uranus 0.60 and Neptune 0.90 match
nothing in theirs and the generator carries no source comment for those three literals
(`make_solar_system.py:772`, `:784`, `:794`) — so Jupiter was authored from the sheet and
the other three from somewhere else, which is as far as the evidence goes. Also: **Pluto**
publishes a complete atmosphere in its sheet (13 microbar, surface T 24–38 K, H ~18 km
lower / ~50 km above 30 km, M ~28, N2 99 % / CH4 0.5 %) and we author **no density for it
at all**, while **Titan** has no fact sheet to author from. Both are skip cases a guard
must distinguish — "no sheet" is different from "sheet is odd" is different from "number
is missing". Titan is the more interesting of the two: its authored datum (a N2/CH4 mix,
`rho0` 5.3, `H` 20 km) resolves to 90.5 K, 143.6 kPa, sound speed 195 m/s — four times
Earth's surface density at under a third of its temperature, and the pair of numbers looks
Titan-ish rather than invented. But **nothing in this repo can confirm that**: the only
Titan entry we vendor is the bulk-parameters row in `saturniansatfact.html` (radius, mass,
density), with no air at all. Of every atmosphere in the game, Titan's is the one where a
guard is impossible and a source is cheapest, so #685's skip list should say "no sheet —
find one" rather than "skip".

---

## 4. The finding this exercise produced: a scale height *is* a surface temperature

A file cannot author a pressure (`atmo.h`, `drag.h`'s "THE THREE ARE LOCKED"). For a
non-layered body:

```
T0 = g H M / R      p0 = rho0 g H      a = sqrt(gamma g H)
```

so **the authored scale height is the surface temperature**, expressed in metres. Nobody
has been picking it on purpose. Read back:

- **Kerbin: 188 K (−85 °C), 66.1 kPa — two-thirds of Earth's pressure — and a sound speed
  of 275 m/s.** `rho0` is authored at Earth's 1.225 kg/m³, so Kerbin is a world with
  Earth's surface *density*, a third less pressure, and cryogenic air. It has seas
  (`has_sea: true`).
- **Laythe: 150 K air over an ocean.** **Shay: 188 K, with seas.** **Duna: 62 K.** Eve:
  611 K (defensible — Eve is a furnace, and it is the one where a hot datum looks
  intentional).
- These are **pinned goldens**: `tests/test_scenarios.cpp:122` holds Kerbin at T 187.96 K,
  a 274.98 m/s, p 66,094.875 Pa with tol 1e-9 — a **relative** band, scaled by each
  expected value at `tests/test_scenarios.cpp:197-200`, so it is as tight as doubles allow.
  The comment explains the arithmetic beautifully and never once asks whether the world is
  right.

This is the #684 meta-law in its purest form: *a derived quantity has one authored number
behind it, so no internal consistency check can ever see it go wrong.* Kerbin is
internally flawless and externally absurd, and every refusal we have is blind to it
**by construction**.

**What it does not cost.** `drag.h:jetAirGate` reads density and oxidizer share only —
never pressure, never temperature. Lift and drag read density. So a 66 kPa datum changes
no flight performance through the door. What it changes is (a) what a barometer and
thermometer read, which is precisely the expert's test, and (b) **sound speed**: at
275 m/s every Mach number in the default game is ~24 % higher than an Earth-like world
would give at the same true airspeed, and Mach feeds `liftSlopeMach`, `waveCd` and #487's
damping. Fixing the datum is a *Mach-law* change, so the flight goldens will move.

**Two ways to fix it, measured.** Kerbin's air is dry air (M 28.965) at g 9.81, so a
288.15 K datum wants H = 8,432 m.

| at altitude | A: today (H 5.5 km) | B: H 8,432 m | C: Earth's 1976 levels, top 70 km |
|---|---|---|---|
| 0 | 188 K, 1.225 | **288.2 K**, 1.225 | **288.2 K**, 1.225 |
| 11 km | 188 K, 0.166 | 288 K, 0.332 | 216.7 K, 0.364 |
| 20 km | 188 K, 0.0323 | 288 K, 0.114 | 216.7 K, 0.0880 |
| 40 km | 188 K, 8.5e-4 | 288 K, 0.0107 | 251 K, 0.00384 |
| rho at 69,999 m | 3.6e-6 | 3.0e-4 | 7.4e-5 |
| **q at 69,999 m, circular (2.30 km/s at Kerbin's 600 km radius)** | **9.6 Pa** | **801 Pa** | **195 Pa** |

(All three are sampled at 69,999 m, not at 70 km: at exactly `height` the model is vacuum,
`src/drag.h:107`.)

**Copy Earth's table verbatim and the game will not load.** Earth's last level is based at
71 km, and `atmo.h:131` refuses any table whose last level is not based *below* the body's
`height` — with Kerbin's `height: 70000` that is a hard `return false`, which
`src/system.cpp:583-592` turns into a thrown `"atmosphere.layers must start at alt 0, rise
strictly to below height, ..."` at load. So option C means **Earth's first six levels**
(the 51 km level, lapse −0.0028, then lapses through to 70 km), which was checked to
satisfy every other invariant: last base 51 km < 70 km, T still 217.5 K at the ceiling,
lapse −0.0028 well above the overturn limit −g/Rs = −0.0342. The measured column is
unchanged by dropping the 71 km row because the 51 km level is what governs at 69,999 m.
That the obvious authoring is a crash is itself the argument for the harness in §8.

Option B is one number and it fixes the datum — and it puts 84× more air at the edge of
the home world's atmosphere, so a Kerbin low orbit that today sees 10 Pa of dynamic
pressure would see 800 Pa. Option C buys the same correct datum with **4× less** residual
air aloft, because a tropopause and a stratopause let the column thin the way a real one
does. That is the law's own advice, from `atmo.h`: *"having both is a layer table, not a
fudge."*

**And a layer table cannot buy warm *and* thin — the trio lock has an exact local form.**
Differentiating `ρ = p /(Rs T)` against `dp/dh = −ρ g` gives the *density* scale height
inside a level:

```
H_rho = T / (g/Rs + lapse)
```

At a 288.15 K Kerbin datum (`Rs` 287.05, so `g/Rs` = 0.034175 K/m) an isothermal level gives
`H_rho` = 8,432 m — the option-B number. A cooling level (lapse < 0, every real troposphere)
thins the density *more slowly* than that: Earth's troposphere runs at ~10.4 km at sea level,
falling to ~7.8 km by 11 km. To thin at today's 5,500 m **while warm**, the level must
invert at **+18.2 K/km** (0.0182 K/m), reaching 835 K at 30 km. The loader would accept it —
`lapse > −g/Rs` is the only lapse floor — and no atmosphere does it. So a warm Kerbin is a
thicker Kerbin unless it is also a runaway greenhouse: the lock is not a quirk of
`airDensity()`'s one-exponential branch, it is hydrostatics itself. (A comment on #625 states
the same bound as "`H_rho` is minimised at `lapse = 0`", which is true only among
non-inverting profiles — an inversion goes below it, at the cost above.)

> **Recommendation, and it is the single highest-value item in this document:** Kerbin
> gets a layer table. No new physics — JSON plus goldens. It fixes the datum temperature,
> the datum pressure, the sound speed, the "there is no lapse rate anywhere" tell, and
> most of #625's guessed-height problem for the world players actually fly, in one
> authoring decision. Kerbin's frame does not move (top 70 km, and
> `bodylimits.h:shellEdge = max(100 km, 1.2 × top)` is already on its 100 km floor).
>
> **But what Kerbin's surface temperature *is* is a design decision, not a derivation.**
> 188 K may be what Kerbin is. If it is, the world is cold, thin-pressured and its seas
> are ice, and the code should say so somewhere a player can read. What is not
> acceptable is 188 K because 5500 was typed in 2024.

The same question, one level up: **the surface has no thermal existence anywhere in the
game.** `#618` closed with "no buoyancy, no splashdown, no water drag," and there is no
surface temperature, no ground heat flux and no albedo. Every item on the wishlist that
mentions deserts, summers, nights or rain is asking for a surface that thermodynamically
does not exist. **That is the missing prerequisite for all of C, and it should be built
before any of it.**

---

## 5. The ladder, with our rung marked

1. one authored density per body — done, pre-#552
2. one exponential column (rho0, H) — done
3. **a layered temperature profile with hydrostatic integration — done (#552), shipped on exactly one body (Earth)**
4. composition → M, gamma, oxidizer share — done (#630, #636, #637)
5. self-consistency refusals (datum, seams, overturn, top) — done
6. **datums checked against an outside source — #685 filed, not built**
7. **the column sits in the gravity field the hull actually feels (geopotential height) — #683, not built**
8. **where the air ends, and how fast it dies there — #682, open, and it is a decision not a fact**
9. **per-body real profiles instead of one guessed H — #625, open**
   ← **we are here, and Kerbin's rung-9 number is what §4 is about**
10. a surface with a temperature — **absent, prerequisite for everything below**
11. horizontal structure: latitude, altitude-of-terrain, local time — absent
12. wind: an air velocity field — absent (and un-ticketed)
13. obliquity/seasons as an insolation cycle — absent (but see §7: the geometry exists)
14. water: humidity, saturation, clouds, precipitation, latent heat — absent
15. climate as the equilibrium of 10–14 — absent, and should stay absent for years

A scientist landing today would put us at rung 3–5 on form, near rung 6 on the real
worlds' constants, and rung 9 on the fictional ones. The honest headline: **we are one
full rung from being defensible on the worlds that exist, and we are not defensible on
Kerbin, which is the world people will actually judge us by.**

---

## 6. The wishlist, sorted by mechanism rather than by charm

| the wish | the object it needs | exists? | cost | worth it |
|---|---|---|---|---|
| "some places are more windy" | an air **velocity** field | no | **low** — one seam | **yes, first** |
| "mountains are cold" | surface T from altitude | terrain height exists | low–mid | yes |
| "nights cool the air" | sun angle at a point + a lag | **geometry exists** | low–mid | yes |
| "deserts are hot" | surface type (moisture/albedo) | no | mid | later |
| "summers are hotter" | obliquity × orbital position | **authored and applied** | mid | yes |
| "some places are humid / it rains seasonally" | water as a species + saturation curve + condensation | H2O is in the species table; nothing else | **high** | not yet |
| "climate" | the equilibrium of all of the above | no | unbounded | **disclaim it** |

**Cheapest first: wind.** Both reference engines agree wind is an *air velocity*, never a
force term, so it enters at exactly one seam — `Vehicle::airRelativeVel`, where
co-rotation already lives. Nothing else in the aero law changes: hull drag, lift, control
authority, Mach, the jet gate and #498's air-relative prograde all become correct for
free, because they all already read that one vector. And the fact sheets already publish
what to aim at (Earth 0–100 m/s, Mars 2–7 m/s, Venus 0.3–1.0 m/s at the surface). This
is one vector field plus a per-body surface number, and it is the only C item that is a
same-day change.

**The seasons surprise.** The geometry a seasonal cycle needs is already built and
already evaluated. Every body authors `rotating.axial_tilt`, `tilt_azimuth` and
`spin_phase0` (Earth 0.4091 rad = 23.44°, Mars 0.4396 = 25.2°, Venus 3.096 = 177.3°
i.e. retrograde — all right), `src/system.cpp:766` loads them and `src/system.cpp:776-787`
composes them into the body's `initial_orient` / `equator_orient` (`src/frame.h:56-57`
documents what that split means), and `surfmap.cpp:18 sunDirRot()` already computes the
direction **toward the star in the body's rotating frame** for shading the surface map.
Subsolar latitude and hour angle are therefore one dot product from a lat/lon —
`surfmap.h:15` already evaluates `dot(n, sun)` per terrain pixel. What is missing is not
geometry, it is a *flux*: **no code in `src/` turns that angle into W·m⁻².** (Related
oddity found on the way: every body in every generated system file authors
`power_scaler` — 1, 3, ... — and **nothing in `src/` reads it.** An authored per-body
stellar scalar with no reader is the closest thing we have to the thing §9 needs.)

**Water is the deep one, and it should wait.** Humidity is not a function of altitude
alone, so it breaks the "profile" data model itself: it needs a surface source, a
saturation curve against T, condensation with latent heat, and something to hold
clouds. It is also the one place where we would be *inventing* on Kerbin, since the
game's air is dry-Earth air with no water in it. Ship it after 10, 11 and 12 are real.

**Climate should be explicitly disclaimed, forever, in this game.** Real climate is a
coupled ocean–atmosphere–surface model at resolution we could not afford at ten times the
budget. Our version of "climate" is: *an authored surface-temperature field plus a
steady diurnal and seasonal response, tuned to look right.* That is a legitimate thing
to build and a thing a scientist will accept **if we say it**. It is not a thing to
accidentally imply.

---

## 7. Deliberate simplifications — say them out loud

The user's instinct about the air top is right, and the right way to hold it is to
publish it as a choice with a number attached.

**Where the air ends (#682).** Today: `top = authored height, else 10 × scale height`,
flat below the datum, **hard zero** at and above it (pinned by
`tests/test_drag.cpp:10`, "vacuum at and above the hard top (height)"). Gameplay claim:
*we want a top high enough that satellites and aerobrakes feel something, and a density
there low enough that no one has to boost.* Those are two decisions and we currently make
one. The pair is:

- **where** — the user's 100–200 km for Earth is reasonable and matches where real drag
  stops mattering over mission timescales; our Earth is 85 km, our Kerbin 70 km, our
  Venus 160 km;
- **how fast it dies there** — this is the one that decides whether a low orbit decays,
  and it is set by the *shape* (§4: a layer table gave 4× less residual air than an
  exponential at the same datum).

The coupling that orders the work. **#683 makes the air at a body's top denser**, because
gravity falls with altitude and the flat model pretends it does not. #683's own table
measures it per body against the real profile — **Kerbin 3.8×, Eve 4.3×, Duna 5.4×,
Laythe 2.3×, Earth 1.01×** — and for a single isothermal column there is a closed form:
`g(z) = g R²/(R+z)²` integrates to `p = p0 exp[−(R/H)·z/(R+z)]`, so the flat model is
light at altitude z by `exp[(z/H)·z/(R+z)]`. Running that on every shipped body's datum
H_eff gives Jupiter 1.04×, Uranus/Neptune 1.08×, Saturn 1.11×, Earth 1.14×, Venus 1.29×,
Jool 1.38×, Mars 1.39×, Titan 2.06×, Laythe 2.29×, Shay 2.67×, Kerbin 3.78×, Eve 4.33×,
Duna 5.41×. The isothermal form over-corrects Earth (1.14× vs the layered profile's 1.01×)
because a warm stratosphere raises the scale height up there; it is a good estimate for the
small bodies, which are exactly the ones where the effect is large. "2–5×" flattens a range
that is really 1.01–5.4, and the bodies where it bites hardest are the tiny home worlds
players actually live on. The opposite lever is the profile shape: at Earth's 85 km top the
authored layered column is **7.6× thinner in density** (11.8× in pressure) than an
exponential with the same datum and the same top. They pull opposite ways, so **#683 must
be settled before the tops are chosen**, or the tops get re-litigated. Frame note: raising a
top moves the rotating-frame shell (`shellEdge = max(100 km, 1.2 × top)`,
`validateBodyLimits` keeps `shell ≥ top + 10 km`), the LowOrbit band, the HighOrbit cut,
spawn beds and the atlas disk together. Kerbin's frame is already free to hold 70 km —
`shellEdge` is on its 100 km floor (`bodylimits.h:34`). The render sky rim is independent of
all of it (`terrain.h:302`, `datum + max_height + thickness`, repeated for the cloud deck at
`:317`) — moving the air does not move the sky.

**Other simplifications to keep, and to name so nobody re-files them:** one gamma per gas
rather than temperature-dependent; composition uniform with altitude (no homopause, no
diffusion separation); blunt-body Cd reads no Mach (that decision is #635); no buoyancy,
splashdown or water drag (#618, closed); air is not a fluid we integrate — it is a table
we sample, and the game will not ever produce weather from it by itself.

---

## 8. A tolerance policy instead of "a few K is fine"

The proposed ladder — *a few K off is fine, 50 K off is something to look at* — needs
three amendments before it can be used, because as stated it will be applied to the
wrong things.

1. **A tolerance is only meaningful where the compared quantity is well defined.** Earth
   at 288.15 K is a definition (the 1976 standard's sea-level row). Kerbin at 188 K is
   not wrong, it is *unexamined* — there is no Kerbin fact sheet, so axis B does not
   apply and the band is a design intent we have to state, not a measurement.
2. **The band must be at least as wide as the real world's own published variability, or
   we chase noise.** Mars's surface pressure is 4.0–8.7 mb *by season*; its density is
   published as 0.016 ± 0.006. Our Mars is +35 % on pressure and +25 % on density and is
   **inside the real world's own range**. Chasing it to 1 % would be busywork against a
   moving target.
3. **For temperatures, the interesting error is in the range, not the mean.** Mars's
   surface swings 184–242 K in a day (58 K). A model whose mean is exact but whose
   diurnal range is zero is not "28 K off" — it is missing the phenomenon, and by the
   user's own 50 K yardstick it fails harder than a 50 K mean error would.

Proposed bands, per axis B (real bodies only), for the generator guard (#685) and any
future profile check:

| quantity | band | why |
|---|---|---|
| datum density, mean molecular weight | **±30 %** | catches the 92-bar-typed-as-density slip (+42 %, #684) and passes Earth (+0.7 %) — but note it would **fail Saturn (−47 %), Uranus (+43 %) and Neptune (+100 %) today** (§3b), which is an authoring bug, not a false alarm |
| datum pressure, mean temperature | **±40 %** | Mars's own seasonal swing is ±37 %; anything tighter is chasing weather |
| scale height | **±10 %** | the sheets state it directly (Venus 15.9, Mars 11.0) and it is our least-constrained number — Uranus at −28 % is the one it catches |
| diurnal range, wind speed | **not a band — a target we do not yet have a place to put** | they exist in the sheets and nowhere in the game (#688) |
| `H == Rs T / g` | **never an assertion** | §3b: the giant sheets disagree with themselves by 4–19 %, so a rule binding those three is stricter than the data |

And the device, generalized from #685: **print what every profile asserts, and diff it
against the outside source, unconditionally.** `tests/test_scenarios.cpp:check_air()`
already prints `-> T … p … a` for every air body and pins it; `--atlas-dump` prints it in
the game. Neither compares to anything. #685 is the first case of comparing; the natural
full form is a **profile regression harness** — p, rho, T, a at standard heights per body,
printed against published reference values, so we run the expert's comparison ourselves
and publish the deltas instead of waiting to be told. `support/datum_table.py` in this
directory is that harness in embryo: it resolves both shipped systems and prints the
deltas against the sheets, in python, without booting the game.

---

## 9. Part temperature: the one thing worth starting now

Agreed with the stated instinct — start heat now, as a display in the part window, with
damage later. Two conditions, and one trap.

**The trap: do not build a display-only heat number.** If the readout is fake, #553
(reentry heat) has to be re-derived from scratch and the player will have spent an hour
learning a number that turns out not to be a law. Build the real equation, ship it with
no consequences, and #553 becomes "now this number can break things" instead of a rewrite.

**What it needs, and what already exists:**

- *air static temperature* — have it (`drag.h:airTemperature`).
- *aerodynamic heating* — the standard quantity is the **recovery/stagnation temperature**
  `T_aw = T · (1 + r·(γ−1)/2·M²)` with recovery factor r ≈ 0.9. We already compute
  per-surface `rho`, `v`, and Mach from ONE read per surface per pass (`flowMach`), so the
  inputs are there and cheap. The flux that goes with it is the next bullet's problem.
- *solar heating* — **does not exist.** No insolation value, no flux vs 1/r², no sun angle
  on a part. The geometry half exists and is better than it looks: `sunDirRot`
  (`surfmap.cpp:18`) gives the star direction in the body's **rotating** frame and
  `surfmap.h:15` already evaluates `dot(n, sun)` per terrain pixel — but only for shading.
  The missing piece is a W·m⁻². This is the new code.
- *radiative cooling* `εσT⁴` and convective exchange — new. Pick the local law
  deliberately rather than inheriting the aero one: a crude forced-convection estimate is
  `∝ ρ·v·(T_skin − T_air)`, but the standard stagnation-point heating rate is
  **Sutton–Graves**, `∝ √ρ_n · v³ / √r_n` — it reads the nose radius, which is the part
  geometry we already project for silhouette. The `q` the aero path computes is *not* that
  law and should not be reused as a heating proxy without saying so. Needs two authored
  numbers per part (emissivity, heat capacity or mass×cp). Note we already derive molar cp
  per gas in `atmo.h`; solids need their own number, ideally one per material class rather
  than per part.
- *surface area* — the silhouette machinery already projects every part and has mesh
  bounding boxes; a wetted-area estimate is close at hand.

**Why this is the keystone and not a side quest.** It is the first thing that makes the
air temperature profile *matter to the player* rather than being a readout; it is #553's
prerequisite; it forces the insolation quantity that §6 shows seasons also need; and
`power_scaler` suggests someone once already wanted a per-body stellar number. One
quantity, three buyers.

**The instrument lesson hidden in it:** a thermometer at speed does not read static air
temperature. On today's Kerbin (187.96 K, gamma 1.4015) at M 2 with r = 0.9 the factor is
`1 + 0.9 × 0.40146/2 × 4 = 1.7227`, so `T_aw = 323.9 K` — a skin probe reads 51 °C in air
that is −85 °C. Static and stagnation are two different instruments with two different
answers, and an expert will ask which one the widget is. Get that distinction right in the
naming now and the whole heat system reads as competent.

---

## 10. Instruments are the contract

The acceptance test is instrument-based, so the instrument set is not decoration — it is
the specification of what the model must contain. Each instrument is a claim:

| instrument | claims the model contains | today |
|---|---|---|
| barometer | static pressure vs altitude | yes (`airPressure`) |
| thermometer (static) | air temperature vs altitude | yes, on one body's real profile |
| skin/stagnation probe | recovery temperature, i.e. heat | **no** |
| hygrometer | water vapour, saturation | **no** |
| anemometer | air velocity relative to the ground | **no** — reads ship velocity only |
| radar vs pressure altimeter | that the two are different quantities | pressure altimeter is implied, never named |

The first three already have data; the last two need rungs 11–14. Read them as a
roadmap: **the instrument list is the C-axis backlog in the order a player would miss
it**, which is also why wind (§6) beats humidity: it makes an instrument honest for one
vector's worth of work.

Our developer-side mirror already exists and should be extended rather than reinvented:
`check_air` in the scenario tests (what each profile asserts, pinned), `--atlas-dump`
(the dossier the player reads, dumped headless — `src/cli.cpp:379`), and the readout rule
in `src/tick.cpp`. The rule for anything new here is the one already in use: **every
readout must be able to be non-zero, and every test must name the mutation that makes it
red.**

---

## 11. The order I would work in

Not a commitment — the reason-order, given the couplings above.

1. **#686 — Kerbin's datum (§4).** A decision, not a derivation: what is Kerbin's surface
   temperature and pressure? If Earth-like, give it a 7-level table (option C, measured
   above) rather than a bigger scale height (option B, which 84×'s the air at the home
   world's air top). Highest credibility per unit work in this document, and it is
   blocking the expert test on the world players actually fly. **Needs your answer about
   what Kerbin *is* — it is not derivable.** Shay, Laythe and Duna ride the same decision.
2. **#683 geopotential height.** Exact, cheap arithmetic, and it must precede the air-top
   decision because it makes the residual air at a top denser by **1.01–5.4×** depending on
   the body — 3.8× on Kerbin, 5.4× on Duna, 1.04× on Jupiter (§7).
3. **#682 where the air ends** — decided as a *pair*: the height and the falloff. Make
   Earth and Kerbin's choices in the same breath as their profile shapes.
4. **#685 + the profile harness (§8)** — and the first thing the harness will find is that
   **Saturn, Uranus and Neptune's densities are −47 %, +43 % and +100 % off their own fact
   sheets, and Uranus's scale height is −28 % off** (§3b). Fixing those four numbers is
   data authoring, needs no decision from anyone, and is the cheapest accuracy win
   available. Pluto wants authoring from its sheet; Titan has no sheet.
5. **#625 the remaining bodies' profiles** — with §4's lesson written into the ticket: a
   scale height is a surface temperature, so a height is never a drive-by. §3b adds the
   other half: the giants want tables because their data cannot be one column at all.
6. **#687 — a surface with a temperature (rung 10)** — the prerequisite for every climate
   wish, and the thing that lets Laythe's ocean and Laythe's 150 K air stop contradicting
   each other. It also owns the insolation quantity §9 needs, and the dead `power_scaler`.
7. **Part heat (display-only, real equations, §9)** — can start in parallel with 6; it
   needs the same insolation quantity. #553 is its eventual consequence.
8. **#688 — wind (rung 12)** — one seam, `Vehicle::airRelativeVel`, which every aero read
   already goes through (verified: `vehicle.cpp:2642` defines it, `vehicle.cpp:2731` makes
   the one call per pass, and six `localFlow` calls consume it — 3072, 3121, 3192, 3263,
   3395, 3436), and published targets already on disk. First C item, and the only one that
   is a same-day change.
9. **Diurnal and latitudinal structure, then seasons** (the geometry is already in
   `system.cpp:776-787`/`surfmap.cpp:sunDirRot`; #687 §"What already exists" has the
   detail).
10. **Water, humidity, rain** — after all of the above.
    **Climate** — disclaimed, not built.

---

## 12. What this document does not claim

So it can be trusted as a snapshot, and so a future reader knows what is *not* verified:

- **Nothing here compares our atmosphere aloft to a reference profile.** The fact sheets
  stop at the datum (they publish a scale height, not a table). Earth's 7-level table is
  the 1976 standard and is therefore right by construction against that standard, which
  is a *model*, not a measurement of any particular day. Nobody has checked our Venus or
  Mars against an entry profile.
- **No multiplier from our profile to a real one may be quoted without naming its
  source.** Recorded on #625 after an earlier draft of mine reached ~20× at 50 km on
  Venus from a figure an agent admitted it was recalling.
- The fictional system (Kerbin etc.) has **no external reference at all**. Its fidelity is
  form and plausibility only. Any claim of "accuracy" on Kerbin is a claim about design
  intent.
- All numbers here are 2026_11_07 and move the moment a scale height, a composition, or
  #683 moves. Re-run `support/datum_table.py` rather than trusting this file — that is
  the whole reason the script is in this directory.
