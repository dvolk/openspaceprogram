# Wind everywhere there is air — 2026_11_12

Snapshot of how the wind model stands, and what it takes to put wind on **every
atmosphere in both catalogues** — the solar system and the Kerbal side. Written
before the work, because the last stretch went faster when the plan existed
before the code than when it was invented per-commit.

Provenance of every number below is in the appendix; the two load-bearing ones
are measured, not estimated: the Kerbal airs' resolved facts come from the game's
own `--atlas-dump`, and the re-baseline surface comes from a full probe run
against a temporary winded `ksp_system.json`.

---

## 1. Where we are

| catalogue | airs | author wind | notes |
|---|---|---|---|
| `solar_system{,_full,_measured,_named}.json` | 9 | **8** | #772 landed Jupiter, Saturn, Uranus, Neptune, Titan. Only Pluto is still air-without-wind, and that exemption is a test's fault (§5.1) |
| `ksp_system.json` | 6 | **0** | Kerbin, Eve, Duna, Jool, Laythe, Shay |
| `wind_playground.json` | 7 | 7 | invented bodies; the only place `wind_gen` ships today |

Mechanically the goal is already reachable. Everything the Kerbal half needs
exists and is exercised on the solar half:

- `WindLevel` carries alt, east, north, **up**, a latitude band and a longitude
  band (`src/atmo.h`); bands are half-open upward so a published
  `<30 / >30` pair tiles instead of overlapping (#767).
- The loader refuses an incoherent table rather than picking a rule: uncovered
  sphere, two rows answering one place at one altitude, a per-band column that
  does not rise, out-of-range components. Overlaps go to the narrowest band.
- `windAt` resolves a hull's air velocity; `Vehicle::airRelativeVel` spends it,
  so a wind moves ground speed and leaves airspeed alone (#688).
- `wind_gen: rhines` turns a two-row column into belts from the body's own spin
  (#775), and the dossier prints the derived Ro rather than quoting a generated
  belt as a measurement.
- Readouts exist: dossier row per authored level, `Tas:`/`Wind:` on the Surface
  window, `tas=`/`wind_east=`/`wind_north=`/`wind_up=` in `[surfinfo]`, and a
  wind map with one arrow row per band where the table has bands (#769, #780).

So the Kerbal half is **data plus a re-baseline**, not new machinery. That is the
good news. The cost is the one we kept tripping over: Kerbin is the default
system, so a wind there is not a catalogue edit, it is a physics edit to most of
the test fleet (§4).

---

## 2. The six Kerbal airs, priced

Resolved by the game itself (`tmp/atlas_ksp.log`), not by hand:

| body | g m/s² | day | ρ₀ kg/m³ | sound m/s | air top |
|---|---|---|---|---|---|
| Kerbin | 9.81 | 6.00 h | 1.225 | 340.5 | 85 km |
| Shay | 8.99 | 6.00 h | 1.225 | 340.5 | 95 km |
| Laythe | 7.85 | 14.72 h | 1.225 | 340.5 | 105 km |
| Eve | 16.67 | 22.68 h | 3.559 | 269.7 | 31 km |
| Duna | 2.94 | 18.27 h | 0.015 | 231.5 | 50 km |
| Jool | 7.85 | 10.00 h | 0.188 | 870.6 | 600 km |

Candidate columns, with what each row costs at its own altitude
(`tmp/ksp_wind_table.py`), against the Kerbin cruise reference of 1.6–3.8 kPa:

| body | column | q at the deck | q at the high row | Mach |
|---|---|---|---|---|
| Kerbin | 5 m/s deck, 40 m/s at 11 km | 15 Pa | 180 Pa | 0.12 |
| Shay | 5 / 40 at 11 km | 15 Pa | 208 Pa | 0.12 |
| Laythe | 5 / 40 at 11 km | 15 Pa | 254 Pa | 0.12 |
| Eve | 10 m/s deck, 60 m/s at 5 km | 178 Pa | 1.11 kPa | 0.22 |
| Duna | 10 m/s deck, 30 m/s at 10 km | 0.8 Pa | 3.0 Pa | 0.13 |
| Jool | 120 m/s deck, 300 m/s at 30 km | 1.35 kPa | 5.10 kPa | 0.34 |

Two things fall out of that table.

**None of it touches the transonic trouble.** Every candidate row is Mach ≤ 0.34
and at most a fifth of a Kerbin cruise. The Kerbal half carries **no #716
exposure at all** — unlike Neptune, where the sheet's own 580 m/s is 73 kPa at
Mach 1.01 and I capped it. Kerbal winds can be authored at full confidence.

**Eve is the one worth being careful with.** 3.559 kg/m³ is the densest air in
either catalogue after Venus, so Eve's wind is the only Kerbal row whose dynamic
pressure is comparable to a cruise: 60 m/s there is 1.1 kPa. A gust of 100 m/s
would be 3.2 kPa. Worth pinning a ceiling in the comment rather than discovering
it from a broken golden.

Where each number comes from, since these are invented worlds with no fact page:
Kerbin/Shay/Laythe are dry air at 1 atm, so Earth's authored column (5 m/s deck,
40 m/s jet at 11 km) transfers directly. Eve's is the Venus shape — slow deck,
fast aloft — scaled to an air a third as dense and a quarter the profile height.
Duna's is Mars's lander-site figure. Jool's is Jupiter's and Saturn's envelope:
zonal jets a fraction of the sound speed. That rationale is the thing that has
to live **in the file comment**, because there is no sheet for `check_fact_wind`
to weigh it against; the waiver machinery (`FACT_WIND_KNOWN`) is for cited
figures and must not be bent to cover invented ones.

---

## 3. What the generator asks for on those bodies

The Kerbal spins are fast and the radii are small, so `Ro = U/(2ΩR)` is much
larger than the solar giants' and the belt count comes out lower:

| body | U m/s | Ro | Rhines scale | belts/hemisphere | bands × 2 anchors |
|---|---|---|---|---|---|
| Kerbin | 40 | 0.115 | 20.8° | 4 | 9 (18 rows) |
| Shay | 40 | 0.125 | 21.8° | 4 | 9 |
| Laythe | 40 | 0.338 | 35.8° | 2 | 5 |
| Eve | 60 | 0.557 | 46.0° | 1 | 3 |
| Duna | 30 | 0.491 | 43.1° | 2 | 5 |
| Jool | 300 | 0.143 | 23.3° | 3 | 7 |

Read the Jool row twice, because it is the counter-intuitive one: **a gentler
column paints more belts.** `L = sqrt(U/β)`, so at 120 m/s Jool asks 6 belts per
hemisphere (13 bands) rather than 3. If we want Jool to *look* banded — and it is
the system's poster world — the lever is the authored speed, not the generator.

My recommendation: **author Kerbin/Shay/Laythe/Eve/Duna by hand and put
`wind_gen` only on Jool.** On the four terrestrial Kerbal airs the Ro is already
0.1–0.6, where the Rhines argument is weakest (it is a turbulence-cascade scaling,
and it visibly overpredicts on Mars, which asks 10 belts where reality has one
jet). A generated field there would launder a made-up number into something that
prints with a Rossby number beside it, which is exactly the "reading the file
never gave" that #727 and the band-edge rule exist to prevent.

---

## 4. The measured cost, before touching anything

The probe (`tmp/probe_ksp_wind.sh`, ~90 s): copy `ksp_system.json`, add the six
candidate columns, run every KSP-side case that quotes an airspeed or a drag
figure, restore the file.

- 392 cases in the battery; **332 run on the Kerbal default system** (they name
  no `--system`), 27 name `ksp_system.json` explicitly.
- 193 of those spawn something on Kerbin.
- **43** name an airspeed, `tas=`, a kPa figure or drag.
- With wind on all six: **32 pass unchanged, 11 fail.**

The 11 sort into four classes, and only one of them is a problem:

1. **A parked hull now has a flow worth reading** —
   `parked-hull-bills-nothing`, `surface-aoa-readout`,
   `skiff-per-surface-alpha`. Locus: `constants.h:kAoAFlowFloor = 0.1 m/s`
   dashes AoA/`Induced=` below it, because a parked hull carries ~1e-4 m/s of
   integrator jitter against the co-rotating air. That floor was written to
   suppress *jitter*; a 5 m/s wind is not jitter, so `lastDragAlphaValid`
   correctly turns the fields on and the `Induced=-` pins break. **This is #770
   arriving whether we ask for it or not** — see §6.
2. **Beds that author an airspeed now aim along moving air** —
   `trim-bed-speeds-follow-the-air`, `flying-slow-starts-in-air`,
   `surface-prograde`, `tvc-slew-prograde`. Ground speed shifts by the wind
   vector while `|v_air|` holds. That is the model working; the pins are
   comparing the two numbers within a tolerance tuned for still air.
3. **Trim and control-surface angles pick up a few degrees of beta** —
   `skiff-trim-and-weathervane`, `elevator-trim`, `roll-ailerons`. Also the
   model working: a weathervaning hull in wind is not the same hull at the same
   heading in still air.
4. **One knock-on:** `thermometer-instrument` pins the pod-recorded air
   temperature against the dossier value; the trajectory moved, so the sample
   point moved. Legitimate re-pin.

So: **11 cases, of which roughly 7 need re-pinning and 3 are the #770 decision
dressed as test failures.** That is an order of magnitude cheaper than the
"226 cases mention Kerbin" headline suggests, and it is now a known list rather
than a surprise discovered at 2am by a red suite.

---

## 5. What gets in the way, and what I am deleting

### 5.1 Pluto's exemption — a test dictating catalogue data. Delete it.

`kWindsAuthored`'s comment and `tests/test_scenarios.cpp:1121` say Pluto "keeps
the 'air, but no wind authored' hole that render/769 and render/717 need an
example of". That sentence is the reason a body in a physics catalogue has no
wind, and it is the wrong kind of reason: the data exists to serve a test.

Delete the exemption. Author Pluto a row (1 m/s at its own datum — its 1.3 Pa of
N₂ resolves to ~1e-4 kg/m³, so q is ~5e-5 Pa: the air is real, it just cannot
push) and re-home the two refusals:

- "no air at all" keeps its real examples: Io and Triton, which ship a render rim
  and no profile. That distinction is exactly what `windLayerFor` was fixed for
  (`6e83f440`), so it is not new coverage.
- "air, but the file states no wind for it" stops needing a *shipped* example. It
  is already covered as a law by `test_surfmap`'s synthetic air-with-no-rows
  surface, and it stays reachable for any hand-authored file. If we want an
  end-to-end leg for it, it goes in a scratch/playground system file, not in the
  catalogue.

### 5.2 Four goldens that are really one invariant. Demote them.

`flight/772-the-giants-and-titan-move-their-air` pins `|v|` and `|v_air|` on five
bodies with regex ranges. The law is one sentence — *the airspeed is the bed's
own value and the ground speed carries the wind vector* — and four of those five
pins are just numbers that move whenever a bed or the aero law moves. Keep
Jupiter (its track is due east, so the gap is exactly the authored 150.0, which
is the tightest possible statement of the law) and one "all five moved"
invariant; drop the other four ranges.

### 5.3 The `kWindsAuthored` list is scoped to the solar files. Extend it.

`tests/test_scenarios.cpp` walks only the four `solar_system*.json` variants, so
a Kerbal wind row would land with no pin against it. Extend the same loop to
`ksp_system.json`. This is the one guard I *add* rather than delete, because the
solar half shows what happens without one (Titan's rows passed `check_fact_wind`
for a whole pass by never reaching it), and because it is 3 lines.

### 5.4 `check_fact_wind` has nothing to say about Kerbal air, and that is fine.

The fact guard is a solar-system instrument: it reads scraped pages from
`utils/rss/html`. There is no Kerbal page and there never will be. Do not teach
it a fake sheet. What the Kerbal files owe instead is the rationale in the
comment beside each column (analogue, speed, q, Mach), which is what I did on the
solar side and which is cheap.

### 5.5 Comments that lie about the roster. Already deleted, keep deleting.

"every body but Venus, Earth and Mars", "the gas giants are waiting on #767" —
these lived in `src/surfmap.h`, `src/terragen.h`, `src/vehicle.cpp` and
`tests/test_surfmap.cpp` and pointed at a roster that had already changed. They
now point at `kWindsAuthored`, the one list that cannot drift. The Kerbal work
has the same failure mode: any comment naming which bodies have wind is a
comment that will be wrong by Friday.

---

## 6. The one decision the Kerbal half cannot route around: #770

`#718` measured a grounded hull in wind and left it at "creeping, below the
threshold", parking the design question as **#770**. On Earth the datum wind is
5.45 m/s and a parked hull creeps to 0.0025 m/s; the ground holds it; the
threshold on that hull is between 20 and 30 m/s.

Making wind universal makes #770 load-bearing rather than interesting. With a
Kerbin deck wind, three shipped cases stop dashing `AoA`/`Induced=` on a parked
hull, because the flow is now genuinely above the 0.1 m/s jitter floor. The
choices:

- **(recommended) Bill it and say so.** The aero laws already see every frame
  however slow (`kAoAFlowFloor` is display-only), so this is not a new force —
  it is the readout admitting the flow exists. Re-claim the three cases as "a
  parked hull in wind has a flow, and it is small": pin the N, not the dash.
  Then #770 becomes a measured statement per body rather than a pending choice.
- Grounded ⇒ no aero, as a rule. A deletion of real physics; it would make a
  hull on Eve's deck deaf to 1.1 kPa of flow. I do not think this survives
  contact with a landing case.
- Keep the dash for grounded hulls regardless of wind (display only). Cheapest,
  but it hides the one quantity the player would use to judge "will the wind
  slide me off the pad", which is the whole reason #718 was measured.

If Kerbin stays winded and #770 stays undecided, we get option one by default
and by accident. Decide it first.

---

## 7. The physics hole that is not a test: #716

Nothing caps an aero impulse. It is why Neptune's published 580 m/s is capped in
the shipped file (73 kPa at Mach 1.01, in the middle of the lift law's
0.75–1.25 transonic blend), and the documented blow-up landed at 810 kPa. The
Kerbal half does not need it (§2), so it does not gate this work. It does gate:

- raising any giant's column toward its published top,
- any future Jool storm or Eve squall that is *interesting* rather than typical,
- and my ability to stop capping numbers with editorial judgement.

I would take it as the next physics ticket after the Kerbal data lands, and the
cap is a small piece of work: clamp the aero impulse per substep to some multiple
of the hull's momentum, log when it clamps, and make the log observable so a
case can pin the clamp rather than the explosion.

---

## 8. Order of work, one proof per step

Each step names the single thing that proves it. Nothing here gets an
adversarial subagent: one pass per feature, at the end, and never over its own
fixes.

1. **Decide #770** (you). Proof: a sentence in the ticket.
2. **Kerbal data**: six columns, rationale in comments, `wind_gen` on Jool only.
   Proof: `--atlas-dump` prints six `air wind` blocks with the authored numbers;
   `kWindsAuthored` extended and green.
3. **Re-baseline the 11** by measurement (`--show-cmd`, read the diff, update
   deliberately). Proof: the probe list runs green and the diff says which
   numbers moved, in the commit body.
4. **One new e2e case** for the Kerbal half, Kerbin only, in the Jupiter shape:
   airspeed at the windless control's value, ground speed carrying the deck row.
   That is the whole fleet-facing claim.
5. **Pluto + refusal re-homing** (§5.1) and the `flight/772` demotion (§5.2) in
   one cleanup commit. Proof: `make test` and the two touched render cases.
6. **Then #716**, as its own feature, with its own review pass.

Estimated e2e cost across all of it: the 43-case probe once, the 11 re-runs,
the new case, and the two render cases. Not the fleet.

---

## 9. Process, so we stop losing days to it

- **Plan before a multi-ticket stretch**, ~40 lines, including the decisions that
  are yours. This document is that plan for the wind work.
- **One adversarial pass per feature, at the end, never over its own fixes.**
  The passes this week found real bugs (`lon == 360` → SIGABRT, a render rim
  printed as authored air, a waiver keyed on the first row written) — the waste
  was recursing, not reviewing. A 30–60 minute subagent over a fix to a fix buys
  nothing I cannot buy by re-running the two gates the fix touches.
- **At most one new e2e case per feature**, and none for a change I know passes.
- **Measure the blast radius before editing shipped data.** The 43/11 split took
  90 seconds and turned "wind on Kerbin might wreck the suite" into a named list
  of eleven. That is the pattern to reuse: probe on a scratch copy, restore,
  report the number.
- **Prefer invariants to goldens.** A golden is a number with a hostage.
- **Comments that enumerate state get deleted or replaced by a pointer to the
  list that owns it.**

---

## Appendix: what produced these numbers

- `tmp/atlas_ksp.log` — `SDL_VIDEODRIVER=offscreen ./osp --system
  res/systems/ksp_system.json --atlas-dump 4000 --timeout 6`.
- `tmp/ksp_wind_table.py` — q and Mach per candidate row, using `drag.h`'s
  geopotential depth `R·h/(R+h)` rather than `alt/H`, and the profile's own
  layer temperatures. Aloft densities are approximate (piecewise lapse); deck
  densities are the game's resolved values.
- `tmp/probe_ksp_wind.sh`, `tmp/probe_ksp_wind.log`, `tmp/ksp_wind_selectors.txt`
  — the edit-run-restore probe, and the 11 named failures.
- Solar-side figures: `python3 utils/make_solar_system.py --check` prints the
  authored-vs-sheet table and the Ro/Rhines scaling for every planetary air.

Scratch, not shipped, not deleted: `tmp/ksp_baseline.json` (the pristine
`ksp_system.json`), `tmp/probe_ksp_wind_outer.log`.
