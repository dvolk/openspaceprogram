# Kerbin's air, and how a hull's drag number is actually built — 2026_11_17

Snapshot of why climbing out of Kerbin reads as soup, written **before** implementing
#945, because that fix changes the one number every flight case is pinned against.
Two separate errors are in play and they are worth separating: one is bookkeeping in
the aero pass (#945), one is the autopilot flying sideways through the air (#927).
Every figure below is measured from the game's own logs except where a line says
**derived** (arithmetic shown, inputs measured) or **expectation** (no measurement yet
— do not quote it as one).

An earlier pass of this report estimated the axial numbers. They are measured now: the
`flying-low` bed fires the audit dump at `alpha 0.0 beta 0.0`, and §2/§3 are the
measured version.

---

## 1. The wall, measured

`mars_explorer_mk1` on Kerbin, `--ascent 100e3 --time-accel 10 --drag-log`, stock
catalog. Mass 106,521 kg (`[ship] compound mass`), weight **1044 kN**.

| t | inertial v | air-relative v | AoA | q (air) | `A=` union | `Cd=` | CdA | `|F|` | F/W |
|---|---|---|---|---|---|---|---|---|---|
| 10 | 24.1 | 21.8 | 6.0 | 281 | 91.17 | 0.880 | 80.2 | 27 kN | 0.03 |
| 20 | 48.5 | 44.5 | 15.7 | 1136 | 111.74 | 0.887 | 99.1 | 135 kN | 0.13 |
| 30 | 69.3 | 63.3 | 21.4 | 2201 | 123.76 | 0.890 | 110.1 | 290 kN | 0.28 |
| 40 | 86.2 | 77.9 | 26.5 | 3181 | 133.14 | 0.893 | 118.9 | 452 kN | 0.43 |
| 50 | 98.9 | 88.4 | 30.7 | 3924 | 140.27 | 0.896 | 125.7 | **593 kN** | **0.57** |

The force identity is exact — `|F| = drag_cd × Cd × A × q_air`, `drag_cd` = 1.2 by
default (`src/vehicle.h:635`), `dragForce` being `½·rho·cd·area·|v_rel|²` with **no Mach
term** (`src/drag.h:327-337`) applied verbatim at `src/vehicle.cpp:3841`. The memo states
it more cleanly than the code (`src/drag.h:1114-1116`): *"Speed and density multiply the
force, never its shape."* The right-hand product reproduces the measured force to 0.4% at
every row, so the logs are internally consistent; part of that residual is the dynamic
pressure being evaluated at the swept cp rather than the cm (§4), not log timing. Note `q`
must be built from the **air-relative** speed (`Mach × airA`), not the row's `|v|`: since
#688 the air moves, and at this altitude the two differ by ~11%. Mach is 0.064 at t=10 and
**0.263** at t=50 — subsonic throughout, which is why the missing Mach term is not what
hurts here. (The compressible corrections that do exist — `waveCd` `src/drag.h:519`,
Prandtl-Glauert on `liftSlopeMach` `src/drag.h:461` — sit in the *surface/lift* pass,
`src/vehicle.cpp:3917`, not in the hull coefficient.)

So by t=50 the air is holding back **57% of the ship's own weight** at Mach 0.26.

The same ship with the #936 anchor retune (tanks and engines less blunt end-on) gets
through that wall only by meeting it sooner and faster:

| t | v_air | AoA | q | Cd | CdA | `|F|` | F/W |
|---|---|---|---|---|---|---|---|
| 30 | 72.1 | 20.3 | 2837 | 0.553 | 67.4 | 229 kN | 0.22 |
| 40 | 94.0 | 24.7 | 4563 | 0.568 | 73.8 | 404 kN | 0.39 |
| 50 | **111.0** | 28.6 | 6049 | 0.585 | 80.0 | **580 kN** | **0.56** |

CdA falls 80 → 48 m² at t=10 (a 40% cut); at t=50 the ship is moving 26% faster through
the air (111.0 m/s air-relative against 88.4; at t=10 the two runs are within 1% of each
other) — and it **lands on the same 0.56 F/W wall at the same instant**. That is the
signature of treating the wrong term: a shallower drag floor raises terminal speed, it
does not remove the floor.

---

## 2. Two errors, measured

Reading the same ship axially (§3's dump — the header prints `alpha 0.0 beta 0.0`, and
its `%.1f` format is rounded, so "axial" means axial *to that rounding*). Corroborated by
the geometry: that dump's `unionA 78.99` sits 13% **below** the 6° row's `91.17`, so the
flow angle there was nowhere near 6°.

| quantity, axial | value | how |
|---|---|---|
| silhouette the force is billed on, `unionA` | **78.99 m²** | measured |
| area that votes on the coefficient, `SUM projA` | **366.70 m²** | measured (sum of dump rows) |
| over-vote | **4.64×** | derived |
| coefficient that produces | **0.875** | derived (the dump prints `cd 0.87`) |
| CdA today | **69.1 m²** | derived |
| terminal speed at that CdA, `rho 1.1813` (375 m) | **160 m/s** | derived |
| coefficient if only the upwind-facing surfaces vote | **0.438** | derived from measured geometry |
| CdA then | **34.6 m²** | derived |
| terminal speed then | **226 m/s** | derived |

The visible set behind 0.438 (`tmp/visible936.py`) walks the dump's own per-part discs:
per `(right, up)` column, credit each part the part of its disc that the parts **ahead of
it** have not already covered.

**The sign convention, because the sort depends on it.** `dragForce` returns
`-normalize(v_rel) · (½·rho·cd·area·|v_rel|²)` (`src/drag.h:337`) and `AeroFrame`'s S axis
is the nose (`src/drag.h:346-350`). The dump prints `noseDot +1.00` on **all 30 parts** in
this view — every part's nose points into the relative wind, so the ship travels
nose-first and the part with the **largest** `fwd` in a column is the upwind one.

| what the air actually meets, axially | area | cd |
|---|---|---|
| 4 × `nose_cap_r2.25h1.125` (the booster fronts) | 63.20 | 0.35 |
| `capsule` (top of the core stack) | 3.12 | 0.35 |
| `adapter_r1.5to1` shoulder | 3.90 | 0.90 |
| `adapter_r2.25to1.5` shoulder | 8.78 | 0.90 |
| **sum** | **79.00** (union 78.99) | → mean **0.438** |

Everything else — the six big tank discs, the four booster decouplers, the engines, the
r1.5/r1 upper stack — sits behind those discs in this view and votes anyway today. The
core column's three rows sum to 15.80, which is one r2.25 disc: the walk credited the
capsule its small disc, then each wider part the annulus the parts ahead of it left.

**What that 79.00-vs-78.99 match does and does not prove.** It is bookkeeping, not
physics. The four booster columns sit at radius `sqrt(2.89² + 3.44²) = 4.49` from the ship
axis; two r2.25 radii are `2.25 + 2.25 = 4.50`, so the booster discs are essentially
**tangent** to the core column (a lens overlap of ~0.002 m²) and disjoint from each other
(5.78 apart). So `union == sum of the five column leaders` was true *before* the annulus
walk ran, and the walk reproducing it shows only that no area was invented or lost. It
pins **no** claim about which part covers what: the walk carries `covered` as a single
scalar, which assumes the discs in a column are concentric and nested — true here because
the stack is coaxial, and unverifiable from this dump at an angle where they are not. The
test that would actually falsify it is §4's depth-sorted oracle at an off-axial angle.

Splitting the t=50 bill of 125.7 m² between the two errors:

| what | CdA | status |
|---|---|---|
| flown today (AoA 31°, mean cd 0.896, A 140.27) | 125.7 | measured |
| if it held ~5° AoA | ~69–80 | derived: axial 78.99 × 0.875 = 69.1; the measured 6° row is 91.17 × 0.880 = 80.2. No measured ~5° row exists |
| nose-first with only visible surfaces voting | 34.6 | derived from measured geometry, unimplemented rule — #945 alone |
| both | ~35 | derived |

A factor ~3.6 between how the rocket flies today and how a nose-first stack should be
billed — note that divides *different rows* (today's 31° silhouette against an axial one),
so it is the combined effect of both fixes, not a clean ratio.

The climb's own shape is the more useful fact: between t=10 and t=50 the coefficient moves
**+2%** (0.880 → 0.896) while the silhouette moves **+78%** (91.17 → 140.27). The wall is
attitude, not coefficient. Near axial, the #945 bookkeeping is worth about half the bill;
at AoA 30° the flanks of the tanks are genuinely exposed, so #945's win shrinks and the
attitude term dominates — **expectation**, because there is no post-#945 number at 30°
anywhere yet, and getting one is part of the work.

---

## 3. How `Cd` is built, and the dump that shows the gap

`Vehicle`'s aero pass, per substep, over every hull part (src/vehicle.cpp ~3700-3800):

```
projectPartHull(...) -> poly          // the part's silhouette in the shared flow plane
a      = polygonArea2(poly)           // that part's OWN projected area
cp     += a * <that part's silhouette centroid>
cpArea += a
if(p->def && cd > 0.0) cdNum += a * partCd(fwd, side, back, cos(part nose, flow))
...
lastDragArea = A_ship                 // the UNION of those polygons (memoised per direction)
lastDragCd   = cdNum / cpArea         // a mean weighted by each part's OWN area
force          = dragForce(da, drag_cd * lastDragCd, A_ship, alt, cp-local flow)
```

The two halves are measured against different areas. The comment in the code says so
outright (~3784-3792): *"The coefficient and the centre of pressure stay weighted by the
SUM of the parts' areas, while the force uses their UNION… A ship whose parts overlap in
the view gets no extra drag, but its cd still leans toward whichever part presents the
most material."*

That is defensible as a *stance* and wrong as *arithmetic*, because "the most material" is
largely material the silhouette has already hidden. The design-audit dump
(`OSP_SHIP_DUMP=1`) prints the gap; fired at `alpha -90` (broadside) on the pad and at
**`alpha 0.0 beta 0.0`** when the same ship spawns in the `flying-low` bed:

```
[ship] compound mass 106521.0  vrel 2218.1 m/s  alpha 0.0  beta 0.0  alt 8500
[ship] SUM projA 366.68  unionA 78.99  cp fwd 0.82 right 0.00 up -0.00  cd 0.87
```

Per part at that axial view — `projA` is each part's own disc, `cd` is the `partCd` value
it argues facing the flow, `fwd` is its offset along the ship axis (so a part at `fwd
+2.08` is ahead of one at `fwd -1.54` in the same column):

| part | fwd | projA | cd |
|---|---|---|---|
| `capsule` | +21.37 | 3.12 | 0.35 |
| `reaction_wheel` | +20.24 | 3.12 | 1.00 |
| `tank_r1h3` | +18.62 | 3.12 | 1.00 |
| `nuclear_engine` | +15.62 | 3.12 | 0.90 |
| `decoupler_r1` | +13.99 | 3.12 | 1.10 |
| `adapter_r1.5to1` | +13.49 | 7.02 | 0.90 |
| `reaction_wheel_r1.5h0.375` | +12.93 | 7.02 | 1.00 |
| `tank_r1.5h5` | +10.24 | 7.02 | 1.00 |
| `orbital_engine_r1.5h1.5` | +6.99 | 7.02 | 0.90 |
| `decoupler_r1.5` | +6.05 | 7.02 | 1.10 |
| `adapter_r2.25to1.5` | +5.30 | 15.80 | 0.90 |
| `tank_r2.25h5` (core) | +2.24 | 15.80 | 1.00 |
| 4 × `nose_cap_r2.25h1.125` | +2.08 | 15.80 | 0.35 |
| 4 × `decoupler_r2.25` | +1.24 | 15.80 | 1.10 |
| 5 × `tank_r2.25h5` | −1.54 | 15.80 | 1.00 |
| 5 × `engine_r2.25h4.5` | −6.29 | 15.80 | 0.90 |

Three footnotes the mechanism has to respect:

- `projectHull` returns the **convex hull of the projected verts** (`src/drag.h:841-846`).
  That is a bigger footnote than an earlier draft carried: `adapter_r2.25to1.5` reports
  `projA 15.80` — the **filled** r2.25 disc, not the `pi(2.25² − 1.5²) ≈ 8.8 m²` annulus
  the part actually presents. Its own area is inflated by ~7 m², so the convex hull, not
  the cd, accounts for the 15.80-vs-8.8 discrepancy. The shoulder annuli in §2 were
  recoverable only because the stack is coaxial.
- `cdNum` is guarded by `p->def != nullptr && cd > 0.0`, while `cpArea` is not — a part
  with no definition, or a cd of 0, votes in the denominator only.
- The axial dump fires at whatever speed the bed spawns (Mach ~6.6 here). Only geometry is
  being read, and the hull law has no Mach term, so the anchors are unaffected.

---

## 4. What fixing it looks like

Weight the coefficient (and the centre of pressure) by each part's **unoccluded** projected
area in the flow direction, instead of its own projected area.

**Attribution has to be per sub-interval, not per merged interval.** The existing union
(`unionProjectedArea`, `src/drag.h:1004`) cuts the plane at every vertex x and every
polygon-pair edge crossing, and inside one slab sums each polygon's cut into `bands` as
bare `std::pair<double,double>` intervals — it **discards which polygon each interval came
from**, then merges overlapping ones. (`Span { size_t poly; double lo, hi; }` keeps that
identity one level up, so the plumbing is close; `bands` is where it is lost.) The real
error mode: crediting a merged interval to one polygon overpays it. A covering `[0,2]` and
B covering `[1,3]` merge to `[0,3]`, and giving that wholly to B bills B 1 m² it does not
cover while zeroing A. So `bands` must carry the polygon id, each merged interval must be
split at its endpoint set into atomic sub-intervals, and each sub-interval credited to the
upwind-most polygon that **actually covers it**.

The slab walk tests a polygon's activity on **x alone** (`if(xm < sp.lo || xm > sp.hi)`),
and for **convex** rings that is enough: a convex polygon whose x-extent straddles the slab
has a non-empty cut, and `bands` only ever receives intervals under `if(hit && hi > lo)`
(`src/drag.h:1093`). An earlier draft of this report claimed a polygon could be "active but
cover no height" — it cannot, on convex rings. But notice what that rests on: the convexity
contract spelled out at `src/drag.h:839-845` ("only correct for convex rings").
Sub-interval attribution reintroduces per-polygon coverage assumptions at exactly the place
where convexity was silently paying the bill.

**Do not oversell the invariant.** With whole-merged-interval credit, "Σ visible == union"
passes automatically while the physics is wrong, and `A_ship` is untouched by this change
anyway — the invariant guards a quantity the change never moves. The quantities at risk are
the cd mean and the cp, and the cp matters more than it looks: the hull force is evaluated
on the cp-local flow including the rotational sweep
(`localFlow(vrel, w, cp - com, rho)`, `src/vehicle.cpp:3838`), so moving the cp moves `|F|`
even at fixed `A_ship` — that is the swept-`q` term behind §1's 0.4% residual. The
invariant is worth asserting only *after* sub-interval attribution, where it says something
real.

**Cache scalars per node — and choose their basis deliberately.** `cd` is itself a
node-level quantity: `unionAtS` already knows the node's flow direction, so
`partCd(fwd, side, back, dot(localRot[2], nodeFlow))` (`src/drag.h:1558`) is computable at
fill time. So `SilhouetteMemo` (`src/drag.h:1120`) stores more than one number per node —
`area`, `cdNum = Σ visible_i · cd_i`, and the cp numerator — and `lookup()` blends them
bilinearly as it blends `area` today; the coefficient is their ratio. No per-part arrays, no
node-keyed sparse storage. Two honest qualifications:

- This is not free. Today's table is `area` (float) plus `stamp` (uint32) over 961 nodes =
  7688 B. A `cdNum` adds 3844 B; a cp numerator is **three** numbers per node, so the table
  roughly triples. At 31×31 nodes that is still small, but it is growth, not zero growth.
  (Fills are also not rare: the code's own note records 88 nodes filled by one 14 s tumble.)
- The cp numerator cannot live in each node's own flow-plane basis: `flowPlane`'s reference
  axis swaps at `|d.x| < 0.9` (`src/drag.h:831-832`), so neighbouring nodes' cp coordinates
  are not comparable and blending them is meaningless. Store it in S (converting each
  node's plane centroids before storing) and project the blended result into whatever plane
  is current.

**The fingerprint does not cover the anchors.** `aeroGeomFingerprint`
(`src/vehicle.cpp:1723-1744`) mixes `parts.size()`, `p->uid`, the body and shape pointers,
`localPos` and `localRot` — **not `p->def`**, which carries the drag anchors and the area
fields that gate the `isAeroSurface` exclusion. A def changed in place leaves matching
fingerprints and stale nodes. #945 makes cached nodes depend on those anchors, so either
hash them into the fingerprint or state that defs are immutable once a hull exists.

**Caveats to keep.** `area` is `float` (`std::vector<float>`), so `Σ visible == area` will
not hold bit-exactly after blending four of them — assert with a tolerance. And the
coefficient is a *new kind of quantity* for this chart: today the memo blends only areas,
linearly, with no ratio in it at all. Adding `cdNum` makes the printed cd a
ratio-of-blends inside a cell, and the thing it interpolates is jumpier than area. The memo
already warns about the area jump — *"the union **JUMPS** where a part slides in or out of
another part's shadow, and no smooth interpolation of neighbouring nodes follows a jump"*,
worst 8.3% (`src/drag.h:1118-1124`). On mx, moving 15.8 m² of coverage from a nose cap
(cd 0.35) to a `decoupler` (cd 1.10) shifts the mean by about `15.8 × 0.75 / 79 ≈ 0.15`,
and adding it as new area instead gives ≈ +0.11 — call it **25-35% of the coefficient**,
against 8% for area. A per-part single "upwind-most" depth scalar — the cheapest way to
resolve two overlapping projections — flips whole regions when depth ranges interleave (mx's
four r2.25 booster columns at `right ±2.89 / ±3.44` overlap along the flow as soon as AoA ≠
0). Options, in order of preference: credit by the depth **at the covered region**; or keep
the coefficient on the exact per-substep path (§3) and memoise only area; or accept the
flip and label it. Do not pretend the blend smooths it.

**Centre of pressure.** It is weighted by the same `a`. The air pushes where it can see, so
weighting cp by visible area is the consistent choice — and it is the part of this change
most likely to redden golden trim and weathervane cases, because cp sets the weathervane
arm. Flagging rather than deciding.

**An exact oracle for the tests, for free.** Depth-sort the polygons and call the existing
`unionProjectedArea` on the first *k* of them: `visible_k = U_k − U_{k−1}`. The existing
union costs roughly a million floating-plane ops on mx's 30 rings, so full-prefix is cubic
and belongs in `tests/test_drag.cpp` as the reference that pins semantics, not in the hot
path. Two cautions an earlier draft got wrong:

- `Σ visible_k = U_n` telescopes **for any depth order**, correct or not — so it verifies
  bookkeeping only, the same trap as §2's 79.00. What pins semantics is **ordering**: a
  small disc in front of a big one must have its full disc visible and the big one
  `union − small`; swap them and the small one's visible area must become **zero**.
- Convex rings whose depth ranges overlap along the flow admit **no** total depth order, so
  the oracle is exact only for cleanly separated or nested sets. The prefix calls recompute
  the slab breakpoints, and float geometry produces differences at the ~1e-4 scale —
  compare with a tolerance, not `==`. And since it works on convex rings, it says nothing
  about the concave parts `projectHull` already hulls.

**The ceiling on what this buys.** `partCd` is a **whole-part** blend off the part's own
axis — `side·(1−c²) + fwd·c²₊ + back·c²₋` (`src/drag.h:1555-1568`). So even after #945 a
part cannot be charged its nose-cone's 0.35 on its tip and its skirt's 1.3 on its side;
occlusion fixes **who** votes, not **which surface** of the voter faces the air. That is the
dominant residual error once the silhouette is honest, and it is why the AoA case in §2
stays expensive.

---

## 5. Why #936 (the coefficient retune) was reverted

Not because the authoring is wrong — an exposed flat tank end arguing ~1.0 is fine physics
(an isolated flat circular plate is ~1.1–1.17 on its own area, and HEAD authors `fuel_tank`
fwd/back at 1.0). It is the wrong lever, and reading #936's measured effect through #945's
lens separates its two halves:

1. **The tank/engine half is exactly inert once #945 lands.** Every r2.25 tank, engine and
   decoupler disc it touched is covered axially, so its vote is zero either way. That half
   was pure compensation for the double-count — which is why it moved the hull mean
   0.875 → 0.532 while the ship's silhouette did not change at all.
2. **The residual of #936 sits entirely on the two adapter shoulders** — 12.68 m² of the
   79 m² visible set, the only parts it changed that are in that set. Layer #936 on top of
   #945 and the mean drops 0.438 → 0.366 (CdA 28.9 m²): two corrections for one error.
3. So the anchors do **not** stop mattering — an earlier draft of this report claimed they
   would become cosmetic, and that is wrong. What survives #945 is a specific authoring
   question that #936 raised and did not answer: is a conical shoulder's end-on cd really
   0.90, when that number is being applied against a **filled** r2.25 disc (§3's
   convex-hull footnote) rather than the ~8.8 m² annulus the part presents? Worth
   re-measuring after #945; not worth reverting #945 for.
4. It also forced a re-baseline of two golden skiff cases (`elevator-trim`,
   `skiff-per-surface-alpha`), which would then have needed a second one.

Draft kept in `tmp/keep936/` in case the re-measurement after #945 still wants the adapter
half of it.

---

## 6. Blast radius, measured

`parts.json` is **generated** — the anchors live in `utils/gen_parts.py` (`DRAG_CD`, ~line
101) and `make test`'s `test-parts` target runs `gen_parts.py --check`, so a hand edit to
the JSON fails the suite. Worth knowing before touching catalog data. (That pin is
conditional: Makefile:630-635 skips it when the `trimesh` python module is absent. It ran
on this box.)

A full e2e battery run (`python3 e2e/run.py --jobs 4`, 469 cases) with #936 applied:
**459/469 passed**. Ten failed:

- `skiff-trim-and-weathervane` — drag golden (`2.4 < aoa < 3.3`), the expected fallout.
- `orbit-plane-target-grounded`, `hold-the-job-runner-and-read-a-window-mid-bake` — passed
  on a re-run at `--jobs 2`: contention.
- `eva-board-round-trip`, `platform-eva-stands`, `systems-window-switches-a-group-off`,
  `systems-window-shuts-the-nozzles`, `title-load-spin`, `title-load-spin-t0`, `part-pick`
  — **fail at clean HEAD**.

Those seven are settled: re-run against a freshly built binary (the earlier runs used one
stale by exactly two objects — `cli.cpp` and `gameui.cpp`, the only files including
`src/version.h`, whose `git describe` stamp rewrites when the tree goes dirty, so the
physics was current and only the version string differed), all seven still fail, alone and
at `--jobs 1` (`tmp/e2e_freshbin.log`). Their shape is a UI/readout regression, not
physics: `[uiclick] … "…/EVA##eva0" FAILED not found (no such item drawn within 2.0s)`,
`EXPECT not found: 'ship=racer part=0 (capsule)'`, a Systems-window row-count CHECK, a
`[surf]` height-step CHECK. Worth their own ticket.

`make test` does not run e2e (`TESTS`, Makefile:612-620; the `test:` target at 644 runs
only those binaries). CI is not lint-only, but close: `.gitea/workflows/lint.yml` runs
`make test-gen`, which is `lint` plus the `sci_phase.py` / `sci_dist.py --check` table pins
(Makefile:686-689). Nothing in the normal loop executes the battery — that is why the seven
are news, and it will bite again on the #945 landing.

---

## 7. Cost, and what it deliberately does not fix

- **#945** is the M it was sized at: one new function in `drag.h` over machinery that
  already exists (the slab decomposition, with polygon identity kept), more scalars per memo
  node, the weighting change in one pass, the fingerprint gap closed, unit tests in
  `tests/test_drag.cpp` against the depth-sorted-union oracle, and one re-baseline of the
  golden drag cases.
- It does **not** fix the ascent. A law that arrives at AoA 31° inflates the silhouette
  **78%** (78.99 axial → 140.27) and pays for it whatever the coefficient says. #927 is that
  fix, and #927 should be tuned against the post-#945 model, which is why #945 goes first:
  the drag-limited climb speed in the ascent law is a function of the **axial** CdA, and the
  axial CdA is exactly what #945 changes.
- Measuring the post-#945 AoA behaviour is part of the work, not a line to quote from here.
  The axial read is easy (spawn in `flying-low`, `OSP_SHIP_DUMP=1`); an off-axis one needs
  the audit dump's trigger widened — `OSP_SHIP_DUMP_BETA` can only defer on beta, never on
  alpha.
- Related and unfixed: #944 (a ship flown below the datum keeps flying, engines lit, 71 km
  deep), #938 (ground residency for live hulls), #934/#935 (fuel accounting).

---

## 8. Provenance

| claim | source |
|---|---|
| §1 ascent tables | `tmp/drag936_before.log`, `tmp/drag936_after.log` — `SDL_VIDEODRIVER=offscreen ./osp --startship mars_explorer_mk1,res/ships/mars_explorer_mk1.json,Kerbin,pad --ascent 100e3 --time-accel 10 --drag-log --sim-steps 300 --no-paint`; the "after" run used the reverted #936 catalog via `--parts` |
| axial dump (§2, §3) | `tmp/dump_flying-low.log` — same binary, `OSP_SHIP_DUMP=1 … ,Kerbin,flying-low --no-paint --sim-steps 40`; header reads `alpha 0.0 beta 0.0` (`%.1f`), `noseDot 1.00` on all 30 parts. The `rot-orbit` variant (`tmp/dump_rot-orbit.log`) produced no dump |
| visible set, 0.438, CdA and terminal speeds | `tmp/visible936.py tmp/dump_flying-low.log` — upwind-first annulus over the dump's own `projA`. Carries `covered` as a scalar, so it assumes concentric nesting within a column; terminal speeds use `rho 1.1813`, the 375 m value from the ascent log |
| column tangency (§2) | dump `(right, up)` = ±2.89 / ±3.44 → radius 4.49; disc radius 2.25 from `nose_cap_r2.25` |
| broadside dump (`SUM 267.65 unionA 170.17 cd 1.01`) | `tmp/shipdump936.log`, `Kerbin,pad` (fires at `alpha -90` on the pad, because the spawn flow is the ground's rotation) |
| code walk §3, §4 | src/vehicle.cpp 3700-3800, 3838/3841, 1723-1744 (`aeroGeomFingerprint`); src/drag.h 337 (`dragForce` sign), 346-350 (`AeroFrame`), 461 and 519 (Mach corrections, lift side only), 831-832 and 839-846 (`flowPlane`, convexity contract), 1004-1103 (`unionProjectedArea`, `Span`, `bands`, guard at 1093), 1114-1124 (memo invariants and the 8.3% jump note), 1120+ (`SilhouetteMemo`), 1555-1568 (`partCd`) |
| sweep, 459/469 | `tmp/e2e_sweep936.log` (4 jobs), `tmp/e2e_retry936.log` (2 jobs), `tmp/e2e_cleanbase.log` (7 at HEAD), `tmp/e2e_single.log` (2 alone), `tmp/e2e_freshbin.log` (7 at HEAD against a current binary) |
