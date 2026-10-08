# Porkchop grid resolution — 2026-10-08

## The question

`--porkchop-n` is 40, a leftover from when the sweep blocked the main thread.
It runs off-thread now (`job.h`, `main.cpp:1390`), so what should it be? The
specific worry: **a coarse plot may miss transfers entirely while still
reporting a plausible dv at a much later departure.**

Measured on five transfers at 40 / 128 / 256 / 512 / 1024 / 2048 (and 4096 for
Mercury→Pluto), scoring every coarser grid against the finest one.

## Method

Scaffolding added for this (all useful beyond it):

- `--porkchop-bench 40,128,256,512` — sweeps several grid sizes back to back
  off **one** ship/target snapshot, logging each. Separate runs would move the
  snapshot too: P fires on a frame boundary, so wall-clock jitter changes the
  departure epoch. One process makes the grids differ only in resolution.
- `--porkchop-dump DIR` — writes each grid as CSV (axis ranges + argmin in the
  header comment) for offline analysis.
- `heli-mercury` / `heli-earth` / `heli-jupiter` / `heli-uranus` scenario beds
  (`vehicle.cpp`): a circular heliocentric orbit at a planet's semi-major axis.
  The planner only lists children of the orbited body
  (`transferplanner.cpp:80-92`), so a planet→planet transfer needs the ship
  Sun-centred. The three moon transfers needed nothing new — `high-orbit`
  around the planet already resolves to that planet's inertial frame.
- `[porkchop]` lines now carry `sweep=<ms>`.

System: `res/systems/solar_system_named.json` (Rosalind and Margaret exist only
in `_full`/`_named`, not `_measured`). Ship: `res/ships/racer.json`.
Analysis: `analyze.py` (grid-vs-reference), `probe.py` (grid smoothness).

Cost is a clean n²: **17.4 µs/cell** — 40² = 25 ms, 512² = 4.3 s,
2048² = 102 s, 4096² = 283 s. Memory is n²·8 bytes (4096² = 134 MB).
Sizes are capped (`--porkchop-n` ≤ 2048; `--porkchop-bench` ≤ 4096, at most
8 entries): CLI11 splits a comma list only with `->delimiter(',')`, and
without it its number lexer reads `40,128` as the integer 40128 — a 40128²
grid is 13 GB, which is how this harness briefly ate all the machine's RAM.

## Results

`dv_err` = this grid's best dv minus the reference grid's best. `t_dep_err` =
departure delay difference. `near_best` = the best dv the reference grid offers
within one coarse cell of where this grid pointed — i.e. what you could still
have got from the plan this plot handed you.

### Jupiter → Carme (ship in a 2.97 h Jovicentric parking orbit)

| n | dv_min | dv_err | dv_err% | t_dep_min | t_dep_err | near_best |
|---|---|---|---|---|---|---|
| 40 | 22158 | +2774 | 14.31% | 324 d | +260 d | 20588 |
| 128 | 20122 | +738 | 3.81% | 99.5 d | +35.7 d | 19445 |
| 256 | 19505 | +120 | 0.62% | 90.9 d | +27.1 d | 20839 |
| 512 | 19463 | +79 | 0.41% | 26.1 d | −37.7 d | 19778 |
| 1024 | 19467 | +83 | 0.43% | 73.4 d | +9.6 d | 33508 |
| 2048 | 19384 | 0 | — | 63.8 d | — | 19384 |

### Uranus → Margaret (ship in a 3.14 h Uranocentric parking orbit)

| n | dv_min | dv_err | dv_err% | t_dep_min | t_dep_err | near_best |
|---|---|---|---|---|---|---|
| 40 | 6936 | +541 | 8.46% | 217 d | −167 d | 6398 |
| 128 | 6513 | +119 | 1.85% | 107 d | −277 d | 6454 |
| 256 | 6394 | −0.1 | −0.00% | 352 d | −31.9 d | 8672 |
| 512 | 6450 | +56 | 0.87% | 345 d | −39.2 d | 10155 |
| 1024 | 8045 | **+1651** | **25.82%** | 3.3 d | −381 d | 7663 |
| 2048 | 6394 | 0 | — | 384 d | — | 6394 |

### Uranus → Rosalind (target period 0.558 d, comparable to the ship's 3.14 h)

| n | dv_min | dv_err | dv_err% | t_dep_min | t_dep_err | near_best |
|---|---|---|---|---|---|---|
| 40 | 16816 | +1870 | 12.51% | 9.62 h | +6.1 min | **14945** |
| 128 | 15137 | +192 | 1.28% | 9.50 h | −1.3 min | 14945 |
| 256 | 15101 | +156 | 1.04% | 9.62 h | +5.9 min | 14989 |
| 512 | 14984 | +39 | 0.26% | 9.50 h | −1.5 min | 14945 |
| 1024 | 14954 | +8 | 0.05% | 9.47 h | −2.9 min | 14951 |
| 2048 | 14945 | 0 | — | 9.52 h | — | 14945 |

### Mercury → Pluto (ship in an 88 d heliocentric orbit)

| n | dv_min | dv_err | dv_err% | t_dep_min | t_dep_err | near_best |
|---|---|---|---|---|---|---|
| 40 | 25059 | +2269 | 9.95% | 146.2 yr | +23.9 yr | 23434 |
| 128 | 24088 | +1298 | 5.69% | 93.7 yr | −28.7 yr | 25852 |
| 256 | 23430 | +640 | 2.81% | 133.3 yr | +10.8 yr | 22865 |
| 512 | 23080 | +290 | 1.27% | 135.5 yr | +13.0 yr | 24299 |
| 1024 | 22882 | +92 | 0.40% | 117.6 yr | −5.0 yr | 35450 |
| 2048 | 22781 | −10 | −0.04% | 127.4 yr | +4.9 yr | 32700 |
| 4096 | 22790 | 0 | — | 122.4 yr | — | 22790 |

### Jupiter → Earth (ship in an 11.9 yr heliocentric orbit)

| n | dv_min | dv_err | dv_err% | t_dep_min | t_dep_err | near_best |
|---|---|---|---|---|---|---|
| 40 | 11955 | +24 | 0.20% | 140.5 d | +38.8 d | 11945 |
| 128 | 11934 | +3 | 0.02% | 92.0 d | −9.7 d | 11932 |
| 256 | 11932 | +0.3 | 0.00% | 106.0 d | +4.3 d | 11931 |
| 512 | 11931 | +0.1 | 0.00% | 103.6 d | +1.9 d | 11931 |
| 1024 | 11931 | 0.0 | 0.00% | 101.0 d | −0.7 d | 11931 |
| 2048 | 11931 | 0 | — | 101.7 d | — | 11931 |

## What actually decides it

The departure axis's fine structure is set by the **ship's own orbital
period**, not the target's. `transferplanner.cpp:213-224` sets the auto window
to **one target period**, so the departure step is `target_period/(n-1)` and
the number of samples per ship orbit is

    n × P_ship / P_target

independent of anything else. That one column explains every result above:

| transfer | P_ship | P_target | ship orbits in the window | samples/orbit @40 | @512 | @2048 |
|---|---|---|---|---|---|---|
| Jupiter→Earth | 4331 d | 365 d | 0.084 | 477 | — | — |
| Uranus→Rosalind | 3.14 h | 0.558 d | 4.3 | 9.4 | 120 | 481 |
| Mercury→Pluto | 88.0 d | 247.9 yr | 1030 | 0.039 | 0.50 | 1.99 |
| Jupiter→Carme | 2.97 h | 702.3 d | 5681 | 0.0070 | 0.090 | 0.36 |
| Uranus→Margaret | 3.14 h | 1694.8 d | 12951 | 0.0031 | 0.040 | 0.16 |

Above ~10 samples per ship orbit the plot is converged (Rosalind at n=40 is
12.5% off on dv but its argmin sits in the *global* best basin — `near_best` =
14945 = the reference optimum). Below ~1 sample per orbit the departure axis is
aliased and nothing converges.

The aliasing is visible directly (`probe.py`), Jupiter→Carme at 2048², walking
the departure axis at the argmin ToF (step 8.23 h, ship period 2.97 h):

    28561 → 102081 → 91093 → 69781 → 41631 → 19384 → 97574 → 80739 → 55124 → 26620 → 101646

Median adjacent jump 24.0 km/s along departure, but only 125 m/s along ToF. The
departure axis is a sawtooth of amplitude ≈ 2·v_parking (84 km/s for a 42 km/s
parking orbit): dv_dep = |v_transfer − v_ship| and v_ship rotates once per
parking orbit. **The ToF axis is fine at n=40-ish; the departure axis is the
whole problem.**

Two side effects of that:

- **dv_min is not monotonic in n.** Grids are not nested (n−1 intervals, so the
  sample points move), and on a sawtooth a coarser grid can land on a lucky
  tooth bottom. Uranus→Margaret at n=1024 reports 8045 m/s, 25.8% *worse* than
  its own 2048 reference and worse than n=40.
- **Commensurability accidents.** Mercury→Pluto at n=1024 has a departure step
  of 7.648e6 s = 88.5 d ≈ exactly one Mercury period, so every column samples
  the same ship phase and the row is smooth (median jump 592 m/s). That is luck,
  not resolution — at n=512 the step is 177 d and the same row is aliased.

How narrow the good region is, from the reference grids:

| transfer | cells within 5% of the best |
|---|---|
| Jupiter→Earth | 1.14% |
| Jupiter→Carme | 0.008% |
| Uranus→Rosalind | 0.009% |
| Uranus→Margaret | 0.008% |
| Mercury→Pluto | 0.003% |

## Answers to the question

1. **Best dv converges well before 512** for every transfer measured: 14.3% →
   0.4% (Carme), 8.5% → 0.9% (Margaret), 12.5% → 0.3% (Rosalind), 10.0% → 1.3%
   (Pluto) going 40 → 512. Jupiter→Earth is already converged at 40 (0.20%).
2. **Departure time never converges for moon targets.** Carme wanders
   324 d → 26 d → 73 d → 64 d; Margaret 217 d → 3.3 d → 384 d. The dv at those
   different epochs is nearly the same, so the *cost* of the wrong epoch is
   small — but "Send best" (`gameui.cpp:913-923`) schedules the burn at
   `pc_computed_at + t_dep_min`, so the player is pointed at an arbitrary
   departure date. Your fear is confirmed for the epoch, and it is benign for
   the dv.
3. **Raising n is the wrong lever.** To get 10 samples per ship orbit over the
   auto window you need n ≈ 57,000 (Carme) and n ≈ 130,000 (Margaret) —
   10⁹–10¹⁰ cells. n² scaling buys nothing when the window itself is the wrong
   shape.

## What I'd do instead

- **Decouple the axes.** `porkchopGrid` already takes `n_dep` and `n_tof`
  separately; `transferplanner.cpp:246-248` passes `n, n`. Spend the budget on
  whichever axis needs it — for Carme, `n_dep=16384, n_tof=256` costs the same
  4.2M cells as 2048² but samples the parking orbit 8× better.
- **Set the departure window from the ship, not the target.** One target period
  is right when P_ship ≫ P_target (planet→planet) and nonsense when the ship is
  in a 3 h parking orbit. A window of a few ship periods (a phasing plot) is
  what a player can actually act on.
- **Refine the argmin** with a local search (~100 extra solves) so dv and
  t_dep are sub-cell for a fraction of any n increase.
- **Separate bug, probably more important than n:** the surface has
  non-physical cliffs. In the Pluto 1024² grid, adjacent departure cells one
  Mercury period apart (Pluto moves 0.35° between them) report 119435 and
  22882 m/s. `dv_hi` (the 95th percentile, `transfer.h:275-286`) is ~118.7 km/s
  on every Pluto grid, so ~5% of cells sit on that spike. The bracket scan in
  `solveLambert` (`transfer.h:79-99`) takes the *first* sign change scanning z
  upward, so it can land on a different conic branch in neighbouring cells.
  Hohmann for 0.39→39.5 AU needs ~19.5 km/s at departure, so 22.9 km/s is the
  physical branch and 119 km/s is the solver failing to find it.

## Repro

```
# one transfer, all sizes, grids dumped for analysis
SDL_VIDEODRIVER=offscreen ./osp --system res/systems/solar_system_named.json \
  --startship x,res/ships/racer.json,Jupiter,high-orbit --transfer-target Carme \
  --porkchop-bench 40,128,256,512,1024,2048 --porkchop-dump tmp/pc/jupiter-carme \
  --sim-press 3000,0,P --timeout 400

python3 reports/porkchop-resolution2026_10_08/analyze.py tmp/pc/jupiter-carme
python3 reports/porkchop-resolution2026_10_08/probe.py tmp/pc/jupiter-carme/porkchop_Carme_2048x2048.csv
```

Raw logs and CSVs: `tmp/pc/<transfer>.log`, `tmp/pc/<transfer>/*.csv`.
`make test` and `e2e/run.py 20-porkchop 16-transfer-body 17-transfer-ship` pass
with the `sweep=` field added to the `[porkchop]` line.

## Caveats

- The reference is the finest grid, not the true optimum. For Carme, Margaret
  and Pluto the reference itself is aliased in the departure axis
  (0.16–2.0 samples per ship orbit), so the true dv_min is probably *lower*
  than the numbers above and the `dv_err` figures are upper bounds on the
  error, not the error.
- One ship (`racer.json`) and one parking bed (`high-orbit`) per planet. The
  parking orbit's period is the controlling variable, so a different bed
  changes the numbers by design — the samples-per-ship-orbit rule is what
  transfers.
- The heliocentric beds are circular at the planet's semi-major axis, so they
  match the origin planet's period but not its instantaneous radius or
  heliocentric longitude.
