# Orbital Stability of Open Space Program at High Time Acceleration

**Scope.** Characterize how the ship's orbit behaves as a function of the CLI
options `--start` (which body/orbit), `--time-accel` (× real time), and
`--orbit-log` (the measurement), using headless runs of the shipped binary.
**Bottom line first:** there is a real, secular, *outward* energy drift that
appears **only while the ship is integrated in a rotating frame**, that scales
**linearly with the physics substep size `h`**, and that makes a 75 km Eerbon
orbit creep out by **~2 % per simulated day** in the "stable" regime and
**rapidly expand out of its sphere of influence** once `h` exceeds ~0.1 s.
The root cause is that the game's semi-implicit Euler is **not symplectic for
the Coriolis (velocity-dependent) term**. Two clean fixes are validated in
§7 (integrate in the inertial frame, or use a symplectic scheme).

---

## 1. TL;DR

| Finding | Evidence |
|---|---|
| Drift is **secular & outward** (energy rises toward 0), not bounded oscillation | §4, fig 1–2 |
| Drift occurs **only in rotating frames**; the inertial control is ~5 orders of magnitude flatter | §5, fig 3 |
| Drift rate **∝ `h`** (substep size); a **plateau** in `ta` wherever `h` is clamped at 0.1 s | §6, fig 1 |
| **~2 %/sim-day** sma growth for a 75 km Eerbon orbit at `h=0.1` (the `ta = 50…10⁴` window) | §8 |
| `ta > 10⁴` (`h > 0.1`): orbit **expands out of the SOI**, then settles (drift self-limits once inertial) | §6, fig 4 |
| Mechanism: **semi-implicit Euler + Coriolis is non-symplectic** → O(`h`) secular drift | §7, `fixtest.py` |
| Fixes (validated): **inertial-frame integration** or **implicit midpoint**; exact-Coriolis-rotation is *not* enough | §7 |

---

## 2. How the game integrates (the relevant code)

- **Substep model** (`src/main.cpp` ~L2007). Each logic step advances sim time
  by `step = 0.02·ta` (s). It is split into
  `n = clamp(round(step/0.1), 3, 2000)` substeps of size `h = step/n`.
  Hence **`h = 0.1 s` for `50 ≤ ta ≤ 10⁴`** (a plateau), `h = 0.02·ta/3` below
  that, and `h = step/2000` above it. Sim time advances at `wall × ta`; the
  orbit-log fires at the logic-loop rate (≤ ~50/s), independent of `ta`.
- **Forces** (`applyGravity`, L707–737). Central gravity `−μ p/r³` plus, **only
  when in a rotating frame**, the fictitious acceleration from
  `frame.h::GetFictitiousAccel`: `−2 ω×v` (Coriolis) `− ω×(ω×p)` (centrifugal),
  with `ω = (0, −Ω, 0)`. No drag. So orbital stability here is a **pure
  integrator property**.
- **Update** is semi-implicit (symplectic) Euler: `v ← v + h·a(p,v)`;
  `p ← p + h·v`.
- **Bodies used.** Eerbon (r=600 km, μ=3.5316e12, Ω=2.9157e-4 rad/s, rot-SOI
  700 km); Moon (r=200 km, μ=6.5138e10, Ω=4.52e-5, rot-SOI 300 km); Sun
  (μ=1.17e18, inertial frame, no rotating frame).

The Coriolis term `−2 ω×v` is a **magnetic (Lorentz-like) force**: it is
velocity-dependent and does no work in the exact continuous system, but the
discretization of it is what matters.

---

## 3. Method

- **Runs.** 35 headless jobs (4 parallel, one Xvfb each) via
  `driver.py`, each `timeout <wall> ./osp --start S --time-accel T
  --orbit-log --orbit-interval I`. Phase A: ~2 orbital periods at
  `ta ∈ {1,10,50,100,1000,10000}` (plus an inertial Sun control at higher
  `ta`). Phase B: `ta ∈ {3·10⁴,10⁵,10⁶}` for 45 s wall (breakdown regime).
- **Measurement.** `analyze.py` parses the `[orbitlog]` lines and reports, per
  run: `dE/E` (relative energy change), a robust **initial drift rate**
  `rate_init` (J kg⁻¹ s⁻¹, fit over the first 10 % of samples — immune to
  late-run dilution), the bounded-oscillation amplitude, `da/a`, max
  eccentricity, and the fraction of samples spent in a rotating frame.
- **Mechanism check.** `replica.py` / `fixtest.py` replay the exact substep in
  NumPy for a 675 km circular Eerbon orbit and compare integrator schemes,
  measuring the *inertial* specific energy (the conserved quantity).

Raw logs: `runs/*.log`. Plots: `figs/`. Per-run numbers: `summary.json`.

---

## 4. The drift is secular and outward

For the 75 km circular Eerbon orbit (start 2), specific energy monotonically
**rises toward zero** (the orbit expands) with no sign of bounding:

```
s2  ta     h(s)    rate_init (J/kg/s)   dE/E      da/a
    1      0.0067  0.0273               2.7e-5    2.7e-5
    10     0.0667  0.3919               5.9e-4    5.9e-4
    50     0.1     0.5953               1.0e-3    1.0e-3
    100    0.1     0.5945               1.2e-3    1.2e-3
    1000   0.1     0.6084               4.4e-3    4.4e-3
    10000  0.1     0.6072               3.6e-2    3.7e-2
```

`fig 2` shows ΔE(t) is a straight line on a log-log plot (secular, not a
bounded oscillation) for every `ta` in the stable window. The eccentricity
stays tiny (e < 0.001) in this regime — `fig 5` — so the orbit is not
"going wild"; it is quietly **gaining energy and expanding**.

---

## 5. The drift lives in the rotating frame

- **Inertial control (start 5, Sun, never in a rotating frame):**
  `dE/E ~ 4e-9 … 2.4e-7` over `2·10⁷ s`; `rate_init ~ 1.2e-5 J/kg/s` at
  `h=0.1` — **~5×10⁴ times smaller** than the Eerbon rotating case
  (0.595 J/kg/s at the same `h`). This is the integrator's ordinary O(h²)
  bounded error, not the Coriolis drift.
- **Duty-cycle scaling (fig 3).** Orbits that cross the SOI (starts 3 and 6)
  spend only ~10 % of their time in the rotating frame, and their measured
  rate is ~10× the fully-rotating Eerbon rate once divided by that fraction.
  Within a body the duty-corrected rates agree; across bodies they differ by
  the body's `Ω` and orbital `v` (Eerbon ≫ Moon, see below).
- **Self-limiting.** At `ta=10⁶` the start-2 orbit has expanded from 675 km
  to ~977 km sma, **entirely outside the 700 km SOI**; it is then in the
  inertial frame, `rot_frac` drops to ~0.1 %, and the drift **stops**. The
  Coriolis drift is only "on" while the ship is in the rotating frame.

**Conclusion:** the secular drift is produced by integrating the
velocity-dependent Coriolis term in a rotating frame. Pure Kepler (inertial)
integration with the same integrator is clean.

---

## 6. Scaling: ∝ `h`, the plateau, and the breakdown regime

**∝ `h`.** The standalone replica (exact game substep, 675 km circular
Eerbon) gives a clean linear law:

```
 h (s)      0.0067   0.067    0.1      1.0     10.0
 rate       0.0408   0.4071   0.6065   5.79    39.6   J/kg/s
 rate/h     6.09     6.08     6.07     5.79    3.96   J/kg
```

`rate ≈ 6·h` (J kg⁻¹ s⁻¹) holds to ~5 % for `h ≤ 1 s`. The in-game
plateau values (`ta = 50…10⁴`, `h=0.1`) match the replica to within a percent
(0.60 vs 0.6065). This is the signature of a **first-order, non-symplectic
discretization of the Coriolis term**.

**The plateau (fig 1).** Because the substepping clamps `h = 0.1 s` for
`50 ≤ ta ≤ 10⁴`, the drift *rate* is **flat across four decades of
`--time-accel`** — a direct fingerprint of the `h` clamp, and the reason
"just lower the time acceleration" does *not* help inside that window.

**Breakdown regime (`ta > 10⁴`, `h > 0.1`).** The rate grows ∝ `h` until the
orbit is pushed **out of the rotating SOI**, after which it settles in the
inertial frame and the drift stops (fig 4). It is a rapid secular expansion
(675 → ~977 km, e up to ~0.03), not a numerical blow-up to infinity — but the
orbit is no longer the one the player intended.

**Cross-body (Ω·v).** The drift scales with the body's rotation and the
orbital speed. At `h=0.1`: Eerbon (start 2) ~0.60 vs Moon (start 4)
~0.003 J/kg/s — **~200× smaller** for the slower-spinning, slower Moon orbit,
consistent with the Coriolis `∝ Ω·v` (and `Ω²v²`) dependence.

---

## 7. Mechanism and validated fixes

**Mechanism.** In the rotating frame the equations are a Hamiltonian system
with a constant "magnetic field" `B = 2ω` (the Coriolis force) plus the
gravity + centrifugal potential. The game's semi-implicit Euler applies the
Coriolis as a plain kick `v ← v + h(−2ω×v)`, i.e. `v ← (I − 2h[ω]) v`. That
matrix is **non-orthogonal** (`|eigenvalues| = √(1+4h²|ω|²) ≠ 1`), so each
step grows `|v|²` by `1 + 4h²ω²` — an O(h²)-per-step, **O(h)-per-second
secular energy injection**. Pure Kepler (no Coriolis) with the same
integrator *is* symplectic, which is why the inertial control is clean.

**Fixes — measured in `fixtest.py`** (inertial specific energy over ~11
periods, 675 km circular Eerbon):

```
 scheme            h=0.1 s              h=1.0 s
 game (as-is)      +0.6065 J/kg/s       +5.79 J/kg/s     ← the bug
 exact-Coriolis    −0.516 J/kg/s        −5.40 J/kg/s     ← NOT enough (order error)
 implicit-midpoint ~0.000 J/kg/s        ~0.000 J/kg/s    ← drift removed
 inertial frame    ~1e-8 rel.           ~1e-8 rel.       ← drift removed
```

- **(A) Integrate in the non-rotating (inertial) frame.** The game already has
  `Frame::getNonRotFrame()` and the frame-switch identity in `frame.h`. With
  no Coriolis term the system is pure Kepler and the existing symplectic
  Euler is exact-to-O(h²) bounded error. **Root-cause fix; no new integrator.**
  Cost: a surface ship must be given its co-rotation (stasis) velocity to sit
  still — bookkeeping, not physics.
- **(B) Stay in the rotating frame but use a symplectic scheme** — the
  implicit midpoint method is symplectic for this magnetic-Hamiltonian system
  and removes the drift (validated above).
- **(C) Quick mitigation (no integrator change):** cap the effective substep
  (`h ≤ 0.1 s`) even at high `ta`. This *bounds* the drift (keeps it in the
  2 %-per-day regime) but does not remove it, and costs more substeps.

Recommendation: **(A)** as the clean fix, **(B)** if the rotating frame must
be kept for terrain/surface reasons, **(C)** as a stopgap.

---

## 8. Practical impact

At the top of the "stable" window (`h = 0.1 s`, i.e. `--time-accel` 50 to
10⁴), a **75 km circular Eerbon** orbit gains ~0.60 J kg⁻¹ s⁻¹:

- **≈ 2 % of semi-major axis per simulated day** (≈ 1 % of binding energy per
  day). Over a simulated month the orbit has roughly **doubled** its radius.
- In wall-clock terms at `ta = 10⁴`, a simulated day is ~8.6 wall-seconds, so
  the 2 %-per-day drift is visible within a minute of play.
- The **Moon** orbit's absolute drift rate is ~200× smaller (0.003 vs
  0.60 J kg⁻¹ s⁻¹) because it spins slower and orbits slower; in relative
  terms it still grows ~0.2 %/sim-day (≈ 10× less than the 75 km Eerbon
  orbit's 2 %/sim-day, the smaller gap reflecting its much weaker binding).
- **`ta > 10⁴`:** expect the orbit to expand well beyond its SOI within a few
  simulated days (start 2: 675 → ~977 km by `ta=10⁶`), i.e. the intended
  orbit is lost.

So: **time acceleration up to ~10⁴ is usable** (slow, bounded-rate creep),
**above ~10⁴ the orbit is no longer preserved.** The creep is not a visual
glitch — it is a genuine secular error in the physics.

---

## 9. Reproduce

```bash
cd /home/ubuntu/openspaceprogram
python3 stability/driver.py      # 35 headless runs -> stability/runs/*.log
python3 stability/analyze.py     # table + stability/figs/*.png + summary.json
python3 stability/fixtest.py     # integrator-scheme validation (mechanism)
```

Single run example:
```bash
xvfb-run -a -s "-screen 0 1920x1080x24" timeout 52 ./osp \
  --start 2 --time-accel 100 --orbit-log --orbit-interval 0.124
```

**Files.** The analysis lives in `stability/`: `driver.py` (matrix driver),
`analyze.py` (parsing/plots), `replica.py` + `fixtest.py` (mechanism +
fixes), `runs/` (raw + parsed logs), `summary.json` (per-run metrics). The
five figures are bundled alongside this report in `figs/` (also in
`stability/figs/`).

---

## 10. Figures

| Fig | File | Shows |
|---|---|---|
| 1 | `fig1_drift_rate_vs_ta.png` | Drift rate vs `--time-accel`; the `h=0.1` plateau; rotating-only |
| 2 | `fig2_s2_E_t.png` | Secular (log-log linear) energy growth, start 2 |
| 3 | `fig3_duty_corrected.png` | Duty-corrected rate ⇒ produced in the rotating SOI only |
| 4 | `fig4_escape.png` | Breakdown regime: radial expansion out of the SOI |
| 5 | `fig5_ecc_osc.png` | Bounded (tiny) eccentricity in the stable regime |
