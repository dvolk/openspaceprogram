# Why ApT/PeT decay slower than 1 s/s at 1x time accel

Date: 2026-08-26
Status: investigation complete, no code written (temporary debug fields in
`--orbit-log` were reverted; `make clean && make && make test && make e2e`
all green — 15/15 e2e)

## TL;DR

**The clock is right, the countdown math is right, and the ship's trajectory
is right.** In a genuine elliptic orbit, ApT/PeT decay at exactly **1.000
s/s** at 1x (measured).

The "slower" decay only appears in **near-circular orbits**, where ApT/PeT
are mathematically undefined — a circle has no apsis. The game still displays
a number: it is derived from the direction of the fitted orbit's *residual*
eccentricity vector, which in this game is not a physical property but
**secular error of Bullet's semi-implicit-Euler integrator**. That residual
grows linearly (~3e-8/s at 1x, rate proportional to step size) and its
apsidal line **precesses at exactly n/2** in the inertial frame (measured
independent of step size). A countdown that tracks that numerical apsidal
line therefore decays at 1 − 0.5 = **0.5 s/s** in the clean single-part case
(measured: 0.50), and at 0.115–0.24 s/s for multi-part ships, where the
weld-constraint solver's error shifts the effective precession to 0.88n
(inertial) / 0.76n (rotating).

A pure-Python replica of Bullet's integrator reproduces the single-part game
measurements to 3 significant figures (de/dt 3.085e-8/s vs 3.1e-8/s;
precession 0.500n vs 0.50n), confirming the root cause.

**Fix recommendation: do not patch Bullet.** Coasting ships should use the
game's existing exact-Kepler coasting (the "rails") at 1x, as they already do
at >=10000x — the machinery exists and is proven, and a coasting ship is
thrust-free and torque-free by construction, so the behavior is identical
except that the orbit is exact. See section 7.

## 1. The question

> "it's my understanding that at 1x time accel the game runs at 1s/s, and
> that in orbit and no thrust, ApT (time to apoapsis) and PeT should change
> at 1s/s too, but they seem to be slower. am i misunderstanding or is there
> something wrong? if it's wrong can you debug?"

## 2. What was verified correct

1. **The 1x clock.** Fixed `dt = 1/50` (src/main.cpp), wall-clock
   (`SDL_GetTicks`) accumulator, `time += dt * time_accel`. The sim-time
   stamps in the logs below advance in exactly 2.0 s intervals at 1x.
2. **The countdown math, for a real orbit.** `ellipse-peri` scenario
   (e = 0.448, T = 3885 s), 60 s at 1x:
   ```
   t=1.7s  ttAp=1940.00  ttPe=3881.73
   t=3.7s  ttAp=1938.00  ttPe=3879.74
   t=5.8s  ttAp=1935.98  ttPe=3877.72
   ```
   Exactly **1.000 s/s** (ttPe starts at T at periapsis, ttAp at T/2 — the
   expected values).
3. **The ship's motion.** In `inertial-orbit` (r = 725 km), the dbg position
   log gives an angular rate of 0.1740 deg/s vs n = 0.17413 deg/s — the ship
   orbits at exactly the right rate.
4. **The orbit fit** (`computeOrbitElements`, src/orbit.h): elements and
   anomaly math are standard and correct; the `ecc < 1e-9` guard reports
   argp = nu = 0 for a perfect circle, which yields a stable (T/2, T)
   display.
5. **The rotating-frame fictitious forces** (`GetFictitiousAccel`,
   src/frame.h: -2w x v - w x (w x p)): the math checks out, and the
   inertial-frame scenario (pure central gravity, no fictitious forces)
   shows the same countdown drift — ruled out as a cause.

## 3. What was measured wrong (the observation in question)

Countdown decay at 1x, near-circular orbits (40–45 s runs; decay =
-dtAp/dt):

| scenario | ship | frame | ttAp decay | apsidal-line precession |
|---|---|---|---|---|
| inertial-orbit (r=725 km) | single capsule | inertial | **0.50 s/s** | **0.50 n** |
| inertial-orbit (r=725 km) | racer (4 parts) | inertial | 0.115 s/s | 0.88 n |
| rot-orbit (r=685 km) | racer (4 parts) | rotating | 0.24 s/s | 0.76 n |
| ellipse-peri (e=0.448) | racer (4 parts) | rotating | 1.000 s/s | ~0 |

The fitted eccentricity of the "circular" orbits grows linearly: ~3.1e-8/s
(racer, inertial), ~5e-8/s (racer, rotating), reaching ~1e-6 after a minute.
Tiny — but the HUD countdowns are built from the *direction* of that vector,
so even this microscopic e fully determines the displayed ApT/PeT.

## 4. Mechanism

- In a circular orbit ApT/PeT are undefined. `computeOrbitElements` derives
  them from the fitted eccentricity vector's direction (argp) and the ship's
  position along the orbit (nu). When the true e = 0, the fitted e is 100%
  numerical, and the displayed countdowns are a countdown to a *numerical*
  apsis.
- Bullet integrates with **semi-implicit (symplectic) Euler**: `v += a*h`,
  then `x += v*h` (`btDiscreteDynamicsWorld::internalSingleStepSimulation`
  -> `btRigidBody::integrateVelocity` / `integrateTransforms`). Applied to a
  near-circular Kepler orbit, this map has a **secular** (never-vanishing,
  time-growing) error: the eccentricity vector grows linearly and its
  direction precesses. Two measured properties of that precession: it is
  **exactly n/2** in the inertial frame, and it is **independent of step
  size h** — so it is a property of the discrete map, not of step-size
  magnitude.
- Countdown rate = 1 - (precession/n) s/s. At 0.5n precession that is
  **0.5 s/s** — the single-part measurement.
- Multi-part ships add `btSequentialImpulseConstraintSolver` (6DOF weld)
  error on top, shifting the effective precession to 0.88n (inertial) /
  0.76n (rotating) -> the observed 0.115 / 0.24 s/s.

## 5. Decisive reproduction (pure Python)

Replicated Bullet's integrator exactly (semi-implicit Euler, game's
h = 0.02/3 s, mu = 3.532e12, r0 = 725 km, 45 s) —
`tmp/symplectic_orbit.py`:

| h (s) | de/dt (1/s) | apsidal precession |
|---|---|---|
| 0.00667 (game 1x) | **3.085e-8** | **0.500 n** |
| 0.02 | 9.26e-8 | 0.500 n |
| 0.05 | 2.31e-7 | 0.500 n |

vs the single-part game measurements: de/dt = 3.1e-8/s, precession 0.50n,
ttAp decay 0.50 s/s (which the Python map predicts as 0.5 s/s). Three
significant figures of agreement on all three quantities: the single-body
game case *is* the integrator's secular error, and nothing else.

## 6. Ruled out

- **The 1x clock** — exact 2.0 s log intervals.
- **Orbit fit / countdown math** — exact 1.000 s/s for the elliptic case.
- **Rotating-frame fictitious forces** — inertial scenario shows the same
  drift with pure central gravity; formulas verified.
- **Tidal / finite-size net force** across the ship — estimated ~1e-9 m/s^2,
  >1000x too small to drive the observed eccentricity growth.
- **Bullet kinematic state saving** — `saveKinematicState` only affects
  kinematic bodies; ships are dynamic.
- **`ApplyCentralForce` normalization** — net force is correct; gravity
  applied at the COM, no spurious torque.
- **More substeps would not fix this** — the precession (hence the
  countdown rate) is independent of h; only the tiny e magnitude scales with
  h. More substeps = more cost for no visible countdown improvement.

## 7. Fix options

**Option 1 — auto-rails at 1x (recommended).**
The game already coasts ships exactly via `propagateKepler` ("rails") at
>=10000x, and the main loop already ticks railed ships analytically at *any*
warp (`if(s->onRails) s->railsTick(dt*time_accel)` — "exact for any step
size, at any time accel"). A coasting ship (throttle 0, rotation keys off)
is thrust-free **and** torque-free by construction in this game — attitude
only changes via explicit `applyRotationForce` commands, there is no free
drift — so rail coasting is behaviorally identical, except the orbit is
exact. Then:

- elliptic countdowns: exactly 1.000 s/s (elements constant every tick);
- circular orbits: the fit hits the existing `ecc < 1e-9` guard -> stable
  T/2 and T display, no drift;
- faster, not slower: O(1) per tick instead of n substeps of the full Bullet
  pipeline;
- no vendored-code fork; the 10000x path already exercises
  `goOnRails`/`leaveRails`, SOI switching, and grounded freezing.

Change: enter rails when coasting even below `kRailsWarp` (throttle 0 + no
rotation commands + `canRail()`), and let the existing control-input
`leaveRails()` path (currently warp-gated) wake the ship. Staging while
railed is already not excluded at 10000x, and SOI/grounded cases are handled
by the existing rails code.

**Option 2 — HUD "—" for effectively-circular orbits.**
Smallest, honest change (show a dash, or stable T/2 and T, when the fitted e
is below a threshold). The physics is still numerically sloppy, but nothing
else displays it. Can be combined with option 1.

**Option 3 — patch Bullet to RK4 — not recommended.**

- Bullet's substep is semi-implicit Euler **plus an iterative velocity
  projection** (`btSequentialImpulseConstraintSolver`). RK4 needs the
  dynamics evaluated at four trial states, but constraint forces are the
  *output* of a projection, not a function of state. An RK4 patch would
  therefore only cleanly cover unconstrained (single-part) bodies;
  multi-part ships keep the weld-solver error (part of the measured
  0.88n/0.76n precession). Done properly — re-solving constraints at every
  stage — it is a dynamics-core rewrite, not a patch.
- The sequential-impulse solver is derived and tuned around the plain
  v-update; changing only the free integration risks contact/weld jitter
  and re-tuning.
- A permanent fork of vendored Bullet (`middleware/bullet3`); upstream never
  ships RK4 constrained dynamics, for exactly these reasons.
- 4x cost (QWEN.md: performance matters) and only asymptotically better:
  RK4 has its own (smaller) secular error, so a circular-orbit countdown
  would still track the numerical e.

**Option 4 — more substeps — rejected.** See section 6: the countdown rate
is h-independent.

## 8. Side findings

- **`--orbit-log` and `--dbg-log` share a timestamp bug.** Both blocks test
  against `orbit_log_last_ms` (src/main.cpp); with both flags enabled, the
  orbit-log block (listed first) always updates the shared timestamp, so the
  dbg-log never fires. Worked around during this investigation by running
  them separately; not fixed.
- The e2e orbit parser (`ORBIT_RE`, e2e/run.py) is strict about field
  adjacency in the `--orbit-log` line — the log format is a test contract.
  (The temporary `argp`/`nu` fields added for this investigation would have
  broken it; they were reverted.)

## 9. Key code references

| concern | location |
|---|---|
| fixed dt, accumulator, sim clock | src/main.cpp main loop (`dt = 1/50`; `time += dt * time_accel`) |
| physics substepping | src/main.cpp (`kMaxSubStep = 0.1`, n >= 3) |
| Bullet world step | src/physics.cpp (`stepSimulation(timeStep, 1, timeStep)`) |
| semi-implicit Euler (the culprit) | middleware/bullet3: `btDiscreteDynamicsWorld::internalSingleStepSimulation`, `btRigidBody::integrateVelocity` / `integrateTransforms` |
| orbit fit + countdowns (+ e<1e-9 guard) | src/orbit.h `computeOrbitElements` |
| HUD ApT/PeT | src/main.cpp (~line 4955) |
| rails machinery | src/main.cpp `goOnRails` / `leaveRails` / `canRail` / `railsTick` / `writeRailPose` / `railsSwitchFrames` |
| rails tick at any warp | src/main.cpp physics branch (`if(s->onRails) s->railsTick(...)`) |
| fictitious forces | src/frame.h `GetStasisVelocity` / `GetFictitiousAccel` |
| Python integrator repro | tmp/symplectic_orbit.py |
| measurement logs | tmp/measure_*.log (1x, inertial, single, ell) |
| single-part test ship | tmp/ship_single.json |
