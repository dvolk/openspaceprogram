# Implicit-midpoint fix for the rotating-frame orbital drift

**Status:** working prototype, **uncommitted**, **not safe for normal play**
(the ship cannot thrust or collide — see §6). Kept for future reference as the
concrete, validated fix for the drift characterized in `stability_report.md`.

Date: 2026-08-20. Files touched: `src/main.cpp` only. Toggle via
`OSP_INTEGRATOR=euler` to restore the legacy behaviour.

---

## 1. Problem (one paragraph)

When the ship is integrated in a **rotating** frame, the Coriolis term
(`-2 ω×v`) is a magnetic/Lorentz force. The game's semi-implicit Euler
(update `v ← v + h·(-2ω×v) = (I − 2h[ω])·v`) is **non-orthogonal**
(`|eigenvalues| = √(1 + 4h²|ω|²) ≠ 1`), so each step injects O(h²) energy →
an **O(h) secular drift**. Measured: ~0.6 J/kg/s at `h = 0.1 s` for a 75 km
Eerbon orbit (~2 %/sim-day of semi-major-axis growth). Pure Kepler (inertial
frame) with the same integrator is clean. Full analysis: `stability_report.md`.

## 2. The fix (one paragraph)

Replace the ship's free-flight COM substep — "apply force + Bullet
symplectic-Euler" — with the **implicit midpoint** method, which *is*
symplectic for this magnetic-Hamiltonian system, so the drift vanishes. The
implicit solve is done by fixed-point iteration (the contraction is
`~h·2|ω| ≪ 1` at the substep sizes used, so 8 iterations converge).

## 3. Code

### 3a. New method on `Vehicle` (after `processGravity()`, `src/main.cpp`)

```cpp
    // Implicit-midpoint (symplectic) integrator for the ship's center-of-mass
    // orbital motion, in the CURRENT frame's coordinates.  Replaces the
    // "apply force + Bullet symplectic-Euler" substep for free flight.
    //
    // Why: the Coriolis term (-2 w x v) is a magnetic / Lorentz force, so the
    // rotating-frame equations form a magnetic-Hamiltonian system.  Semi-
    // implicit Euler is NOT symplectic for that term: the update
    //   v <- v + h(-2 w x v)  =  (I - 2h[w]) v
    // is non-orthogonal (|eigenvalues| = sqrt(1 + 4 h^2 |w|^2) != 1), so each
    // step injects O(h^2) energy -> an O(h) SECULAR drift (measured ~0.6 J/kg/s
    // at h=0.1s for a 75 km Eerbon orbit; see reports/stability2026_08_20/).
    // The implicit midpoint IS symplectic for this system, so the drift vanishes.
    // Solved by fixed-point iteration (the contraction is ~h*2|w| << 1 at the
    // substep sizes used, so a handful of iterations converges).
    void integrateOrbitMidpoint(double h, int iters = 8) {
        const glm::dvec3 p0 = get_center_of_mass();
        const glm::dvec3 v0 = GetVel();
        const double mu = m_parent->mu;
        const bool rotating = frame->isRotFrame();
        auto accel = [&](const glm::dvec3 &p, const glm::dvec3 &v) -> glm::dvec3 {
            const double r = glm::length(p);
            glm::dvec3 a = -mu * p / (r * r * r);
            if (rotating) { a += frame->GetFictitiousAccel(p, v); }
            return a;
        };
        glm::dvec3 p_next = p0 + h * v0;
        glm::dvec3 v_next = v0;
        for (int it = 0; it < iters; it++) {
            const glm::dvec3 pm = 0.5 * (p0 + p_next);
            const glm::dvec3 vm = 0.5 * (v0 + v_next);
            v_next = v0 + h * accel(pm, vm);
            p_next = p0 + h * vm;
        }
        // Write the new COM back onto every part, preserving each part's
        // offset and orientation (same convention as the spawn code).
        void setPosRot(Body *b, glm::dvec3 pos, glm::dmat3 rot);
        glm::dmat3 GetOrient(Body *b);
        const glm::dvec3 com = get_center_of_mass();
        for (auto &&part : parts) {
            setPosRot(part, p_next + (GetPosition(part) - com), GetOrient(part));
            SetVelocity(part, v_next);
        }
    }
```

### 3b. Substep loop (replaces the `applyGravity + physics_tick` per substep)

```cpp
                const double step = dt * time_accel;
                const double kMaxSubStep = 0.1;
                int n = 3;
                int need = (int)(step / kMaxSubStep + 0.5);
                if (need > n) { n = need; }
                if (n > 2000) { n = 2000; }
                const double h = step / n;
                // Ship free-flight integrator. Default: implicit midpoint
                // (symplectic for the Coriolis term -> no secular drift).
                // Set OSP_INTEGRATOR=euler to use the legacy "apply force +
                // Bullet symplectic-Euler" path (kept for baseline/perf runs).
                static const bool use_midpoint = [] {
                    const char *e = getenv("OSP_INTEGRATOR");
                    return !(e && e[0] && strcmp(e, "euler") == 0);
                }();
                for (int i = 0; i < n; i++) {
                    if (use_midpoint) {
                        ship->integrateOrbitMidpoint(h);
                    } else {
                        grav = ship->processGravity();
                        physics_tick(h);
                    }
                }
```

Also added `#include <cstdlib>` and `#include <cstring>` near the top
(for `getenv` / `strcmp`).

## 4. Verification (in the real game)

Start 2 (75 km circular Eerbon orbit), `--time-accel 10000` (`h = 0.1 s`),
~16 s wall:

| integrator | ΔE (J/kg) | rate (J/kg/s) | radius (km) | ecc (max) |
|---|---|---|---|---|
| legacy Euler | 9.3×10⁴ | 0.607 | 675 → 700 (drifts) | 0.00054 |
| **implicit midpoint** | **1×10⁻⁶** | ~10⁻¹⁴ | **675.0 (locked)** | **0.00000** |

Drift removed by ~10 orders of magnitude; the orbit stays exactly the intended
circle. (Matches the standalone replica in `stability/fixtest.py`.)

## 5. Perf (sim-speed = achieved × of real-time; target `ta = 10⁴`)

| config | sim-speed | note |
|---|---|---|
| Euler, rotating frame | 9950× | baseline |
| **midpoint, rotating frame** | **9762×** | ~2 % overhead (8 fixed-point iters) |
| midpoint, non-rotating frame | 9950× | no Coriolis term → same as baseline |

The fix is effectively free performance-wise (≈2 %, within run-to-run noise).

## 6. Known limitation (why it's not merged)

`integrateOrbitMidpoint` **owns the ship's free-flight COM integration and
bypasses Bullet's `stepSimulation` for the ship.** Consequences:

- **Thrust is not applied** (engines are a no-op) — the ship can't move.
- **Collision response is not applied** — the ship can't land / collide.
- Attitude (angular) integration is also skipped (irrelevant in orbit).

So this is a correct **orbital-dynamics** fix but **not** a drop-in for
gameplay. To make it production-ready, one of:
- wire thrust + collision + attitude back into the midpoint path (keep Bullet
  for contacts, own the free-flight translation), **or**
- use the alternative from `stability_report.md` §7: integrate in the
  **non-rotating (inertial) frame** (no Coriolis term; Bullet's existing
  symplectic Euler is then clean and thrust/collision keep working).

## 7. How to run / revert

```bash
cd /home/ubuntu/openspaceprogram
make
# fixed (drift-free, but no thrust/collision):
xvfb-run -a ./osp --start 2 --time-accel 10000 --orbit-log --orbit-interval 0.004
# legacy behaviour (drift returns, thrust/collision work):
xvfb-run -a env OSP_INTEGRATOR=euler ./osp --start 2 --time-accel 10000 --orbit-log --orbit-interval 0.004
```

Reproduce the A/B + perf numbers: `python3 stability/verify_fix.py`
(uses the `/tmp/osp_*.log` outputs).
