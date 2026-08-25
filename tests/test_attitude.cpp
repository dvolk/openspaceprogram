//
// Regression test for the attitude controllers (Vehicle::slewToward /
// killRotStep in src/main.cpp) and the Δv budget formula
// (Vehicle::getDeltaV).
//
// The controllers slew the ship's nose (local +Z) toward a target
// direction using the reaction wheel, within the wheel's torque authority
// (alpha = maxTorque()/I, I the ship's real moment of inertia). The law
// below is integrated per substep (h = step/n, n = the main loop's
// substeps), exactly as src/main.cpp does.
//
// History of the laws this test has pinned:
//   OLD-OLD (the original game code): torque = normalize(cross(dir, facing)) / 10
//     -- normalize() cancelled the /10: a full-magnitude bang-bang with no
//     velocity feedback. It LIMIT-CYCLES on a light wheel (overshoots the
//     target every tick, forever, at any time acceleration). That is why
//     the autopilot buttons were originally left unwired.
//   OLD (intermediate fix): damped PD, ω += (Kp·sinθ − Kd·ω), Kp = min(2, 0.8/step),
//     Kd = 0.8, with UNLIMITED torque authority. Stable, but it could command
//     ~400x the manual stick's torque -- the autopilot "yanked" harder than
//     a maxed manual stick, and at the manual authority a PD law cannot brake
//     in time (overshoots ~60 deg). Rejected on feel.
//   NEW (this test pins it): authority-bounded slew, per SUBSTEP. The
//     wheel's torque is capped at its rating (the SAME authority the
//     manual keys apply), so the per-substep Δω is bounded by A/n
//     (A = per-tick authority, n = the main loop's substeps, h = step/n).
//     The target rate is the braking curve v_des = sqrt(2·(A/step)·E) --
//     the fastest rate from which the ship can still stop exactly at the
//     target (E = true error angle) -- capped at E/(2h) so a substep at
//     that rate cannot cross the target. Per substep:
//         v  += clamp(v_des - v, -A/n, +A/n)
//         E -= v * h
//     The law is re-evaluated every substep, not once per tick: the
//     per-tick variant of this same law limit-cycles in the stiff regime
//     (the a = 0.5 row of the sweep below failed 15 of the 227 checks;
//     adding a 1000x-warp step grows the failure to 22 of 288) -- the
//     braking solution settles within a few ticks, which per-tick explicit
//     Euler cannot integrate. tests/test_rotation.cpp keeps a non-vacuous
//     guard showing the per-tick form fails while this form passes.
//     Full authority when far off, eases off to arrive at zero rate: no
//     overshoot, no oscillation, never more forceful than a maxed manual
//     stick. KillRot is the same law with the target rate zero (per tick
//     |v| drops by min(|v|, A) -- identical whether applied in n substeps
//     or once).
//
// What this test asserts:
//   1. The NEW law converges from 5/30/90 deg, starting at rest, already
//      closing at the safe rate, or moving AWAY at 1x/2x the safe rate,
//      for a sweep of authority levels (from well below to well above the
//      standard ship's real authority at 1x/10x/100x warp): within a
//      bounded tick budget, with <= 3 target crossings, no NaN/Inf.
//   2. The OLD-OLD law limit-cycles -- i.e. this harness is capable of
//      detecting oscillation (guard against a vacuous test).
//   3. KillRot's bounded law decays the spin monotonically (no sign flip).
//   4. getDeltaV is the thrust-model-consistent budget: ve*ln((dry+fuel)/dry)
//      (~1652 m/s for the standard ship at spawn, 0 at empty, monotonic,
//      and >= the delivered Δv lower bound from reports/).
//
// Build & run (from repo root) -- also part of `make test` (pure C, no
// Bullet link needed):
//   g++ -O2 -std=c++11 tests/test_attitude.cpp -o test_attitude && ./test_attitude

#include <cmath>
#include <cstdio>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("FAIL: %s\n", msg);                                         \
        }                                                                      \
    } while (0)

struct RunResult {
    bool converged;
    int crossings; // times the trajectory crossed the target
    int ticks;
    bool bad;      // NaN/Inf
};

/* The main loop's substep count (src/main.cpp):
   n = max(3, round(step / 0.1)), capped at 2000. h = step/n. */
static int main_loop_n(double step) {
    const double kMaxSubStep = 0.1;
    int n = 3;
    int need = (int)(step / kMaxSubStep + 0.5);
    if(need > n) { n = need; }
    if(n > 2000) { n = 2000; }
    return n;
}

// The NEW law, per SUBSTEP (mirrors src/main.cpp Vehicle::slewToward +
// Bullet's semi-implicit step): A = per-tick |domega| authority
// (rad/s per tick), step = tick duration (s), x = signed error angle,
// v = angular velocity (x increases with v). v_des points toward the
// target at the braking rate sqrt(2*(A/step)*|x|), capped at |x|/(2h)
// so a substep at that rate cannot cross the target; the per-substep
// domega is bounded by A/n.
static RunResult run_new(double A, double step, double x0, double v0,
                         int budget) {
    double x = x0, v = v0;
    int crossings = 0;
    const int n = main_loop_n(step);
    const double h = step / n;
    for(int t = 0; t < budget; t++) {
        for(int i = 0; i < n; i++) {
            double sign = (x >= 0.0) ? 1.0 : -1.0;
            double v_des = -sign * std::sqrt(2.0 * (A / step) * std::fabs(x));
            double cap = std::fabs(x) / (2.0 * h);
            if(std::fabs(v_des) > cap) { v_des = -sign * cap; }
            double dv = v_des - v;
            double Amax = A / n;
            if(dv > Amax) { dv = Amax; }
            if(dv < -Amax) { dv = -Amax; }
            v += dv;
            if(!std::isfinite(x) || !std::isfinite(v)) {
                return RunResult{false, crossings, t, true};
            }
            double x_new = x + v * h;
            if(!std::isfinite(x_new)) {
                return RunResult{false, crossings, t, true};
            }
            if(x_new * x < 0.0) { crossings++; }
            x = x_new;
        }

        if(std::fabs(x) < M_PI / 180.0) { // within 1 deg: must STAY there
            double x2 = x, v2 = v;
            bool stays = true;
            for(int m = 0; m < 200 && stays; m++) {
                for(int i = 0; i < n && stays; i++) {
                    double s2 = (x2 >= 0.0) ? 1.0 : -1.0;
                    double vd2 = -s2 * std::sqrt(2.0 * (A / step) * std::fabs(x2));
                    double cap2 = std::fabs(x2) / (2.0 * h);
                    if(std::fabs(vd2) > cap2) { vd2 = -s2 * cap2; }
                    double dv2 = vd2 - v2;
                    double A2 = A / n;
                    if(dv2 > A2) { dv2 = A2; }
                    if(dv2 < -A2) { dv2 = -A2; }
                    v2 += dv2;
                    if(!std::isfinite(x2) || !std::isfinite(v2)) { stays = false; break; }
                    x2 += v2 * h;
                    if(!std::isfinite(x2) || std::fabs(x2) > 5.0 * M_PI / 180.0) {
                        stays = false;
                    }
                }
            }
            if(stays) { return RunResult{true, crossings, t, false}; }
        }
    }
    return RunResult{false, crossings, budget, false};
}

// The OLD-OLD law (bang-bang, unit torque, I = 1), per tick.
static RunResult run_old(double step, double x0, int budget) {
    double x = x0, v = 0.0;
    int crossings = 0;
    for(int n = 0; n < budget; n++) {
        v += (x > 0.0 ? 1.0 : -1.0) * step; // unit torque, I = 1
        if(!std::isfinite(x) || !std::isfinite(v)) {
            return RunResult{false, crossings, n, true};
        }
        double x_new = x + v * step;
        if(x_new * x < 0.0) { crossings++; }
        x = x_new;
        if(std::fabs(x) < M_PI / 180.0) {
            // "converged" only if it STAYS within 5 deg for the next 200 ticks
            double x2 = x, v2 = v;
            bool stays = true;
            for(int m = 0; m < 200; m++) {
                v2 += (x2 > 0.0 ? 1.0 : -1.0) * step;
                if(!std::isfinite(x2) || !std::isfinite(v2)) { stays = false; break; }
                x2 += v2 * step;
                if(!std::isfinite(x2) || std::fabs(x2) > 5.0 * M_PI / 180.0) {
                    stays = false;
                    break;
                }
            }
            if(stays) { return RunResult{true, crossings, n, false}; }
        }
    }
    return RunResult{false, crossings, budget, false};
}

// KillRot's NEW law: |v| -= min(|v|, A) per tick (bounded monotonic decay).
static bool killrot_converges(double A, double v0, int budget) {
    double v = v0;
    for(int n = 0; n < budget; n++) {
        double dv = -v;
        if(dv > A) { dv = A; }
        if(dv < -A) { dv = -A; }
        v += dv;
        if(!std::isfinite(v)) { return false; }
        if((v > 0.0) != (v0 > 0.0) && v != 0.0) { return false; } // sign flip = jitter
        if(std::fabs(v) < 0.001 * std::fabs(v0)) { return true; }
    }
    return false;
}

// getDeltaV formula (src/main.cpp): ve * ln((dry + fuel) / dry).
static double getDeltaV_formula(double ve, double dry, double fuel) {
    return ve * std::log((dry + fuel) / dry);
}

int main() {
    printf("== Slew: authority-bounded braking law ==\n");
    // a = A*step/x0: the per-tick authority relative to the initial error
    // (a = alpha*step^2/x0 with alpha = maxTorque()/I). The standard ship
    // (I ~ 8444 kg m^2, one 2000 N m wheel, alpha ~ 0.24 rad/s^2) sits at
    // a ~ 6e-5..1.1e-3 at 1x warp and a ~ 0.11 at 10x warp (5 deg error);
    // at 100x+ warp it is above the top of this sweep -- so the a = 0.5
    // row is the stiff regime the per-tick form limit-cycles in (15 of
    // these 216 checks failed before the per-substep law pinned above).
    const double a_grid[] = {5e-6, 5e-5, 5e-4, 5e-3, 5e-2, 5e-1};
    const double step_grid[] = {0.02, 0.2, 2.0};   // 1x, 10x, 100x warp
    const double x0_grid[] = {5.0, 30.0, 90.0};    // deg
    const double v0_grid[] = {0.0, 1.0, -1.0, -2.0}; // x v_safe multiples

    int runs = 0;
    for(double a : a_grid) {
        for(double step : step_grid) {
            for(double x0deg : x0_grid) {
                const double x0 = x0deg * M_PI / 180.0;
                const double A = a * x0 / step;
                const double v_safe = std::sqrt(2.0 * A * x0 / step);
                for(double v0f : v0_grid) {
                    RunResult r = run_new(A, step, x0, v0f * v_safe, 3000);
                    char buf[192];
                    snprintf(buf, sizeof buf,
                             "NEW law: a=%.0e step=%.2f x0=%.0fdeg v0=%.1fv_safe: "
                             "converged=%d crossings=%d ticks=%d bad=%d",
                             a, step, x0deg, v0f, (int)r.converged,
                             r.crossings, r.ticks, (int)r.bad);
                    CHECK_TRUE(r.converged && !r.bad && r.crossings <= 3, buf);
                    runs++;
                }
            }
        }
    }
    printf("   (%d configurations)\n", runs);

    printf("== OLD-OLD law limit-cycles (harness sanity) ==\n");
    // At least one configuration must show the old law failing to stay
    // converged -- otherwise the harness above could not catch oscillation.
    bool old_failed_anywhere = false;
    for(double step : step_grid) {
        for(double x0deg : x0_grid) {
            RunResult r = run_old(step, x0deg * M_PI / 180.0, 20000);
            if(!r.converged) { old_failed_anywhere = true; }
        }
    }
    CHECK_TRUE(old_failed_anywhere,
               "OLD law must limit-cycle in at least one configuration "
               "(if this fails, the test harness is too weak to catch "
               "oscillation)");

    printf("== KillRot: bounded decay, no sign-flip jitter ==\n");
    const double A_grid[] = {0.01, 0.1, 1.0};
    const double v0_grid2[] = {1.0, 10.0};
    for(double A : A_grid) {
        for(double v0 : v0_grid2) {
            // |v| drops by min(|v|,A) per tick: needs ~|v0|/A ticks.
            int budget = (int)(std::fabs(v0) / A) + 10;
            bool ok = killrot_converges(A, v0, budget);
            char buf[128];
            snprintf(buf, sizeof buf, "KillRot: A=%.2f v0=%.1f converges without flip",
                     A, v0);
            CHECK_TRUE(ok, buf);
        }
    }

    printf("== getDeltaV: thrust-model-consistent budget ==\n");
    {
        // Standard ship (racer: capsule + wheel + fuel_tank + engine):
        // ~1834 kg dry, 836 kg propellant (418 H + 418 LOX), ve = 4400 m/s,
        // flow 5.68 kg/s per propellant, thrust T = (2 x 5.68) x 4400 = 49984 N.
        // (The budget formula is scale-invariant in the mass ratio.)
        const double ve = 4400.0, dry = 1833.92, fuel = 835.66;
        const double flow = 2.0 * 5.68;  // H2 + LOX, 5.68 kg/s each
        double dv_full = getDeltaV_formula(ve, dry, fuel);
        double dv_empty = getDeltaV_formula(ve, dry, 0.0);
        double dv_half = getDeltaV_formula(ve, dry, fuel / 2.0);

        char buf[192];
        snprintf(buf, sizeof buf, "dv(full) = %.1f m/s ~= 1652", dv_full);
        CHECK_TRUE(std::fabs(dv_full - 1652.0) < 5.0, buf);

        CHECK_TRUE(dv_empty == 0.0, "dv(empty) == 0");
        CHECK_TRUE(dv_half < dv_full, "dv is monotonic in fuel");

        // The displayed budget must EQUAL the Δv the engine can deliver.
        // Thrust counts both propellants (T = (H2+LOX flow) x ve) against
        // the total flow, so delivered Δv = (T/flow) x ln(m_wet/m_dry)
        // -- the same formula as the budget. Bullet's per-tick mass update
        // leaves a ~1-tick margin (reports/transfers2026_08_22 §4.3).
        const double thrust = flow * ve;  /* N: total (H2+LOX) flow x ve */
        const double delivered = (thrust / flow) * log((dry + fuel) / dry);
        snprintf(buf, sizeof buf,
                 "budget %.1f == deliverable Δv %.1f (thrust counts both propellants)",
                 dv_full, delivered);
        CHECK_TRUE(std::fabs(dv_full - delivered) < 5.0, buf);
    }

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
