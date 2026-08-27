// test_transfer: Lambert solver + planTransfer (src/transfer.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++11 -I./src -I./middleware/glm/ tests/test_transfer.cpp -o test_transfer && ./test_transfer)
//
// Pins the Hohmann transfer as the analytic reference case.
#include "transfer.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

#define CHECK_NEAR(a, b, tol) do { \
        double _a = (a), _b = (b), _t = (tol); \
        if(!(std::fabs(_a - _b) <= _t)) { \
            printf("FAIL %s:%d: %s = %.12g, want %.12g +- %g\n", \
                   __FILE__, __LINE__, #a, _a, _b, _t); \
            failures++; \
        } \
    } while(0)

static const double MU = 1.0e12;   // m^3/s^2

int main() {
    // =========================================================================
    // 1. solveLambert: Hohmann transfer between two circular orbits
    //    r1 = 1e6 (departure), r2 = 2e6 (arrival), 180 deg apart.
    //    Analytic: a_t = 1.5e6, T_t = 2*pi*sqrt(a_t^3/MU), tof = T_t/2
    //    v_dep = sqrt(MU*(2/r1 - 1/a_t)), v_arr = sqrt(MU*(2/r2 - 1/a_t))
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double a_t = (r1 + r2) / 2.0;
        const double T_t = 2.0 * M_PI * std::sqrt(a_t * a_t * a_t / MU);
        const double tof = T_t / 2.0;

        const double v_dep = std::sqrt(MU * (2.0 / r1 - 1.0 / a_t));
        const double v_arr = std::sqrt(MU * (2.0 / r2 - 1.0 / a_t));

        glm::dvec3 r1v(r1, 0, 0), r2v(-r2, 0, 0);
        glm::dvec3 v1, v2;
        bool ok = solveLambert(r1v, r2v, MU, tof, v1, v2);
        CHECK(ok);
        if(ok) {
            // v1 is along +Y (prograde at departure)
            CHECK_NEAR(v1.x, 0.0, 1e-6 * v_dep);
            CHECK_NEAR(v1.y, v_dep, 1e-6 * v_dep);
            CHECK_NEAR(v1.z, 0.0, 1e-6 * v_dep);
            // v2 is along -Y (prograde at arrival, 180 deg around)
            CHECK_NEAR(v2.x, 0.0, 1e-6 * v_arr);
            CHECK_NEAR(v2.y, -v_arr, 1e-6 * v_arr);
            CHECK_NEAR(v2.z, 0.0, 1e-6 * v_arr);
        }
    }

    // =========================================================================
    // 2. solveLambert round-trip: propagate v1 for tof, must land on r2
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double a_t = (r1 + r2) / 2.0;
        const double T_t = 2.0 * M_PI * std::sqrt(a_t * a_t * a_t / MU);
        const double tof = T_t / 2.0;
        const double v_arr = std::sqrt(MU * (2.0 / r2 - 1.0 / a_t));

        glm::dvec3 r1v(r1, 0, 0), r2v(-r2, 0, 0);
        glm::dvec3 v1, v2;
        CHECK(solveLambert(r1v, r2v, MU, tof, v1, v2));

        // Propagate v1 from r1 for tof seconds
        glm::dvec3 p, v;
        propagateKepler(r1v, v1, MU, tof, p, v);
        CHECK_NEAR(glm::length(p - r2v), 0.0, 1e-4 * r2);
        CHECK_NEAR(glm::length(v - v2), 0.0, 1e-4 * v_arr);
    }

    // =========================================================================
    // 3. planTransfer: Hohmann transfer to a pre-positioned moving target.
    //    The target is on a circular orbit of radius r2; we place it so that
    //    at the Hohmann time-of-flight it is exactly at the anti-point, making
    //    the Hohmann transfer the optimal solution. Pins dv_dep = Hohmann dv1.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        const double a_t = (r1 + r2) / 2.0;
        const double dv1_hoh = std::sqrt(MU * (2.0 / r1 - 1.0 / a_t)) - v1_circ;

        // Hohmann time of flight
        const double T_t = 2.0 * M_PI * std::sqrt(a_t * a_t * a_t / MU);
        const double tof_hoh = T_t / 2.0;

        // Target angular rate on its circular orbit
        const double w2 = std::sqrt(MU / (r2 * r2 * r2));
        // Angle the target moves during the Hohmann ToF
        const double dtheta = w2 * tof_hoh;
        // We want the target at angle pi (anti-point) at t = tof_hoh,
        // so at t = 0 it is at angle pi - dtheta.
        const double theta0 = M_PI - dtheta;

        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(r2 * std::cos(theta0), r2 * std::sin(theta0), 0.0);
        // prograde (counter-clockwise) velocity on the circular orbit
        glm::dvec3 tgt_vel(-v2_circ * std::sin(theta0), v2_circ * std::cos(theta0), 0.0);

        TransferSolution sol = planTransfer(
            ship_pos, ship_vel, tgt_pos, tgt_vel,
            MU, 0.0, 0.0,   // no capture
            1000.0, 20000.0, 500);

        CHECK(sol.valid);
        // best departure dv within 5% of Hohmann dv1
        CHECK_NEAR(sol.dv_departure, dv1_hoh, 0.05 * dv1_hoh);
        // no capture burn
        CHECK_NEAR(sol.dv_capture, 0.0, 1e-12);
        // ToF should be near the Hohmann ToF
        CHECK_NEAR(sol.tof, tof_hoh, 0.05 * tof_hoh);
    }

    // =========================================================================
    // 4. planTransfer: round-trip — propagating v_departure for sol.tof must
    //    land on the target's position at that time (the solver's core contract).
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(-r2, 0, 0), tgt_vel(0, -v2_circ, 0);

        TransferSolution sol = planTransfer(
            ship_pos, ship_vel, tgt_pos, tgt_vel,
            MU, 0.0, 0.0,
            1000.0, 20000.0, 500);

        CHECK(sol.valid);
        // Propagate the departure velocity for the solution ToF
        glm::dvec3 p, v;
        propagateKepler(ship_pos, sol.v_departure, MU, sol.tof, p, v);
        // Target position at that time
        glm::dvec3 tgt_pos_t, tgt_vel_t;
        propagateKepler(tgt_pos, tgt_vel, MU, sol.tof, tgt_pos_t, tgt_vel_t);
        // Must land on the target
        CHECK_NEAR(glm::length(p - tgt_pos_t), 0.0, 1e-3 * r2);
        // v_arrival must match the propagated transfer velocity
        CHECK_NEAR(glm::length(v - sol.v_arrival), 0.0, 1e-3 * v2_circ);
    }

    // =========================================================================
    // 5. planTransfer: with capture burn, total dv = dv_dep + dv_cap
    //    where dv_cap uses the v_inf from the solution.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        const double mu_t = 1.0e11;
        const double r_cap = 5.0e5;

        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(-r2, 0, 0), tgt_vel(0, -v2_circ, 0);

        TransferSolution sol = planTransfer(
            ship_pos, ship_vel, tgt_pos, tgt_vel,
            MU, mu_t, r_cap,
            1000.0, 20000.0, 500);

        CHECK(sol.valid);
        // dv_cap must be consistent with the solution's v_inf
        const double v_circ_cap = std::sqrt(mu_t / r_cap);
        const double dv_cap_expected = v_circ_cap *
            (std::sqrt(2.0 + sol.v_inf * sol.v_inf * r_cap / mu_t) - 1.0);
        CHECK_NEAR(sol.dv_capture, dv_cap_expected, 1e-6 * dv_cap_expected);
        // total = dep + cap
        CHECK_NEAR(sol.total_dv, sol.dv_departure + sol.dv_capture, 1e-9);
        // capture orbit period
        const double T_cap = 2.0 * M_PI * std::sqrt(r_cap * r_cap * r_cap / mu_t);
        CHECK_NEAR(sol.capture_orbit_period, T_cap, 1e-9 * T_cap);
    }

    // =========================================================================
    // 6. solveLambert: short transfer (not 180 deg) — sanity check
    //    r1 = (1e6, 0, 0), r2 = (0, 2e6, 0) — 90 deg apart
    //    Just verify convergence and that the round-trip works.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        glm::dvec3 r1v(r1, 0, 0), r2v(0, r2, 0);
        // pick a tof that's reasonable: ~1/4 of the circular period at r1
        const double T1 = 2.0 * M_PI * std::sqrt(r1 * r1 * r1 / MU);
        const double tof = T1 / 4.0;

        glm::dvec3 v1, v2;
        bool ok = solveLambert(r1v, r2v, MU, tof, v1, v2);
        CHECK(ok);
        if(ok) {
            // round-trip: propagate v1 for tof, must land on r2
            glm::dvec3 p, v;
            propagateKepler(r1v, v1, MU, tof, p, v);
            CHECK_NEAR(glm::length(p - r2v), 0.0, 1e-4 * r2);
            // all components finite
            CHECK(std::isfinite(v1.x) && std::isfinite(v1.y) && std::isfinite(v1.z));
            CHECK(std::isfinite(v2.x) && std::isfinite(v2.y) && std::isfinite(v2.z));
        }
    }

    // =========================================================================
    // 7. solveLambert: hyperbolic transfer (90 deg, short tof -> fast leg).
    //    Pins: velocity exceeds circular, energy is positive (a < 0), and the
    //    round-trip lands on r2.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 3.0e6;
        glm::dvec3 r1v(r1, 0, 0), r2v(0, r2, 0);
        const double T1 = 2.0 * M_PI * std::sqrt(r1 * r1 * r1 / MU);
        const double tof = T1 / 20.0;   // much faster than circular

        glm::dvec3 v1, v2;
        bool ok = solveLambert(r1v, r2v, MU, tof, v1, v2);
        CHECK(ok);
        if(ok) {
            const double v_circ = std::sqrt(MU / r1);
            CHECK(glm::length(v1) > 1.5 * v_circ);
            // energy positive -> hyperbolic
            const double energy = 0.5 * glm::dot(v1, v1) - MU / r1;
            CHECK(energy > 0.0);
            // round-trip still works
            glm::dvec3 p, v;
            propagateKepler(r1v, v1, MU, tof, p, v);
            CHECK_NEAR(glm::length(p - r2v), 0.0, 1e-3 * r2);
        }
    }

    // =========================================================================
    // 8. solveLambert: 180 deg at a non-Hohmann ToF has no solution
    //    (the conic -- and hence the ToF -- is unique for that geometry).
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        glm::dvec3 r1v(r1, 0, 0), r2v(-r2, 0, 0);
        const double T1 = 2.0 * M_PI * std::sqrt(r1 * r1 * r1 / MU);
        glm::dvec3 v1, v2;
        CHECK(!solveLambert(r1v, r2v, MU, T1 / 20.0, v1, v2));
    }

    if(failures == 0) {
        printf("test_transfer: all checks passed\n");
        return 0;
    }
    printf("test_transfer: %d FAILURES\n", failures);
    return 1;
}
