// test_porkchop: porkchopGrid (src/transfer.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++11 -I./src -I./middleware/glm/ tests/test_porkchop.cpp -o test_porkchop && ./test_porkchop)
//
// Pins the 2-D porkchop sweep against (a) planTransfer itself (at
// t_dep = 0 the two are the same computation cell for cell) and (b) the
// analytic Hohmann reference. Also exercises the no-solution (all-NaN)
// path and the grid bookkeeping.
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
    // 1. Cross-check with planTransfer: with n_dep = 1 (t_dep = 0) every
    //    porkchop cell is bit-for-bit one planTransfer sample (ship at
    //    (r1, v1) now, target propagated by tof). Same ToF range + resolution
    //    => the minima must agree to solver tolerance.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(-r2, 0, 0), tgt_vel(0, -v2_circ, 0);

        const double tof_lo = 1000.0, tof_hi = 20000.0, n = 500;
        TransferSolution pt = planTransfer(ship_pos, ship_vel, tgt_pos, tgt_vel,
                                           MU, 0.0, 0.0, tof_lo, tof_hi, n);
        PorkchopResult pc = porkchopGrid(ship_pos, ship_vel, tgt_pos, tgt_vel,
                                         MU, 0.0, 0.0, 0.0, 0.0, tof_lo, tof_hi,
                                         1, n);
        CHECK(pt.valid);
        CHECK(pc.valid);
        CHECK_NEAR(pc.dv_min, pt.total_dv, 1e-6 * pt.total_dv);
        CHECK_NEAR(pc.tof_min, pt.tof, 1e-6 * pt.tof);
        // The min sits at the t_dep = 0 column (the only column).
        CHECK(pc.i_min == 0);
    }

    // =========================================================================
    // 2. Hohmann reference: phase the target so that at (t_dep = 0,
    //    tof = T_hoh) it is at the anti-point, making that cell the
    //    analytic Hohmann transfer. Sweeping both axes must recover the
    //    Hohmann dv (departure burn only) up to grid discretization.
    //    dv1_hoh is the global min for a circular->circular transfer, so
    //    porkchop min >= dv1_hoh and close to it.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        const double a_t = (r1 + r2) / 2.0;
        const double T_t = 2.0 * M_PI * std::sqrt(a_t * a_t * a_t / MU);
        const double T_hoh = T_t / 2.0;
        const double w2 = std::sqrt(MU / (r2 * r2 * r2));
        const double T2 = 2.0 * M_PI / w2;   // target period

        // Target initial phase: at t = T_hoh it must be at angle pi.
        const double theta0 = M_PI - w2 * T_hoh;
        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(r2 * std::cos(theta0), r2 * std::sin(theta0), 0.0);
        glm::dvec3 tgt_vel(-v2_circ * std::sin(theta0), v2_circ * std::cos(theta0), 0.0);

        const double dv1_hoh = std::sqrt(MU * (2.0 / r1 - 1.0 / a_t)) - v1_circ;

        PorkchopResult pc = porkchopGrid(ship_pos, ship_vel, tgt_pos, tgt_vel,
                                         MU, 0.0, 0.0,
                                         0.0, T2, 0.5 * T_hoh, 1.5 * T_hoh,
                                         40, 100);
        CHECK(pc.valid);
        CHECK(pc.dv_min >= dv1_hoh * (1.0 - 1e-9));   // >= the global min
        CHECK(pc.dv_min <= dv1_hoh * 1.2);            // within grid error
        // The optimum is near the Hohmann point (t_dep = 0, tof = T_hoh).
        CHECK(pc.t_dep_min <= 0.5 * T2);
        CHECK(std::fabs(pc.tof_min - T_hoh) <= 0.2 * T_hoh);
    }

    // =========================================================================
    // 3. Grid bookkeeping: sizes, argmin in range, dv_min == the stored cell,
    //    and the min coordinates inside the requested windows.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 2.0e6;
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        const double w2 = std::sqrt(MU / (r2 * r2 * r2));
        const double T2 = 2.0 * M_PI / w2;
        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(-r2, 0, 0), tgt_vel(0, -v2_circ, 0);

        const int n_dep = 10, n_tof = 20;
        const double t_dep_lo = 0.0, t_dep_hi = T2;
        const double tof_lo = 1000.0, tof_hi = 20000.0;
        PorkchopResult pc = porkchopGrid(ship_pos, ship_vel, tgt_pos, tgt_vel,
                                         MU, 0.0, 0.0, t_dep_lo, t_dep_hi,
                                         tof_lo, tof_hi, n_dep, n_tof);
        CHECK(pc.n_dep == n_dep);
        CHECK(pc.n_tof == n_tof);
        CHECK((int)pc.total_dv.size() == n_tof * n_dep);
        CHECK(pc.valid);
        CHECK(pc.i_min >= 0 && pc.i_min < n_dep);
        CHECK(pc.j_min >= 0 && pc.j_min < n_tof);
        // dv_min is exactly the stored cell at the argmin (same double).
        CHECK(pc.total_dv[(size_t)pc.j_min * pc.n_dep + pc.i_min] == pc.dv_min);
        CHECK(pc.t_dep_min >= t_dep_lo && pc.t_dep_min <= t_dep_hi);
        CHECK(pc.tof_min >= tof_lo && pc.tof_min <= tof_hi);
        // dv_min is the min over the valid (non-NaN) cells.
        double m = 1e300;
        for(double v : pc.total_dv) { if(std::isfinite(v) && v < m) { m = v; } }
        CHECK_NEAR(pc.dv_min, m, 1e-12);
    }

    // =========================================================================
    // 4. No-solution path: a ~180 deg geometry (ship +X, target -X, target
    //    slow enough to stay in the Lambert 180-deg special branch during the
    //    short ToF window) with a ToF range far from the unique 180-deg ToF.
    //    Every cell fails => valid = false, all cells NaN, argmin unset.
    // =========================================================================
    {
        const double r1 = 1.0e6, r2 = 7.37e8;   // r2 large => w2 tiny
        const double v1_circ = std::sqrt(MU / r1);
        const double v2_circ = std::sqrt(MU / r2);
        glm::dvec3 ship_pos(r1, 0, 0), ship_vel(0, v1_circ, 0);
        glm::dvec3 tgt_pos(-r2, 0, 0), tgt_vel(0, -v2_circ, 0);

        // The unique 180-deg ToF is ~2.2e7 s; [1, 10] s is far from it, and
        // the target barely moves (w2 ~ 5e-8 rad/s) so the geometry stays
        // inside the special branch for the whole window.
        PorkchopResult pc = porkchopGrid(ship_pos, ship_vel, tgt_pos, tgt_vel,
                                         MU, 0.0, 0.0, 0.0, 0.0, 1.0, 10.0,
                                         1, 10);
        CHECK(!pc.valid);
        CHECK(pc.i_min == -1 && pc.j_min == -1);
        CHECK((int)pc.total_dv.size() == 10);
        for(double v : pc.total_dv) { CHECK(std::isnan(v)); }
    }

    if(failures == 0) {
        printf("test_porkchop: all checks passed\n");
        return 0;
    }
    printf("test_porkchop: %d FAILURES\n", failures);
    return 1;
}
