#pragma once
// Two-point boundary-value transfers (Lambert) + a min-dv parent->child planner.
// Header-only pure math. The transfer conic is under the PARENT's mu only;
// both endpoint states are in the SAME inertial frame. dt > 0. SI units,
// radians.

#include "orbit.h"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <limits>
#include <vector>

struct TransferSolution {
    double dv_departure = 0.0;   // m/s, burn at the ship's position
    double dv_capture = 0.0;     // m/s, circularize at r_cap around target
    double total_dv = 0.0;       // m/s
    double tof = 0.0;            // s, time of flight of the best transfer
    double r_cap = 0.0;          // m, capture periapsis used (0 = none): a
                                 // RADIUS from the target's centre, i.e. its
                                 // seaLevelRadius() + shellEdge() (#405)
    double v_inf = 0.0;          // m/s, hyperbolic excess at the target
    double capture_orbit_period = -1.0; // s, period of the r_cap circle
    double transfer_semi_major = 0.0;   // m, negative for a hyperbolic leg
    double transfer_ecc = 0.0;
    glm::dvec3 v_departure;      // transfer-conic velocity at r1 (parent frame)
    glm::dvec3 v_arrival;        // transfer-conic velocity at r2 (parent frame)
    glm::dvec3 v_target_at_arrival; // target's own velocity at t + tof
    bool retrograde = false;       // the leg's angular momentum fights the ship's
    int revolutions = 0;           // full revolutions swept by the chosen leg
    bool valid = false;
};

// Stub: semi-major axis needs a time of flight, not just geometry. Use
// specific orbital energy instead.
inline double transfer_semi_major_of(const glm::dvec3 &, const glm::dvec3 &, double, double) {
    return 0.0;
}

// One Lambert conic through r1 -> r2 in `tof`: the velocity pair at the two
// endpoints. The boundary value problem is under-determined twice over:
//  * the same (r1, r2, tof) is reached by an arc that turns less than 180 deg
//    and one that turns more. They wind opposite ways round the focus -- one
//    prograde, one retrograde relative to the ship's own orbit -- with similar
//    but not equal energies. Only the prograde one is cheap for a ship that
//    did not arrive retrograde, and pricing only the other costs an order of
//    magnitude more (issue #197).
//  * it is also reached with `rev` full revolutions swept (issue #136), and for
//    a co-orbital or long-ToF target that is the ONLY cheap answer.
// Anything that prices a transfer has to look at all of them: solveLambertLegs.
struct LambertLeg {
    glm::dvec3 v1, v2;
    bool a_neg = false;    // which sign of A produced this leg
    int rev = 0;           // full revolutions swept; 0 = the direct family
};

// stumpffC(z) vanishes at z = 4 pi^2 k^2, and those zeros are the walls
// between revolution families: band N is (4 pi^2 N^2, 4 pi^2 (N+1)^2). Band 0
// is the direct family and is always scanned; kLambertMaxRev says how many
// OUTER bands to scan too. N=1 is where a co-orbital's cheap phasing leg sits
// and costs about +30% on a grid sweep; each further band buys rapidly less
// (#136, #252).
inline constexpr int kLambertMaxRev = 1;

// Band 0 yields at most one leg per sign of A (F is monotone there); an outer
// band can hold two, the pair that merges at the band's minimum-energy edge.
inline constexpr int kLambertLegMax = 4 * (kLambertMaxRev + 1);

// Fractional offsets from a band wall where the sample grid gets an extra
// probe. F blows up at the walls and the CHEAP leg of an outer band sits hard
// against one of them -- a phasing leg's root was 1.4% into band 1, inside the
// dead zone of a uniform 48-sample step, and quoting the next root out costs
// four orders of magnitude. Nothing closer than 1e-3: that is where F stops
// being a number at all, and a root that close to the wall is a conic whose
// dv no cost function would pick.
inline constexpr double kWallProbeFrac[] = {1e-3, 3e-3, 1e-2, 3e-2};

/* Every leg of one revolution band for one sign of A, appended to legs[*n]
   up to cap. A = sin(dnu) * sqrt(r1 r2 / (1 - cos dnu)), and the geometry only
   fixes cos dnu, so the sign of A is a free choice: A > 0 is the arc that
   turns less than 180 deg from r1 to r2, A < 0 the reflex arc that turns more.
   Universal variable formulation (Bate, Mueller & White). */
inline void lambertBand(const glm::dvec3 &r1, const glm::dvec3 &r2,
                        double mu, double tof, double A,
                        double z_lo, double z_hi, int samples,
                        int rev, LambertLeg *legs, int *n, int cap) {
    const double r1l = glm::length(r1);
    const double r2l = glm::length(r2);

    auto F = [&](double z) -> double {
        const double C = stumpffC(z);
        const double S = stumpffS(z);
        // C vanishes at the band walls and y < 0 is the radial limit (that
        // chord cannot be reached). Both mean "no conic here" and answer NaN,
        // which no comparison can mistake for a sign.
        if(!(std::fabs(C) > 1e-30)) return std::numeric_limits<double>::quiet_NaN();
        const double y = r1l + r2l + A * (z * S - 1.0) / std::sqrt(C);
        if(y < 0.0) return std::numeric_limits<double>::quiet_NaN();
        return std::pow(y / C, 1.5) * S + A * std::sqrt(y) - std::sqrt(mu) * tof;
    };
    // A magnitude threshold here would be a size limit on the star system: F
    // carries units of L^1.5, so "huge" means something different at every
    // radius. Only NaN is out of play; +inf at a wall is real mathematics and
    // brackets legitimately.
    auto usable = [](double fz) { return !std::isnan(fz); };

    const double target = std::sqrt(mu) * std::fabs(tof);
    const double band = z_hi - z_lo;
    double z_prev = 0.0, f_prev = 0.0;
    bool have_prev = false;

    auto bisect = [&](double lo, double hi, double f_lo, double *zroot) -> bool {
        double zz = 0.5 * (lo + hi);
        for(int iter = 0; iter < 100; iter++) {
            zz = 0.5 * (lo + hi);
            const double fzz = F(zz);
            // A domain edge INSIDE the bracket: the sign change is the edge,
            // not a conic. Give this bracket up rather than converge onto it.
            if(!usable(fzz)) { return false; }
            if(std::fabs(fzz) < 1e-10 * (target + 1.0)) { break; }
            if((fzz < 0.0) != (f_lo < 0.0)) { hi = zz; } else { lo = zz; f_lo = fzz; }
            if((hi - lo) < 1e-12 * (1.0 + std::fabs(zz))) { break; }
        }
        *zroot = zz;
        return true;
    };

    // One sample: bracket against the previous usable one. Returns false once
    // the cap is full and the caller must stop walking.
    auto visit = [&](double z) -> bool {
        const double fz = F(z);
        if(!usable(fz)) { have_prev = false; return *n < cap; }
        double zz = 0.0;
        if(have_prev && (f_prev < 0.0) != (fz < 0.0) && bisect(z_prev, z, f_prev, &zz)) {
            const double C = stumpffC(zz);
            const double S = stumpffS(zz);
            const double y = r1l + r2l + A * (zz * S - 1.0) / std::sqrt(C);
            const double f = 1.0 - y / r1l;
            const double g_dot = 1.0 - y / r2l;
            const double g = A * std::sqrt(y / mu);
            if(std::fabs(g) > 1e-20) {
                legs[*n].v1 = (r2 - f * r1) / g;
                legs[*n].v2 = (g_dot * r2 - r1) / g;
                legs[*n].rev = rev;
                (*n)++;
            }
        }
        z_prev = z; f_prev = fz;
        have_prev = true;
        return *n < cap;
    };

    for(int i = 0; i <= samples; i++) {
        const double za = z_lo + band * i / (double)samples;
        const double zb = z_lo + band * (i + 1) / (double)samples;
        if(!visit(za)) { return; }
        // Emit the wall probes that fall in this step, in ascending z.
        for(int p = 0; p < (int)(sizeof(kWallProbeFrac) / sizeof(*kWallProbeFrac)); p++) {
            const double zl = z_lo + band * kWallProbeFrac[p];
            if(zl > za && zl < zb && !visit(zl)) { return; }
        }
        for(int p = (int)(sizeof(kWallProbeFrac) / sizeof(*kWallProbeFrac)) - 1; p >= 0; p--) {
            const double zh = z_hi - band * kWallProbeFrac[p];
            if(zh > za && zh < zb && !visit(zh)) { return; }
        }
    }
}

/* Every conic under mu that carries r1 to r2 in `tof` seconds, written into
   legs[0..kLambertLegMax): the 0-revolution family first (minor arc, then
   reflex arc), then the outer revolution bands in the same order. Returns how
   many legs converged. Callers that price a transfer loop them all and keep
   the cheapest by their own cost; callers that just want the classic arc take
   legs[0]. */
inline int solveLambertLegs(const glm::dvec3 &r1, const glm::dvec3 &r2_in,
                            double mu, double tof, LambertLeg *legs) {
    const double r1l = glm::length(r1);
    const double r2l = glm::length(r2_in);
    if(r1l < 1e-9 || r2l < 1e-9 || mu <= 0.0 || tof == 0.0) {
        return 0;
    }
    const double cos_dnu = glm::dot(r1, r2_in) / (r1l * r2l);

    // 180 deg transfer: the orbit plane is undefined (A -> 0), so it is
    // handled analytically. The conic is unique, so only half its period is
    // solvable -- and the two branches collapse into it.
    if(cos_dnu < -0.9999999) {
        const double a = 0.5 * (r1l + r2l);
        const double T = 2.0 * std::numbers::pi * std::sqrt(a * a * a / mu);
        if(std::fabs(tof - 0.5 * T) > 1e-6 * T) { return 0; }
        const double v1m = std::sqrt(mu * (2.0 / r1l - 1.0 / a));
        const double v2m = std::sqrt(mu * (2.0 / r2l - 1.0 / a));
        glm::dvec3 normal(0.0, 0.0, 1.0);
        if(std::abs(r1.z) > 0.9 * r1l) { normal = glm::dvec3(0.0, 1.0, 0.0); }
        glm::dvec3 pg1 = glm::normalize(glm::cross(normal, r1));
        glm::dvec3 pg2 = glm::normalize(glm::cross(normal, r2_in));
        legs[0].v1 = pg1 * v1m;
        legs[0].v2 = pg2 * v2m;
        legs[0].a_neg = false;
        return 1;
    }

    const double A = std::sqrt(r1l * r2l * (1.0 + cos_dnu));
    /* Scan band-major so legs[0..1] stay the 0-revolution minor/reflex pair
       whenever that family has a root: everything that reads only the first
       two legs keeps working. */
    // A band-N conic cannot fly in less than N periods of the minimum-energy
    // ellipse (a = s/2, s the fundamental triangle's semiperimeter), so a
    // short ToF never needs the outer bands. Measured: the gate loses nothing.
    const double a_min = 0.25 * (r1l + r2l + glm::length(r2_in - r1));
    const double T_min = 2.0 * std::numbers::pi * std::sqrt(a_min * a_min * a_min / mu);
    const double P = 4.0 * std::numbers::pi * std::numbers::pi;

    int nleg = 0;
    for(int rev = 0; rev <= kLambertMaxRev && nleg < kLambertLegMax; rev++) {
        if(rev > 0 && tof < rev * T_min) { continue; }
        // The walls are the zeros of stumpffC; step just inside them.
        const double z_lo = (rev == 0) ? -P : P * rev * rev + 1e-6;
        const double z_hi = P * (rev + 1) * (rev + 1) - 1e-6;
        for(int k = 0; k < 2; k++) {
            const int before = nleg;
            // Band 0 holds at most one root per sign of A (F is monotone
            // there), an outer band at most two, so stop scanning once they
            // are in hand instead of walking the rest of the band.
            const int want = nleg + ((rev == 0) ? 1 : 2);
            lambertBand(r1, r2_in, mu, tof, k ? -A : A, z_lo, z_hi,
                        (rev == 0) ? 400 : 48, rev, legs, &nleg,
                        std::min(want, kLambertLegMax));
            for(int m = before; m < nleg; m++) { legs[m].a_neg = (k == 1); }
        }
    }
    return nleg;
}

/* Lambert: ONE conic under mu from r1 to r2 in `tof` -- the arc that turns
   less than 180 deg, which is the one this solver has always returned. It is
   NOT the one to price a transfer with: whether that arc is the cheap one
   depends on the ship's own velocity, which this function never sees. Callers
   that price a transfer sweep both legs instead (planTransfer, porkchopGrid).
   Returns false if no solution converged (degenerate geometry). */
inline bool solveLambert(const glm::dvec3 &r1, const glm::dvec3 &r2_in,
                         double mu, double tof,
                         glm::dvec3 &v1, glm::dvec3 &v2) {
    LambertLeg legs[kLambertLegMax];
    const int n = solveLambertLegs(r1, r2_in, mu, tof, legs);
    for(int k = 0; k < n; k++) {
        if(legs[k].rev == 0) { v1 = legs[k].v1; v2 = legs[k].v2; return true; }
    }
    return false;
}

/* Minimum-total-dv transfer from (r1, v1) to (r2_0, v2_0) under mu_parent,
   with a capture burn at periapsis r_cap around the target (mu_target).
   Sweeps `n` linear ToF samples in [tof_min, tof_max] and keeps the cheapest
   leg at each sample (both windings, every revolution band).
   mu_target <= 0 / capture = false means intercept-only: dv_capture is 0. */
inline TransferSolution planTransfer(const glm::dvec3 &r1, const glm::dvec3 &v1,
                                     const glm::dvec3 &r2_0, const glm::dvec3 &v2_0,
                                     double mu_parent,
                                     double mu_target, double r_cap,
                                     double tof_min, double tof_max,
                                     int n, bool capture = true) {
    TransferSolution best;
    if(n < 1) { n = 1; }
    if(tof_max < tof_min) { std::swap(tof_max, tof_min); }
    const double dt = (n > 1) ? (tof_max - tof_min) / (n - 1) : 0.0;

    for(int i = 0; i < n; i++) {
        const double tof = tof_min + ((n > 1) ? dt * i : 0.0);
        glm::dvec3 r2, v2;
        propagateKepler(r2_0, v2_0, mu_parent, tof, r2, v2);
        LambertLeg legs[kLambertLegMax];
        const int nleg = solveLambertLegs(r1, r2, mu_parent, tof, legs);

        for(int k = 0; k < nleg; k++) {
            const glm::dvec3 &vt1 = legs[k].v1;
            const glm::dvec3 &vt2 = legs[k].v2;

            TransferSolution s;
            s.tof = tof;
            s.revolutions = legs[k].rev;
            s.dv_departure = glm::length(vt1 - v1);
            s.v_departure = vt1;
            s.v_arrival = vt2;
            s.v_target_at_arrival = v2;
            s.v_inf = glm::length(vt2 - v2);

            const glm::dvec3 h_leg = glm::cross(r1, vt1);
            // Every leg is checked against the ship's own angular momentum, so
            // this says which ones leave going its own way and which fight it.
            s.retrograde = glm::dot(glm::cross(r1, v1), h_leg) < 0.0;

            const double v_dep_sq = glm::dot(vt1, vt1);
            const double r1l = glm::length(r1);
            const double energy = 0.5 * v_dep_sq - mu_parent / r1l;
            if(std::fabs(energy) < 1e-12) {
                s.transfer_semi_major = 0.0; // Parabolic
            } else {
                s.transfer_semi_major = -mu_parent / (2.0 * energy);
            }

            const double h2 = glm::dot(h_leg, h_leg);
            const double p = h2 / mu_parent;
            if(s.transfer_semi_major < 0.0) {
                const double inv_a = 1.0 / s.transfer_semi_major;
                const double e2 = 1.0 - p * inv_a;
                s.transfer_ecc = (e2 > 0.0) ? std::sqrt(e2) : 0.0;
            } else if(s.transfer_semi_major > 0.0) {
                const double e2 = 1.0 - p / s.transfer_semi_major;
                s.transfer_ecc = (e2 > 0.0) ? std::sqrt(e2) : 0.0;
            } else {
                s.transfer_ecc = 1.0;
            }

            if(capture && mu_target > 0.0 && r_cap > 0.0) {
                const double vc = std::sqrt(mu_target / r_cap);
                s.dv_capture = vc * (std::sqrt(2.0 + s.v_inf * s.v_inf * r_cap / mu_target) - 1.0);
                s.r_cap = r_cap;
                s.capture_orbit_period = 2.0 * std::numbers::pi * std::sqrt(r_cap * r_cap * r_cap / mu_target);
            }
            s.total_dv = s.dv_departure + s.dv_capture;
            s.valid = true;

            if(!best.valid || s.total_dv < best.total_dv) {
                best = s;
            }
        }
    }
    return best;
}

// Porkchop plot: total dv swept over a 2-D grid of (departure delay, ToF).
// Each cell is the cheapest leg at that (t_dep, tof) over both windings and
// every revolution band.
// Storage is ToF-major for a direct ImPlot PlotHeatmap call; NaN = no solution.
struct PorkchopResult {
    int n_dep = 0;     // cols: departure-delay samples (x axis)
    int n_tof = 0;     // rows: ToF samples (y axis)
    double t_dep_lo = 0.0, t_dep_hi = 0.0;   // s, departure delay from now
    double tof_lo = 0.0, tof_hi = 0.0;       // s
    std::vector<double> total_dv;            // n_tof * n_dep, ToF-major
    int i_min = -1;      // departure index at the min (-1 = no valid cell)
    int j_min = -1;      // ToF index at the min
    double dv_min = 0.0;  // m/s, min total dv over the valid cells
    double dv_hi = 0.0;   // m/s, 95th percentile (robust max for the color scale)
    double t_dep_min = 0.0; // s, departure delay at the min
    double tof_min = 0.0;   // s, ToF at the min
    bool valid = false;    // true if at least one cell solved
};

inline PorkchopResult porkchopGrid(const glm::dvec3 &r1, const glm::dvec3 &v1,
                                   const glm::dvec3 &r2_0, const glm::dvec3 &v2_0,
                                   double mu_parent,
                                   double mu_target, double r_cap,
                                   double t_dep_lo, double t_dep_hi,
                                   double tof_lo, double tof_hi,
                                   int n_dep, int n_tof,
                                   bool capture = true) {
    PorkchopResult res;
    res.n_dep = (n_dep < 1) ? 1 : n_dep;
    res.n_tof = (n_tof < 1) ? 1 : n_tof;
    if(t_dep_hi < t_dep_lo) { std::swap(t_dep_hi, t_dep_lo); }
    if(tof_hi < tof_lo) { std::swap(tof_hi, tof_lo); }
    res.t_dep_lo = t_dep_lo; res.t_dep_hi = t_dep_hi;
    res.tof_lo = tof_lo; res.tof_hi = tof_hi;

    const double step_dep = (res.n_dep > 1) ? (t_dep_hi - t_dep_lo) / (res.n_dep - 1) : 0.0;
    const double step_tof = (res.n_tof > 1) ? (tof_hi - tof_lo) / (res.n_tof - 1) : 0.0;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    res.total_dv.assign((size_t)res.n_tof * res.n_dep, nan);

    double best = std::numeric_limits<double>::infinity();
    int bi = -1, bj = -1;

    for(int i = 0; i < res.n_dep; i++) {
        // Ship state depends only on t_dep: propagate once per row.
        const double t_dep = t_dep_lo + ((res.n_dep > 1) ? step_dep * i : 0.0);
        glm::dvec3 r1d, v1d;
        propagateKepler(r1, v1, mu_parent, t_dep, r1d, v1d);
        for(int j = 0; j < res.n_tof; j++) {
            const double tof = tof_lo + ((res.n_tof > 1) ? step_tof * j : 0.0);
            glm::dvec3 r2a, v2a;
            propagateKepler(r2_0, v2_0, mu_parent, t_dep + tof, r2a, v2a);
            LambertLeg legs[kLambertLegMax];
            const int nleg = solveLambertLegs(r1d, r2a, mu_parent, tof, legs);
            if(nleg == 0) { continue; }

            double cell = std::numeric_limits<double>::infinity();
            for(int k = 0; k < nleg; k++) {
                const double dv_dep = glm::length(legs[k].v1 - v1d);
                const double v_inf = glm::length(legs[k].v2 - v2a);
                double dv_cap = 0.0;
                if(capture && mu_target > 0.0 && r_cap > 0.0) {
                    const double vc = std::sqrt(mu_target / r_cap);
                    dv_cap = vc * (std::sqrt(2.0 + v_inf * v_inf * r_cap / mu_target) - 1.0);
                }
                cell = std::min(cell, dv_dep + dv_cap);
            }
            res.total_dv[(size_t)j * res.n_dep + i] = cell;
            if(cell < best) { best = cell; bi = i; bj = j; }
        }
    }

    if(bi >= 0) {
        res.valid = true;
        res.i_min = bi; res.j_min = bj;
        res.dv_min = best;
        res.t_dep_min = t_dep_lo + ((res.n_dep > 1) ? step_dep * bi : 0.0);
        res.tof_min = tof_lo + ((res.n_tof > 1) ? step_tof * bj : 0.0);

        // 95th percentile: the unphysical fast-transfer spike would otherwise
        // stretch the color scale into a sliver.
        std::vector<double> valid;
        valid.reserve(res.total_dv.size());
        for(const double v : res.total_dv) {
            if(!std::isnan(v)) { valid.push_back(v); }
        }
        if(!valid.empty()) {
            const size_t idx = std::min(valid.size() - 1,
                                        (size_t)(0.95 * valid.size()));
            std::nth_element(valid.begin(), valid.begin() + idx, valid.end());
            res.dv_hi = valid[idx];
        }
    }
    return res;
}
