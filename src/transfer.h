#pragma once
// Two-point boundary-value transfers (Lambert problem) + a minimum-dv
// planner for parent->child transfers.
//
// Header-only pure math (like orbit.h) so tests/test_transfer.cpp can pin
// it without rendering/Bullet. Reuses orbit.h's Stumpff functions; the
// Newton structure mirrors propagateKepler's.
//
// Model (the standard interplanetary assumption):
//   * the transfer conic is a Kepler orbit under the PARENT's mu only;
//   * both endpoint states are expressed in the SAME inertial frame --
//     for parent->child that is the parent's inertial frame, the ship's
//     current frame (ship in the parent SOI, target a direct child);
//   * departure burn = |v_transfer(r1) - v_ship| at the ship's position;
//   * the arrival is treated as a hyperbolic encounter with the target:
//     v_inf = |v_transfer(r2) - v_target(r2)|, and the capture burn
//     circularizes at periapsis r_cap of the target:
//       dv_cap = v_circ * (sqrt(2 + v_inf^2 * r_cap / mu_t) - 1),
//     which reduces to (sqrt(2)-1) * v_circ for a parabolic arrival
//     (v_inf = 0) and -> v_inf for large v_inf.
//
// Conventions: SI units, radians. dt > 0. The transfer may be elliptic
// or hyperbolic (the solver handles both through the universal variable).

#include "orbit.h"

#include <cmath>
#include <algorithm>

struct TransferSolution {
    double dv_departure = 0.0;   // m/s, burn at the ship's position
    double dv_capture = 0.0;     // m/s, circularize at r_cap around target
    double total_dv = 0.0;       // m/s
    double tof = 0.0;            // s, time of flight of the best transfer
    double r_cap = 0.0;          // m, capture periapsis used (0 = none)
    double v_inf = 0.0;          // m/s, hyperbolic excess at the target
    double capture_orbit_period = -1.0; // s, period of the r_cap circle
    double transfer_semi_major = 0.0;   // m, negative for a hyperbolic leg
    double transfer_ecc = 0.0;
    glm::dvec3 v_departure;      // transfer-conic velocity at r1 (parent frame)
    glm::dvec3 v_arrival;        // transfer-conic velocity at r2 (parent frame)
    glm::dvec3 v_target_at_arrival; // target's own velocity at t + tof
    bool valid = false;
};

/* Semi-major axis of the conic cannot be determined from geometry alone
   without time of flight. We retain a stub here to prevent linker errors 
   if used elsewhere in the project. Use specific orbital energy instead. */
inline double transfer_semi_major_of(const glm::dvec3 &, const glm::dvec3 &, double, double) {
    return 0.0;
}

/* Lambert problem: find the conic under mu that goes from r1 to r2 in
   exactly `tof` seconds, and return its velocity at both endpoints.
   Returns false if no solution converged (degenerate geometry). */
inline bool solveLambert(const glm::dvec3 &r1, const glm::dvec3 &r2_in,
                         double mu, double tof,
                         glm::dvec3 &v1, glm::dvec3 &v2) {
    glm::dvec3 r2 = r2_in;
    const double r1l = glm::length(r1);
    double r2l = glm::length(r2);
    double cos_dnu = glm::dot(r1, r2) / (r1l * r2l);
    
    if(r1l < 1e-9 || r2l < 1e-9 || mu <= 0.0 || tof == 0.0) {
        return false;
    }

    // 180 degree transfer: the orbit plane is undefined (A -> 0), so the
    // y-formulation is singular. Handle analytically: the conic is uniquely
    // determined (periapsis at the smaller radius, apoapsis at the larger),
    // a = (r1+r2)/2, and the velocities are the vis-viva values in the
    // prograde direction. The prograde sense is chosen so the transfer goes
    // "the short way" around the assumed normal.
    if(cos_dnu < -0.9999999) {
        const double a = 0.5 * (r1l + r2l);
        // The conic is unique, so the ToF is fixed at half its period. Only
        // this ToF is solvable; any other is rejected (no conic fits).
        const double T = 2.0 * M_PI * std::sqrt(a * a * a / mu);
        if(std::fabs(tof - 0.5 * T) > 1e-6 * T) { return false; }
        const double v1m = std::sqrt(mu * (2.0 / r1l - 1.0 / a));
        const double v2m = std::sqrt(mu * (2.0 / r2l - 1.0 / a));
        glm::dvec3 normal(0.0, 0.0, 1.0);
        if(std::abs(r1.z) > 0.9 * r1l) { normal = glm::dvec3(0.0, 1.0, 0.0); }
        glm::dvec3 pg1 = glm::normalize(glm::cross(normal, r1));
        glm::dvec3 pg2 = glm::normalize(glm::cross(normal, r2));
        v1 = pg1 * v1m;
        v2 = pg2 * v2m;
        return true;
    }
    
    // A = sin(dnu) * sqrt(r1 r2 / (1 - cos dnu)) -> simplifies algebraically:
    const double A = std::sqrt(r1l * r2l * (1.0 + cos_dnu));
    
    // Universal variable formulation (Bate, Mueller & White).
    // F(z) = (y(z)/C(z))^{1.5} S(z) + A sqrt(y(z)) - sqrt(mu) * tof = 0
    auto F = [&](double z) -> double {
        const double C = stumpffC(z);
        const double S = stumpffS(z);
        const double y = r1l + r2l + A * (z * S - 1.0) / std::sqrt(C);
        if(y < 0.0) return 1e20; // invalid z
        return std::pow(y / C, 1.5) * S + A * std::sqrt(y) - std::sqrt(mu) * tof;
    };
    
    // Scan for a bracket between two REAL values (y >= 0). The y < 0 region
    // is where no physical conic exists; F is a guard sentinel there and must
    // never be used for bracketing (a sign "change" against the sentinel is
    // the boundary of the physical domain, not a root). F is monotone in z on
    // the short-transfer branch, so the first real sign change is the root.
    const double z_min = -4.0 * M_PI * M_PI;
    const double z_max = 4.0 * M_PI * M_PI;

    double z_lo = 0.0, z_hi = 0.0, f_lo = 0.0;
    double z_prev = 0.0, f_prev = 0.0;
    bool have_prev = false, bracketed = false;
    for(int i = 0; i <= 400; i++) {
        const double z = z_min + (z_max - z_min) * i / 400.0;
        const double fz = F(z);
        if(fz >= 1e19) { continue; }   // outside the physical domain
        if(have_prev && f_prev * fz < 0.0) {
            z_lo = z_prev; z_hi = z;
            f_lo = f_prev;
            bracketed = true;
            break;
        }
        z_prev = z; f_prev = fz;
        have_prev = true;
    }
    if(!bracketed) { return false; }   // no conic reaches r2 in this tof

    // Bisect (F monotone in z on this branch).
    double z = 0.5 * (z_lo + z_hi);
    const double target = std::sqrt(mu) * std::fabs(tof);
    for(int iter = 0; iter < 100; iter++) {
        z = 0.5 * (z_lo + z_hi);
        const double fz = F(z);
        if(fz >= 1e19) { z_lo = z; continue; }   // back out of the guard region
        if(std::fabs(fz) < 1e-10 * (target + 1.0)) { break; }
        if(fz * f_lo < 0.0) { z_hi = z; } else { z_lo = z; f_lo = fz; }
        if((z_hi - z_lo) < 1e-12 * (1.0 + std::fabs(z))) { break; }
    }
    
    const double C = stumpffC(z);
    const double S = stumpffS(z);
    const double y = r1l + r2l + A * (z * S - 1.0) / std::sqrt(C);
    
    const double f = 1.0 - y / r1l;
    const double g_dot = 1.0 - y / r2l;
    const double g = A * std::sqrt(y / mu);
    
    if(std::fabs(g) < 1e-20) return false; 

    v1 = (r2 - f * r1) / g;
    v2 = (g_dot * r2 - r1) / g;
    
    return true;
}

/* Minimum-total-dv transfer from (r1, v1) to (r2_0, v2_0) under mu_parent,
   with a capture burn at periapsis r_cap around the target (mu_target).
   Sweeps `n` linear ToF samples in [tof_min, tof_max] and keeps the best.
   Set mu_target <= 0 / capture = false for an intercept-only solution
   (e.g. targeting another ship): dv_capture is then 0. */
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
        glm::dvec3 vt1, vt2;
        if(!solveLambert(r1, r2, mu_parent, tof, vt1, vt2)) { continue; }

        TransferSolution s;
        s.tof = tof;
        s.dv_departure = glm::length(vt1 - v1);
        s.v_departure = vt1;
        s.v_arrival = vt2;
        s.v_target_at_arrival = v2;
        s.v_inf = glm::length(vt2 - v2);
        
        // Compute semi-major axis precisely from specific orbital energy
        const double v_dep_sq = glm::dot(vt1, vt1);
        const double r1l = glm::length(r1);
        const double energy = 0.5 * v_dep_sq - mu_parent / r1l;
        if(std::fabs(energy) < 1e-12) {
            s.transfer_semi_major = 0.0; // Parabolic
        } else {
            s.transfer_semi_major = -mu_parent / (2.0 * energy);
        }
        
        // Eccentricity derived from angular momentum and semi-major axis
        const double h2 = glm::dot(glm::cross(r1, vt1), glm::cross(r1, vt1));
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
            s.capture_orbit_period = 2.0 * M_PI * std::sqrt(r_cap * r_cap * r_cap / mu_target);
        }
        s.total_dv = s.dv_departure + s.dv_capture;
        s.valid = true;

        if(!best.valid || s.total_dv < best.total_dv) {
            best = s;
        }
    }
    return best;
}
