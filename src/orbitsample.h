#pragma once
// orbitsample.h -- sample a closed orbit's points, cached on the elements.
// Header-only pure math. Points are in the focus body's INERTIAL frame.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>

#include "orbit.h"

// Relative-tolerant equality: coasting re-propagates leave last-bit noise on
// the same conic; exact == used to miss every frame and thrash the cache.
inline bool orbitKeyEq(double x, double y) {
    const double d = std::fabs(x - y);
    const double s = std::max(std::fabs(x), std::fabs(y));
    return d <= 1e-6 * s + 1e-3;
}

// Unit directions: 1-dot ~ angle^2/2. 1e-6 is far below any real plane change.
inline bool orbitKeyEqDir(const glm::dvec3 &u, const glm::dvec3 &v) {
    return glm::dot(u, v) >= 1.0 - 1e-6;
}

// One cached orbit sampling. Reuse one instance per orbiting body.
struct OrbitSampleCache {
    bool valid = false;
    // Conic key (incl. N, so a LOD change cannot serve the wrong grid).
    double a = 0.0, e = 0.0, mu = 0.0;
    glm::dvec3 h_hat = glm::dvec3(0.0, 1.0, 0.0);  // orbital-plane normal
    glm::dvec3 e_hat = glm::dvec3(1.0, 0.0, 0.0);  // periapsis direction
    int N = 0;
    double apoapsis = -1.0;  // cheap LOD / view-cull input
    std::vector<glm::dvec3> pts;  // in the focus's inertial frame

    // use_cache: caller decides (a ship passes its onRails flag; off rails the
    // orbit is moving, so re-sample every frame). On a near-circle (e <= 1e-2)
    // the periapsis direction is noise and is NOT part of the key (keying on it
    // would thrash). Returns empty for a non-closed orbit.
    const std::vector<glm::dvec3> &sample(const glm::dvec3 &pos,
                                          const glm::dvec3 &vel, double mu,
                                          int N, bool use_cache = true) {
        const OrbitElements o = computeOrbitElements(pos, vel, mu);
        const bool closed = o.ecc < 1.0 && o.period > 0.0;
        if(!closed) {
            valid = false;
            pts.clear();
            apoapsis = -1.0;
            return pts;
        }
        const glm::dvec3 h = glm::cross(pos, vel);
        const double h_len = glm::length(h);
        const glm::dvec3 h_hat =
            h_len > 0.0 ? h / h_len : glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 e_hat(1.0, 0.0, 0.0);
        {
            const double r = glm::length(pos);
            const glm::dvec3 evec =
                (r > 0.0) ? glm::cross(vel, h) / mu - pos / r : glm::dvec3(0.0);
            const double e_len = glm::length(evec);
            if(e_len > 1e-12) { e_hat = evec / e_len; }
        }
        const bool peri_stable = o.ecc > 1e-2;
        if(use_cache && valid && N == this->N &&
           orbitKeyEq(o.semi_major, a) && orbitKeyEq(o.ecc, e) &&
           orbitKeyEq(mu, this->mu) && orbitKeyEqDir(h_hat, this->h_hat) &&
           (!peri_stable || orbitKeyEqDir(e_hat, this->e_hat))) {
            return pts;  // hit: coasting orbit, same conic
        }
        pts.clear();
        pts.reserve(N);
        // Even grid in ECCENTRIC ANOMALY, not uniform in time (which clusters
        // at apoapsis and starves periapsis on an eccentric ellipse).
        const double n_mean = 2.0 * std::numbers::pi / o.period;   // mean motion, rad/s
        for(int i = 0; i < N; i++) {
            const double E = 2.0 * std::numbers::pi * i / N;
            const double M = E - o.ecc * std::sin(E);
            const double dt = (M - o.mean_anomaly) / n_mean;
            glm::dvec3 p, v;
            propagateKepler(pos, vel, mu, dt, p, v);
            pts.push_back(p);
        }
        a = o.semi_major; e = o.ecc;
        this->mu = mu;  // param shadows the member; store it explicitly
        this->h_hat = h_hat; this->e_hat = e_hat;
        this->N = N;
        apoapsis = o.apoapsis;
        valid = true;
        return pts;
    }
};

// Cheap conic size for view culling / LOD, before the expensive sampling.
inline void orbitConicSize(const glm::dvec3 &pos, const glm::dvec3 &vel,
                           double mu, double &sma, double &apo) {
    sma = 0.0; apo = -1.0;
    if(!(mu > 0.0)) { return; }
    const double r = glm::length(pos);
    if(r < 1e-9) { return; }
    const double v2 = glm::dot(vel, vel);
    const double a = 1.0 / (2.0 / r - v2 / mu);
    if(!(a > 0.0)) { return; }  // open / degenerate: no closed ellipse to cull
    const glm::dvec3 h = glm::cross(pos, vel);
    const double e2 = std::max(0.0, 1.0 - glm::dot(h, h) / (mu * a));
    sma = a;
    apo = a * (1.0 + std::sqrt(e2));
}

// Quantized sample count from on-screen radius; quantization keeps a slow zoom
// from thrashing the cache key (which includes N).
inline int orbitSamplesForRadius(double r_px) {
    if(r_px < 24.0) { return 12; }
    if(r_px < 80.0) { return 24; }
    if(r_px < 240.0) { return 48; }
    return 64;
}

// Sample an OPEN trajectory as a finite arc in TRUE ANOMALY, truncated at
// r_cap, symmetric about periapsis. Nothing to cache (no period); cheap to
// re-sample per frame. r_cap should be >= the ship's current radius so the
// ship lies on the arc. Empty for a closed orbit or degenerate state.
inline std::vector<glm::dvec3> sampleOpenTrajectory(const glm::dvec3 &pos,
                                                     const glm::dvec3 &vel,
                                                     double mu, int N,
                                                     double r_cap) {
    std::vector<glm::dvec3> pts;
    if(!(mu > 0.0) || N < 2 || !(r_cap > 0.0)) { return pts; }
    const double r0 = glm::length(pos);
    if(r0 < 1e-9) { return pts; }
    const glm::dvec3 h = glm::cross(pos, vel);
    const double h_len = glm::length(h);
    if(h_len < 1e-9) { return pts; }   // radial: no conic
    const OrbitElements o = computeOrbitElements(pos, vel, mu);
    if(!(o.ecc >= 1.0)) { return pts; }  // closed orbit, not an open arc
    const double e = o.ecc;
    const double p = h_len * h_len / mu;   // semi-latus rectum, always > 0
    // Truncate where r(nu) reaches r_cap; clamp guards the e ~ 1 edge.
    const double nu_cap = std::acos(glm::clamp((p / r_cap - 1.0) / e, -1.0, 1.0));
    const glm::dvec3 hhat = h / h_len;
    const glm::dvec3 evec = glm::cross(vel, h) / mu - pos / r0;
    const double e_len = glm::length(evec);
    if(e_len < 1e-9) { return pts; }
    const glm::dvec3 rhat_p = evec / e_len;
    const glm::dvec3 thathat = glm::cross(hhat, rhat_p);
    pts.reserve(N);
    for(int i = 0; i < N; i++) {
        const double nu = -nu_cap + (2.0 * nu_cap) * (double)i / (N - 1);
        const double r = p / (1.0 + e * std::cos(nu));
        pts.push_back(r * (std::cos(nu) * rhat_p + std::sin(nu) * thathat));
    }
    return pts;
}

// Sample a TRANSFER CONIC arc (departure state propagated over tof) as N+1
// points, even in ANOMALY (endpoints exactly at departure / arrival).
// Near-circular or parabolic legs fall back to even-in-time (anomaly is
// degenerate there).
inline std::vector<glm::dvec3> sampleTransferArc(const glm::dvec3 &pos,
                                                  const glm::dvec3 &vel,
                                                  double mu, double tof,
                                                  int N) {
    std::vector<glm::dvec3> pts;
    if(N < 1 || !(mu > 0.0) || tof < 0.0) { return pts; }
    const OrbitElements o = computeOrbitElements(pos, vel, mu);
    const double e = o.ecc;
    const bool elliptic = (e >= 1e-3) && (e < 1.0);   // genuinely eccentric elliptic
    const bool hyperbolic = (e > 1.0);
    if(!elliptic && !hyperbolic) {   // near-circular (e < 1e-3) or parabolic (e = 1): even-in-time
        pts.reserve(N + 1);
        for(int i = 0; i <= N; i++) {
            glm::dvec3 p, v;
            propagateKepler(pos, vel, mu, tof * i / N, p, v);
            pts.push_back(p);
        }
        return pts;
    }
    // Elliptic anomaly wraps to [0, 2pi); unwrap one turn if arrival reads behind.
    glm::dvec3 arr_pos, arr_vel;
    propagateKepler(pos, vel, mu, tof, arr_pos, arr_vel);
    double A2 = computeOrbitElements(arr_pos, arr_vel, mu).ecc_anomaly;
    const double A1 = o.ecc_anomaly;
    if(elliptic && A2 < A1) { A2 += 2.0 * std::numbers::pi; }
    const double M1 = o.mean_anomaly;
    const double a_abs = std::fabs(o.semi_major);
    const double tau = elliptic
        ? o.period / (2.0 * std::numbers::pi)
        : std::sqrt(a_abs * a_abs * a_abs / mu);
    pts.reserve(N + 1);
    for(int i = 0; i <= N; i++) {
        const double A = A1 + (A2 - A1) * (double)i / N;
        const double M = hyperbolic ? e * std::sinh(A) - A : A - e * std::sin(A);
        glm::dvec3 p, v;
        propagateKepler(pos, vel, mu, (M - M1) * tau, p, v);
        pts.push_back(p);
    }
    return pts;
}
