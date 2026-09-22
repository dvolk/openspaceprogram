#pragma once
// orbitsample.h -- sample a closed orbit's points, cached on the elements.
//
// The orbital map draws each orbit as N points propagated over one period
// (propagateKepler, an iterative Kepler solve -- the cost center). That
// work is wasted when repeated frame-to-frame: a coasting two-body orbit has
// CONSTANT elements, so the same ellipse is re-propagated every frame for
// no reason. This caches the sampled points keyed on the orbit's elements,
// so a coasting orbit is propagated once and only recomputed when the orbit
// actually changes (a burn / SOI switch).
//
// The points live in the focus body's INERTIAL frame and are independent of
// the map's projection plane and scale, so the cache survives both changing
// (the plane/scale are applied later, by OrbitMap::project).
//
// Header-only pure math (orbit.h + glm + <vector>) so tests/ can pin it
// without rendering, imgui, Bullet, or GL.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>

#include "orbit.h"

// Relative-tolerant equality for the cache key. A coasting re-propagate
// (railsTick / UpdateOrbitRails) leaves the state numerically perturbed but
// on the same conic -- over thousands of in-place Kepler steps the last-bit
// noise accumulates to metres on a planetary radius -- while a real change
// (a burn, an SOI switch to a new central body) moves the elements by far
// more. Exact == used to miss every frame and re-run N Kepler solves, the
// map's cost center once a system has a few hundred bodies.
inline bool orbitKeyEq(double x, double y) {
    const double d = std::fabs(x - y);
    const double s = std::max(std::fabs(x), std::fabs(y));
    return d <= 1e-6 * s + 1e-3;
}

// Unit directions: 1 - dot = 1 - cos(angle) ~ angle^2/2. 1e-6 is ~1e-3 rad,
// far below any real plane / periapsis change (a burn, an SOI switch).
inline bool orbitKeyEqDir(const glm::dvec3 &u, const glm::dvec3 &v) {
    return glm::dot(u, v) >= 1.0 - 1e-6;
}

// One cached orbit sampling. Reuse one instance per orbiting body (the
// cache is keyed internally on the conic, so a body that changes orbit
// -- a burn -- simply misses and re-samples).
struct OrbitSampleCache {
    bool valid = false;
    // Conic key: size, shape, mu, and the ellipse's orientation in the
    // sampling frame (plane normal + periapsis). N is part of the key so a
    // LOD change of the sample count cannot serve the wrong grid.
    double a = 0.0, e = 0.0, mu = 0.0;
    glm::dvec3 h_hat = glm::dvec3(0.0, 1.0, 0.0);  // orbital-plane normal
    glm::dvec3 e_hat = glm::dvec3(1.0, 0.0, 0.0);  // periapsis direction
    int N = 0;
    // Apoapsis radius of the cached ellipse (m; -1 when not valid / open).
    // Cheap LOD + view-cull input -- callers get it free after a sample.
    double apoapsis = -1.0;
    std::vector<glm::dvec3> pts;  // in the focus's inertial frame

    // Returns the orbit's N sampled points, or an empty vector if (pos, vel,
    // mu) is not a closed (elliptic) orbit.
    //
    // use_cache: whether a prior sampling may be reused. When true, a cache
    // hit (conic + mu + N unchanged) returns the stored points without
    // re-propagating; when false the points are always re-sampled. The caller
    // owns this decision: a ship passes its onRails flag (coasting on a
    // Keplerian conic => the ellipse is fixed and safe to reuse; off rails --
    // Bullet-integrated, or right after a burn / staging / SOI switch / crash
    // -- the orbit is moving, so re-sample every frame). Terrain bodies are
    // always on their conic, so they sample with use_cache true -- ideally
    // from the rail's immutable epoch state (orbit_pos0/orbit_vel0), which
    // makes the key bit-stable.
    //
    // On a near-circle (e <= 1e-2) the periapsis direction is numerically
    // meaningless (ecc_vec is noise) and is NOT part of the key: the points
    // still lie on the right circle, and keying on e_hat there would thrash.
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
        // Even grid in ECCENTRIC ANOMALY, not uniform in time. Uniform-in-time
        // (period * i / N) spaces points by how fast the ship is: they cluster
        // where it is slow (near apoapsis) and starve periapsis, so an
        // eccentric ellipse draws lopsided (a sparse spike at periapsis).
        // Stepping by equal eccentric-anomaly increments instead gives an even
        // ellipse. Point i lands at eccentric anomaly E_i = 2 pi i / N no
        // matter the start state's phase: mean anomaly M = E - e sin E fixes
        // the propagation time (dt = (M_i - M0) / n), and the propagator
        // carries any start state on the same ellipse to that same point. So
        // the N points are a fixed even grid on the ellipse -- identical for
        // every ship on it, and always including the exact periapsis (E = 0)
        // and apoapsis (E = pi) when N is even.
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

// Cheap conic size (semi-major + apoapsis) from a state, without sampling
// or filling a cache -- enough for view culling and the sample-count LOD
// to run BEFORE the (expensive) point generation.
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

// Quantized sample count from the orbit's on-screen radius: a 20 px smudge
// does not need 64 chords, and the quantization keeps a slow zoom from
// thrashing the cache key (which includes N). ~8 px per chord.
inline int orbitSamplesForRadius(double r_px) {
    if(r_px < 24.0) { return 12; }
    if(r_px < 80.0) { return 24; }
    if(r_px < 240.0) { return 48; }
    return 64;
}

// Sample an OPEN (hyperbolic or parabolic) two-body trajectory as a finite
// arc, evenly spaced in TRUE ANOMALY, truncated where the radius would
// exceed r_cap. The arc is symmetric about periapsis (nu in [-nu_cap,
// +nu_cap]), so it shows both the leg the ship came in on and the leg it is
// flying out on. Points are in the SAME frame as `pos` (the focus's inertial
// frame). Returns an empty vector for a closed orbit (ecc < 1) or a
// degenerate state (no angular momentum).
//
// An open trajectory has no period, so there is nothing to cache the way
// OrbitSampleCache does for a coasting ellipse; the caller re-samples
// whenever the state changes. It is cheap (N trig evaluations, no Kepler
// solve), so per-frame re-sampling is fine even off rails.
//
// r_cap should be at least the ship's current radius so the ship itself lies
// on the arc; a view-extent value makes the curve run to the edge of the map.
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
    // Truncate where r(nu) = p / (1 + e cos nu) reaches r_cap:
    //   cos nu = (p / r_cap - 1) / e.
    // For e > 1 this bound is > -1 (a finite arc); as r_cap -> inf it
    // approaches the asymptote angle acos(-1 / e). The clamp guards the
    // e ~ 1 (parabolic) edge where the bound can dip to -1.
    const double nu_cap = std::acos(glm::clamp((p / r_cap - 1.0) / e, -1.0, 1.0));
    // Orientation basis in pos's frame: rhat_p toward periapsis (along the
    // eccentricity vector), thathat 90 deg ahead in the direction of motion.
    // A point at true anomaly nu is then r(nu) (cos nu rhat_p + sin nu thathat)
    // -- the same decomposition the ship's current position satisfies, so the
    // arc passes through the ship.
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

// Sample a TRANSFER CONIC arc -- the Kepler orbit from the departure state
// (pos, vel) under mu, propagated over tof seconds (one Lambert leg) -- as
// N+1 points from departure (i = 0) to arrival (i = N).
//
// The points are spread evenly in ANOMALY (eccentric for an elliptic leg,
// hyperbolic for a hyperbolic one) -- the same even grid OrbitSampleCache
// uses for a closed orbit -- NOT uniform in time. Uniform-in-time spaces
// points by how fast the ship is: they crowd the slow end of the arc (near
// its apoapsis) and starve the fast periapsis end, so a transfer leg draws
// lopsided. Even-in-anomaly gives it an even outline.
//
// The arc spans anomaly [A1, A2] (departure to arrival); the anomaly
// increases monotonically with time, so A2 >= A1. Sample i lands at
// A_i = A1 + (A2 - A1) i / N. Its propagation time is (M(A_i) - M(A1)) / n,
// where M is the mean anomaly (A - e sin A elliptic, e sinh A - A
// hyperbolic) and n the mean motion; propagateKepler then carries the
// departure state to that point, so the endpoints are EXACTLY the departure
// (dt = 0) and arrival (dt = tof) states.
//
// A transfer leg spans less than one full orbit (you are going TO the
// target, not orbiting), so at most one 2pi turn is unwrapped for the
// elliptic case. A near-circular leg (e < 1e-3) has no well-defined anomaly
// (the reference direction is degenerate, and the speed is ~constant), so
// even-in-time is already even there and is used instead. A parabolic leg
// (e = 1 exactly, no period -- measure zero in practice) falls back to
// even-in-time as well.
//
// pos/vel are the departure state in the focus's inertial frame; the
// returned points are in that same frame (like the other samplers).
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
    // The arrival state lies on the SAME conic; its anomaly is the arc's far
    // end. For an elliptic leg the stored anomaly is wrapped to [0, 2pi), so
    // if the arrival reads behind the departure the arc crossed the seam and
    // the true arrival anomaly is one turn ahead. (A hyperbolic anomaly is
    // unbounded and already ordered, so no unwrap is needed.)
    glm::dvec3 arr_pos, arr_vel;
    propagateKepler(pos, vel, mu, tof, arr_pos, arr_vel);
    double A2 = computeOrbitElements(arr_pos, arr_vel, mu).ecc_anomaly;
    const double A1 = o.ecc_anomaly;
    if(elliptic && A2 < A1) { A2 += 2.0 * std::numbers::pi; }
    // Seconds per radian of mean anomaly: period/(2pi) elliptic,
    // sqrt(|a|^3/mu) hyperbolic (the t = M/n and t = M sqrt(a^3/mu) relations).
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
