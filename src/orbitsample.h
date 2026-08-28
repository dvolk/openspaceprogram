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

#include <vector>

#include <glm/glm.hpp>

#include "orbit.h"

// One cached orbit sampling. Reuse one instance per orbiting body (the
// cache is keyed internally on the elements, so a body that changes orbit
// -- a burn -- simply misses and re-samples).
struct OrbitSampleCache {
    bool valid = false;
    // Element key a closed orbit is defined by (constant while coasting).
    double a = 0.0, e = 0.0, inc = 0.0, raan = 0.0, argp = 0.0, mu = 0.0;
    std::vector<glm::dvec3> pts;  // in the focus's inertial frame

    // Returns the orbit's N sampled points, or an empty vector if (pos, vel,
    // mu) is not a closed (elliptic) orbit.
    //
    // use_cache: whether a prior sampling may be reused. When true, a cache
    // hit (elements + mu unchanged) returns the stored points without
    // re-propagating; when false the points are always re-sampled. The caller
    // owns this decision: a ship passes its onRails flag (coasting on a
    // Keplerian conic => the ellipse is fixed and safe to reuse; off rails --
    // Bullet-integrated, or right after a burn / staging / SOI switch / crash
    // -- the orbit is moving, so re-sample every frame). Terrain bodies are
    // always on their conic, so they sample with use_cache true.
    const std::vector<glm::dvec3> &sample(const glm::dvec3 &pos,
                                          const glm::dvec3 &vel, double mu,
                                          int N, bool use_cache = true) {
        const OrbitElements o = computeOrbitElements(pos, vel, mu);
        const bool closed = o.ecc < 1.0 && o.period > 0.0;
        if(!closed) {
            valid = false;
            pts.clear();
            return pts;
        }
        if(use_cache && valid && o.semi_major == a && o.ecc == e &&
           o.inclination == inc && o.raan == raan && o.arg_periapsis == argp &&
           mu == this->mu) {
            return pts;  // hit: coasting orbit, elements unchanged
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
        const double n_mean = 2.0 * M_PI / o.period;   // mean motion, rad/s
        for(int i = 0; i < N; i++) {
            const double E = 2.0 * M_PI * i / N;
            const double M = E - o.ecc * std::sin(E);
            const double dt = (M - o.mean_anomaly) / n_mean;
            glm::dvec3 p, v;
            propagateKepler(pos, vel, mu, dt, p, v);
            pts.push_back(p);
        }
        a = o.semi_major; e = o.ecc; inc = o.inclination;
        raan = o.raan; argp = o.arg_periapsis;
        this->mu = mu;  // param shadows the member; store it explicitly
        valid = true;
        return pts;
    }
};

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
