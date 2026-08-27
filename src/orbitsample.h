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
        for(int i = 0; i < N; i++) {
            glm::dvec3 p, v;
            propagateKepler(pos, vel, mu, o.period * i / N, p, v);
            pts.push_back(p);
        }
        a = o.semi_major; e = o.ecc; inc = o.inclination;
        raan = o.raan; argp = o.arg_periapsis;
        this->mu = mu;  // param shadows the member; store it explicitly
        valid = true;
        return pts;
    }
};
