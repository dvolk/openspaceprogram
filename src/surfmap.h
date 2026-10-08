#pragma once
// surfmap.h -- the "Surface Map" window's 2-D projection + shading.
// EQUIRECTANGULAR over the body's ROTATING frame: the direction <-> (lon,
// lat) <-> pixel math lives in equirect.h (lon 0 at the left edge, north
// up). This header adds the map-only helpers: terminator shade, the
// antimeridian wrap test, and the compute request.

#include <cmath>
#include <numbers>

#include <glm/glm.hpp>

#include "equirect.h"

// Terminator shading: light factor in [0.3, 1] (night keeps 30% ambient).
inline float surfmapShade(const glm::dvec3 &n, const glm::dvec3 &sun) {
    const double d = glm::dot(n, sun);
    const double t = glm::clamp((d + 0.05) / 0.20, 0.0, 1.0);
    return 0.30f + 0.70f * (float)t;
}

// Consecutive lon samples cross the antimeridian when their difference
// exceeds half a turn (the orbit line must break there).
inline bool surfmapWraps(double lon_prev, double lon_cur) {
    return std::fabs(lon_prev - lon_cur) > std::numbers::pi;
}

// Request the surface map: snapshot inputs on the main thread, post the
// sweep to the background worker (g.jobs).
struct Game;
void surfmapCompute(Game &g);
