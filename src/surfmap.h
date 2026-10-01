#pragma once
// surfmap.h -- the "Surface Map" window's 2-D projection + shading (pure
// math). EQUIRECTANGULAR over the body's ROTATING frame: north up, east to
// the right, lon 0 at the left edge. column i: lon = 2 pi i / w (+Z -> +X);
// row j: lat = pi/2 - pi j / (h-1) (row 0 = north pole).

#include <cmath>
#include <numbers>

#include <glm/glm.hpp>

// Unit surface direction at pixel (i, j) of a w x h map.
inline glm::dvec3 surfmapDir(int i, int j, int w, int h) {
    const double lon = 2.0 * std::numbers::pi * (double)i / (double)w;
    const double lat = std::numbers::pi * 0.5 - std::numbers::pi * (double)j / (double)(h - 1);
    const double cl = std::cos(lat);
    return glm::dvec3(cl * std::sin(lon), std::sin(lat), cl * std::cos(lon));
}

// Inverse of the pixel direction: a unit direction -> (lon, lat).
inline void surfmapLonLat(const glm::dvec3 &d, double &lon, double &lat) {
    lat = std::asin(glm::clamp((double)d.y, -1.0, 1.0));
    lon = std::atan2((double)d.x, (double)d.z);
    if(lon < 0.0) { lon += 2.0 * std::numbers::pi; }
}

// (lon, lat) -> fractional pixel (u, v) of a w x h map.
inline void surfmapPixel(double lon, double lat, int w, int h,
                         double &u, double &v) {
    u = lon / (2.0 * std::numbers::pi) * (double)w;
    v = (std::numbers::pi * 0.5 - lat) / std::numbers::pi * (double)(h - 1);
}

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
