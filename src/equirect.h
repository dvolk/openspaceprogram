#pragma once
// equirect.h -- the game's ONE equirectangular (lon, lat) <-> direction
// convention over a body's ROTATING frame, and the shared map layout.
//
// Axes: north = +Y, lon 0 = +X (the rail longitude zero, #146/#200),
// lon +pi/2 = -Z (east). Inverse is lon = atan2(-z, x), unwrapped to
// [0, 2pi). Forward is (cos lat * cos lon, sin lat, -cos lat * sin lon).
// Surface lon 0 is the body's prime meridian (rot-frame +X = WGCCRE /
// spin_phase0), so the map, the HUD readout, and rail longitudes all
// share one azimuth.
//
// Map layout (surfmap pixels, heightmaps, cloud coverage): lon 0 at the
// LEFT edge (column 0), north up (row 0). Heightmap / surfmap grids put
// the poles ON the first/last row (v scaled by h-1); a tileable bake
// (cloud coverage) instead samples pixel CENTRES and may use v in [0,1]
// scaled by H. Both use the same direction formulas -- only the grid
// parametrisation differs.
//
// Everything that maps a surface direction to a map column (or back)
// MUST go through these. No local atan2 / sin/cos lon formulas.
//
// create_atmosphere_mesh (terrain.cpp) carries unwrapped phi = atan2(z, x)
// in the vertex colour slot (phi = 0 at +X toward +Z). Surface lon is
// -phi (mod 2pi), so the cloud deck's u = lon/2pi is -phi/2pi
// (cloudShader.vs; texture REPEAT closes the seam).

#include <cassert>
#include <cmath>
#include <numbers>

#include <glm/glm.hpp>

// (lon, lat) -> unit direction. lon 0 = +X, +pi/2 = -Z, north = +Y.
inline glm::dvec3 equirectDir(double lon, double lat) {
    const double cl = std::cos(lat);
    return glm::dvec3(cl * std::cos(lon), std::sin(lat), -cl * std::sin(lon));
}

inline glm::vec3 equirectDir(float lon, float lat) {
    const float cl = std::cos(lat);
    return glm::vec3(cl * std::cos(lon), std::sin(lat), -cl * std::sin(lon));
}

// Unit direction -> (lon, lat), lon unwrapped to [0, 2pi).
inline void equirectLonLat(const glm::dvec3 &d, double &lon, double &lat) {
    lat = std::asin(glm::clamp(d.y, -1.0, 1.0));
    lon = std::atan2(-d.z, d.x);
    if(lon < 0.0) { lon += 2.0 * std::numbers::pi; }
}

inline void equirectLonLat(const glm::vec3 &d, float &lon, float &lat) {
    lat = std::asin(glm::clamp(d.y, -1.0f, 1.0f));
    lon = std::atan2(-d.z, d.x);
    if(lon < 0.0f) { lon += 2.0f * std::numbers::pi_v<float>; }
}

// Fractional pixel (u, v) of a w x h poles-on-rows map (surfmap /
// heightmap): u = lon/2pi * w (lon 0 at column 0), v = (pi/2 - lat)/pi
// * (h-1) (row 0 = north pole, row h-1 = south).
inline void equirectPixel(double lon, double lat, int w, int h,
                          double &u, double &v) {
    assert(w >= 2 && h >= 2);
    u = lon / (2.0 * std::numbers::pi) * (double)w;
    v = (std::numbers::pi * 0.5 - lat) / std::numbers::pi * (double)(h - 1);
}

inline void equirectPixel(float lon, float lat, int w, int h,
                          float &u, float &v) {
    assert(w >= 2 && h >= 2);
    u = lon / (2.0f * std::numbers::pi_v<float>) * (float)w;
    v = (std::numbers::pi_v<float> * 0.5f - lat) / std::numbers::pi_v<float>
      * (float)(h - 1);
}

// Pixel (i, j) -> unit direction on a w x h poles-on-rows map (the
// grid sample at i, j -- NOT the pixel centre).
inline glm::dvec3 equirectDirAt(int i, int j, int w, int h) {
    assert(w >= 2 && h >= 2);
    const double lon = 2.0 * std::numbers::pi * (double)i / (double)w;
    const double lat = std::numbers::pi * 0.5
                     - std::numbers::pi * (double)j / (double)(h - 1);
    return equirectDir(lon, lat);
}
