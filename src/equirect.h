#pragma once
// equirect.h -- the game's ONE equirectangular (lon, lat) <-> direction
// convention over a body's ROTATING frame, and the shared map layout.
//
// Axes: north = +Y, lon 0 = +X (the rail longitude zero, #146/#200),
// lon +pi/2 = -Z (east). Inverse is lon = atan2(-z, x), unwrapped to
// [0, 2pi). Forward is (cos lat * cos lon, sin lat, -cos lat * sin lon).
// Surface lon 0 is the body's prime meridian (rot-frame +X = WGCCRE /
// spin_phase0), so map, HUD readout and rail longitudes share one azimuth.
//
// Map layout (surfmap pixels, heightmaps, cloud coverage): lon 0 at the
// LEFT edge (column 0), north up (row 0). Heightmap / surfmap grids put
// the poles ON the first/last row (v scaled by h-1); a tileable bake
// (cloud coverage) instead samples pixel CENTRES and may use v in [0,1]
// scaled by H. Same direction formulas -- only the grid parametrisation
// differs.
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
#include <limits>
#include <numbers>

#include <glm/glm.hpp>

#include "angmath.h"   // asinUnit: the bounded-ratio spelling (#622)

/* "No coordinate was given", for the DEGREES spelling the authored coordinates
   below use: a world_buildings pad's lat_deg/lon_deg, a start ship's lat/lon
   (--startship / --startships). It lives here rather than in constants.h because
   it changes nothing in the game -- it is the ABSENCE of one of these coordinates.
   NaN and not a number outside the range: every pair in [-90,90] x (any angle) is
   a real place, so a range sentinel is a value someone could type into the live
   path. NaN never equals itself, so test it with isnan(), never `!= kNoCoord`. A
   NaN that reaches a spawn is a half-filled pair, which the spawn asserts on
   rather than guesses. */
inline constexpr double kNoCoord = std::numeric_limits<double>::quiet_NaN();

/* The lon half of the authored-coordinate law: wrap once, at the parse, into
   [0, 360). Wrapping there is what makes the angle the trig sees and the number a
   readout echoes the SAME number: a raw 1e308 fed to cos/sin is an arbitrary
   meridian with no precision left to say which, while an echo that fmod'd it
   afterwards would print a tidy place the ship is not at. A wrapped 1e308 is
   still an arbitrary meridian -- it is just the honest one. Same law as the
   inverse above, which unwraps lon into [0, 2pi). */
inline double wrapLonDeg(double lon_deg) {
    const double w = std::fmod(lon_deg, 360.0);
    return w < 0.0 ? w + 360.0 : w;
}

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
    lat = asinUnit(d.y);
    lon = std::atan2(-d.z, d.x);
    if(lon < 0.0) { lon += 2.0 * std::numbers::pi; }
}

/* The same inverse in the DEGREES spelling an authored table is keyed by -- a wind row's
   lat/lon band (#767), a pad's lat_deg/lon_deg. Spelled once here rather than as a
   rad2deg at each call site, so the number a file is keyed by and the number a direction
   unwraps to cannot drift apart. wrapLonDeg() runs AFTER the scale: 2*pi rounds to a 360
   that no half-open band covers, and a place outside every band is not a place. */
inline void equirectLonLatDeg(const glm::dvec3 &d, double &lon_deg, double &lat_deg) {
    double lon = 0.0, lat = 0.0;
    equirectLonLat(d, lon, lat);
    constexpr double rad2deg = 180.0 / std::numbers::pi;
    lon_deg = wrapLonDeg(lon * rad2deg);
    lat_deg = lat * rad2deg;
}

/* Local horizontal unit vectors at a surface direction, in the same rotating-frame
   axes as equirectDir: east points along INCREASING lon, north along increasing lat.
   Both are perpendicular to `up`, which is what makes them the basis a horizontal
   velocity (the wind, #688) is authored in -- a wind is a surface direction that is
   not a place, and this is the only place that basis is spelled.

   east = normalize(Y x up) is exact off the poles: for up = equirectDir(lon, lat),
   Y x up = (up.z, 0, -up.x) = cos(lat) * (the unit east). EXACTLY at a pole the
   meridian is undefined and that cross product is zero; east then falls back to
   lon 0's east, which is the limit of approaching the pole along that meridian --
   arbitrary, but never NaN and never off the horizontal plane. north = up x east
   is unit whenever up and east are unit and perpendicular. */
inline glm::dvec3 equirectEast(const glm::dvec3 &up) {
    const glm::dvec3 e(up.z, 0.0, -up.x);
    const double l2 = glm::dot(e, e);
    if(l2 < 1e-24) { return glm::dvec3(0.0, 0.0, -1.0); }   // a pole: lon 0's east
    return e * (1.0 / std::sqrt(l2));
}

inline glm::dvec3 equirectNorth(const glm::dvec3 &up, const glm::dvec3 &east) {
    return glm::cross(up, east);
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
