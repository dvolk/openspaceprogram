#pragma once
// surfmap.h -- the "Surface Map" window's 2-D projection + shading, as
// pure math (glm + <vector> only) so tests/ can pin it without rendering,
// Bullet, or GL (the pixel-buffer build, surfmapCompute, is in
// surfmap.cpp and links the game).
//
// The map is EQUIRECTANGULAR over the body's ROTATING frame -- the frame
// the terrain height + color functions (TerrainBody::SurfaceColor,
// GetTerrainHeight) live in, and the frame the HUD's lon/lat (render.cpp
// surf_pos) and the ship's dot (sub-satellite point) use:
//
//   column i (0..w-1):  lon = 2 pi i / w, measured from +Z toward +X
//       (the game's own convention: render.cpp does
//        longitude = atan2(dir.x, dir.z))
//   row j (0..h-1):     lat = pi/2 - pi j / (h-1)   (row 0 = north pole)
//   pixel direction:    (cos lat sin lon, sin lat, cos lat cos lon)
//
// So the map reads like a world map: north up, east (the spin direction
// for a prograde spinner) to the right, lon 0 at the left edge.

#include <cmath>

#include <glm/glm.hpp>

// Unit surface direction at pixel (i, j) of a w x h map.
inline glm::dvec3 surfmapDir(int i, int j, int w, int h) {
    const double lon = 2.0 * M_PI * (double)i / (double)w;
    const double lat = M_PI * 0.5 - M_PI * (double)j / (double)(h - 1);
    const double cl = std::cos(lat);
    return glm::dvec3(cl * std::sin(lon), std::sin(lat), cl * std::cos(lon));
}

// Inverse of the pixel direction: a unit direction -> (lon, lat).
// lon in [0, 2 pi) (the map's column range), lat in [-pi/2, pi/2].
// The inverse of render.cpp's (atan2(x, z), asin(y)) with the column
// range unwrapped.
inline void surfmapLonLat(const glm::dvec3 &d, double &lon, double &lat) {
    lat = std::asin(glm::clamp((double)d.y, -1.0, 1.0));
    lon = std::atan2((double)d.x, (double)d.z);
    if(lon < 0.0) { lon += 2.0 * M_PI; }
}

// (lon, lat) -> fractional pixel (u, v) of a w x h map: u in [0, w),
// v in [0, h-1]. For an overlay drawn over the displayed image, scale
// (u / w, v / (h-1)) into the image rect.
inline void surfmapPixel(double lon, double lat, int w, int h,
                         double &u, double &v) {
    u = lon / (2.0 * M_PI) * (double)w;
    v = (M_PI * 0.5 - lat) / M_PI * (double)(h - 1);
}

// Terminator (the star's day/night shading): a light factor in [0.3, 1]
// for the surface normal n under a sun direction `sun` (unit, TOWARD the
// star, same frame as n). Full day where n points at the sun, full night
// where it points away, a soft band across the terminator between. The
// night side keeps 30% (ambient) so the map reads, not black.
inline float surfmapShade(const glm::dvec3 &n, const glm::dvec3 &sun) {
    const double d = glm::dot(n, sun);
    const double t = glm::clamp((d + 0.05) / 0.20, 0.0, 1.0);
    return 0.30f + 0.70f * (float)t;
}

// A run of consecutive lon samples (each in [0, 2 pi)) crosses the
// antimeridian between them when their difference exceeds half a turn --
// the map's left and right edges are the same meridian, so an orbit line
// must break there instead of sweeping across the whole map.
inline bool surfmapWraps(double lon_prev, double lon_cur) {
    return std::fabs(lon_prev - lon_cur) > M_PI;
}

// Build the surface map's pixel buffer for g (fills g.surfmap_* state).
// g (Game) is forward-declared so this header stays light; the
// implementation (surfmap.cpp) does the TerrainBody sampling.
struct Game;
void surfmapCompute(Game &g);
