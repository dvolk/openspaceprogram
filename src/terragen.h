// terragen.h -- terrain generation as pure math (glm + STL only: no GL,
// no Bullet, no game state), so tests/ can pin it without the render
// chain (like surfmap.h) and terrain work can iterate on it in isolation.
//
//   Surface / TerrainParams   per-body terrain + color params (the
//                             "surface" JSON block); value-copyable so a
//                             worker thread can hold a snapshot (the async
//                             subdivision job does -- see terrain.h)
//   terrainHeight(...)        the analytic height function (a star function
//                             in the body's rotating frame)
//   terrainSurfaceColor(...)  the exact per-vertex color the grid bakes
//                             (noise, palette / band, sea, contrast)
//   buildGridGeom(...)        the grid a GeoPatch draws: size x size
//                             terrain vertices + normals + colors, an
//                             optional skirt ring, and the indices
//
// The game-side half (GL upload into a Mesh, Bullet collision, the patch
// tree, LOD) lives in terrain.h / terrain.cpp: the GeoPatch ctor consumes
// a GridGeom on the main thread.

#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/noise.hpp>

typedef struct {
    float r, g, b;
} COLOUR;

// One stop on a body's land-color ramp: elevation fraction t in [0,1]
// (0 = sea level / lowest land, 1 = highest relief) and the color there.
struct PaletteStop {
    float t;
    glm::vec3 color;
};

// Per-body atmosphere appearance (optional "surface.atmosphere" block).
// v1 is a single Fresnel limb-glow shell drawn over the terrain; see
// reports/atmosphere2026_08_25/atmosphere.md for the design + roadmap.
struct AtmosphereParams {
    bool enabled = false;
    glm::vec3 color = glm::vec3(0.3f, 0.5f, 1.0f);  // rim tint (N2/O2 blue)
    float thickness = 0.0f;   // [m] shell radius above radius + max_height
    float power = 3.0f;       // Fresnel falloff (higher = tighter rim)
    float intensity = 1.0f;   // overall alpha scale
};

// Per-body terrain + color parameters (the optional "surface" JSON block).
// Defaults reproduce the legacy hardcoded behavior.
struct Surface {
    float amplitude = 2500.0f;   // [m] peak noise height
    int octaves = 12;            // simplex octaves
    float persistence = 0.6f;    // octave falloff
    float frequency = 1.0f;      // noise-domain scale
    int power = 3;               // height-distribution exponent
    bool has_sea = false;
    float sea_level = 0.0f;      // [m] above base radius
    glm::vec3 sea_color = glm::vec3(0.1f, 0.1f, 0.8f);
    std::vector<PaletteStop> palette;  // empty => type-based default palette
    float max_height = 1.0f;     // [m] scaled relief above sea level (computed)
    glm::vec3 seed_offset = glm::vec3(0.0f);
    bool bands = false;          // gas giant: smooth sphere, latitude bands
    int band_count = 9;          // stripes pole to pole (odd => bright equator)
    AtmosphereParams atmosphere; // optional rim; enabled => body has air

    COLOUR PaletteColor(float t) const {
        const std::vector<PaletteStop> &s = palette;
        if (s.empty()) {
            return { 1.0f, 1.0f, 1.0f };
        }
        if (t <= s.front().t) {
            return { s.front().color.x, s.front().color.y, s.front().color.z };
        }
        if (t >= s.back().t) {
            return { s.back().color.x, s.back().color.y, s.back().color.z };
        }
        for (size_t i = 1; i < s.size(); i++) {
            if (t <= s[i].t) {
                float f = (t - s[i-1].t) / (s[i].t - s[i-1].t);
                glm::vec3 c = glm::mix(s[i-1].color, s[i].color, f);
                return { c.x, c.y, c.z };
            }
        }
        return { 1.0f, 1.0f, 1.0f };
    }

    // Gas-giant color at unit direction p: latitude runs through a triangle
    // wave so each stripe is dark at its edges, light at its center, sampled
    // through the palette (first stop = dark, last = light).
    COLOUR BandColor(const glm::vec3& p) const {
        float y = glm::clamp(p.y, -1.0f, 1.0f);
        float u = 0.5f + 0.5f * y;           // 0 = south pole, 1 = north
        float x = u * (float)band_count;
        float v = 1.0f - std::fabs(2.0f * (x - std::floor(x)) - 1.0f);
        return PaletteColor(v);
    }
};

// Everything the height/color functions and the grid builder need from a
// body, as VALUES: the async terrain job snapshots one for the worker
// thread, so the worker never reads the (main-thread-owned) TerrainBody.
struct TerrainParams {
    Surface surface;
    float radius;
    COLOUR (*colour_func)(float, float, float);
};

// Fractal Brownian motion (the body's height noise).
inline float terrainNoise3d(const glm::vec3& p, int octaves, float persistence) {
    float sum = 0;
    float strength = 1.0;
    float scale = 2.0;

    for(int i = 0; i < octaves; i++) {
        sum += strength * glm::simplex(p * scale);
        scale *= 2.0;
        strength *= persistence;
    }

    return sum;
}

// Bilinear point on the quad (v0, v1, v2, v3) at (x, y) in [0,1]^2,
// normalized back onto the sphere (the patch's surface patch).
inline glm::vec3 terrainSpherePoint(const glm::vec3& v0, const glm::vec3& v1,
                                    const glm::vec3& v2, const glm::vec3& v3,
                                    const float x, const float y)
{
    // Bilinear on the quad (the standard (1-x)(1-y)v0 + x(1-y)v1 +
    // xy*v2 + (1-x)y*v3 blend), normalized back onto the sphere.
    return glm::normalize(v0 +
                          x * (1.0f - y) * (v1 - v0) +
                          x * y * (v2 - v0) +
                          (1.0f - x) * y * (v3 - v0));
}

inline float terrainScaleHeightNoise(float noise, const TerrainParams& t) {
    constexpr float ref_height = 3000.0; // guess

    // rescale noise by altitude (sign-safe for fractional powers)
    float sign = noise < 0 ? -1.0f : 1.0f;
    float n = sign * noise;
    n *= pow(n / ref_height, t.surface.power);
    return sign * n;
}

// Height (m, from the body center) at a unit direction: the noise, the
// sea floor, and the altitude rescale. Gas giants are a smooth sphere.
inline float terrainHeight(const glm::vec3& p, const TerrainParams& t) {
    const Surface &s = t.surface;
    if(s.bands) {
        return t.radius;   // gas giant: smooth sphere
    }
    float noise = terrainNoise3d(p * s.frequency + s.seed_offset,
                                 s.octaves, s.persistence) * s.amplitude;

    if(s.has_sea && noise < s.sea_level) {
        noise = s.sea_level;
    }

    return t.radius + terrainScaleHeightNoise(noise, t);
}

// Same, without the altitude rescale (the raw relief; the color ramp
// reads this for a smoother gradient).
inline float terrainHeightUnscaled(const glm::vec3& p, const TerrainParams& t) {
    const Surface &s = t.surface;
    float noise = terrainNoise3d(p * s.frequency + s.seed_offset,
                                 s.octaves, s.persistence) * s.amplitude;

    if(s.has_sea && noise < s.sea_level) {
        noise = s.sea_level;
    }

    return t.radius + noise;
}

// The surface color at a unit direction in the body's rotating frame:
// the exact per-vertex color the grid bakes (noise, palette / band, sea,
// contrast), so the 2-D surface map (surfmap.cpp) matches the rendered
// surface pixel for pixel.
inline glm::vec3 terrainSurfaceColor(const glm::vec3& p, const TerrainParams& t) {
    const Surface &s = t.surface;
    if(s.bands) {
        // gas giant: smooth sphere, color by latitude band
        COLOUR cb = s.BandColor(p);
        glm::vec3 color = glm::vec3(cb.r, cb.g, cb.b);
        float brightness = (cb.r + cb.g + cb.b) / 6;
        // lower contrast, increase brightness
        color = float(0.5) * color + glm::vec3(brightness,
                                               brightness,
                                               brightness);
        return color;
    }

    // set the color based on unscaled noise for better gradient
    float height = terrainHeightUnscaled(p, t);

    // add some color noise
    float color_noise = terrainNoise3d(p * t.radius + s.seed_offset, 1, 0.60) * 100;
    float h = height + ((color_noise / 2) - color_noise);

    COLOUR c;
    if(s.palette.empty()) {
        // no palette in the JSON: fall back to the type-based default
        c = (*t.colour_func)(h, t.radius - 1, t.radius + 3000);
    } else {
        float tt = (h - (t.radius + s.sea_level)) / s.max_height;
        c = s.PaletteColor(tt);
    }

    glm::vec3 color = glm::vec3(c.r, c.g, c.b);
    float brightness = (c.r + c.g + c.b) / 6;
    // lower contrast, increase brightness
    color = float(0.5) * color + glm::vec3(brightness,
                                           brightness,
                                           brightness);

    if(s.has_sea && height <= t.radius + s.sea_level) {
        color = s.sea_color;
    }
    return color;
}

// Elevation palette samplers, one per body type (assigned to
// TerrainBody::colour_func by load_system()).
inline COLOUR GetColourMoon(float v, float vmin, float vmax) {
    return { 0.5, 0.5, 0.5 };
}

inline COLOUR GetColourSun(float v, float vmin, float vmax) {
    return { 1.0, 1.0, 0.0 };
}

inline COLOUR GetColourEarth(float v, float vmin, float vmax)
{
    COLOUR c = {1.0,1.0,1.0}; // white
    float dv;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    const int factor = 3;

    if (v < (vmin + 0.25 * dv)) {
        c.r = 0;
        c.g = factor * (v - vmin) / dv;
    } else if (v < (vmin + 0.5 * dv)) {
        c.r = 0;
        c.b = 1 + factor * (vmin + 0.25 * dv - v) / dv;
    } else if (v < (vmin + 0.75 * dv)) {
        c.r = factor * (v - vmin - 0.5 * dv) / dv;
        c.b = 0;
    } else {
        c.g = 1 + 2 * (vmin + 0.75 * dv - v) / dv;
        c.b = 0;
    }

    return(c);
}

// One terrain-grid vertex (the GL-free half of Mesh's PosNorColVertex;
// the GeoPatch ctor converts to Mesh's type when it uploads).
struct TerrVert {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec3 color;

    TerrVert() {}
    TerrVert(const glm::vec3& p, const glm::vec3& n, const glm::vec3& c)
        : pos(p), normal(n), color(c) {}
};

// The grid a GeoPatch draws: size x size terrain vertices (or (size+2)^2
// with the skirt ring) + indices. When num_inner is nonzero, the first
// num_inner indices are the terrain and the tail is the skirt (Mesh::
// DrawSkirt() renders the tail, after the terrain has written depth).
struct GridGeom {
    std::vector<TerrVert> verts;
    std::vector<unsigned int> indices;
    unsigned int num_inner = 0;
};

// The patch grid (pure math; the GeoPatch ctor does the GL upload +
// Bullet collision with the result). The terrain grid is size x size;
// with a skirt it's (size+2)^2, one extra ring of "skirt" vertices
// around it to hide the cracks that open between neighbouring patches at
// different subdivision depths (their edge polylines sample the
// heightfield at different points). Each skirt vertex sits one grid cell
// OUTSIDE the patch boundary, dropped to the patch's lowest terrain
// radius nudged in by 5e-6 (above float precision at any body size, below
// every point on all four edges). Normals/colors are copied from the
// adjacent edge vertex so the skirt shades identically to the terrain
// seam. Technique from Pioneer's GeoPatch; with backface culling on, the
// skirt only rasterises at the limb, exactly where the cracks show.
// Root patches (depth 1) get no skirt: at the ranges they're visible the
// float view-transform noise in fragment depth exceeds the tiny skirt
// depth margin (zipper artefacts), gaps can't open between the six
// equal-depth roots, and a root-vs-child T-junction is masked by the
// child's skirt flaring across the seam.
inline GridGeom buildGridGeom(const TerrainParams& t, bool has_skirt,
                              glm::vec3 p1, glm::vec3 p2, glm::vec3 p3,
                              glm::vec3 p4)
{
    GridGeom geom;
    const int size = 25;
    const int off = has_skirt ? 1 : 0;
    const int edge = size + 2 * off;
    const float frac = 1.0f / (size - 1);
    const float skirt_scale = 0.999995f;

    // sized for the skirted grid (edge == size+2); the skirtless root
    // patches just use the first edge*edge of them
    geom.verts.resize((size_t)edge * (size_t)edge);

    float min_height = t.surface.bands ? t.radius : HUGE_VALF;

    // inner grid at grid coords [off..off+size-1]^2
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            glm::vec3 sphere_p = terrainSpherePoint(p1, p2, p3, p4, i*frac, j*frac);

            // The vertex color (noise, palette / band, sea, contrast):
            // shared with the 2-D surface map (terrainSurfaceColor) so the
            // map matches the rendered surface.
            const glm::vec3 color = terrainSurfaceColor(sphere_p, t);

            if (t.surface.bands) {
                // gas giant: smooth sphere
                geom.verts[(size_t)(j + off) + (size_t)edge * (i + off)] =
                    TerrVert(sphere_p * t.radius, sphere_p, color);
                continue;
            }

            // set the color based on unscaled noise for better gradient
            float height = terrainHeightUnscaled(sphere_p, t);

            // add back scaling
            height = t.radius + terrainScaleHeightNoise(height - t.radius, t);

            min_height = std::min(min_height, height);

            glm::vec3 p = sphere_p * height;

            geom.verts[(size_t)(j + off) + (size_t)edge * (i + off)] =
                TerrVert(p, sphere_p, color);
        }
    }

    // normals: finite differences over the whole inner grid, edge ring
    // included. Stencils that reach past the patch sample the true
    // heightfield one cell outside (it's analytic and shared, so neighbour
    // patches compute matching seam normals and no line shows at the
    // border). The skirt copies the edge normals; its dropped-down
    // vertices never enter a stencil.
    auto terrain_pos = [&](float u, float v) {
        glm::vec3 d = terrainSpherePoint(p1, p2, p3, p4, u, v);
        if (t.surface.bands) {
            return d * t.radius;
        }
        float hgt = terrainHeightUnscaled(d, t);
        hgt = t.radius + terrainScaleHeightNoise(hgt - t.radius, t);
        return d * hgt;
    };
    for (int i = off; i < off + size; i++) {
        for (int j = off; j < off + size; j++) {
            // x along j, y along i: same axes as the old interior-only
            // pass, the cross product sign matters for lighting
            glm::vec3 x1 = (j - 1 >= off) ? geom.verts[(size_t)(j-1) + (size_t)i*edge].pos
                                          : terrain_pos((i - off) * frac, (j - 1 - off) * frac);
            glm::vec3 x2 = (j + 1 < off + size) ? geom.verts[(size_t)(j+1) + (size_t)i*edge].pos
                                                : terrain_pos((i - off) * frac, (j + 1 - off) * frac);
            glm::vec3 y1 = (i - 1 >= off) ? geom.verts[(size_t)j + (size_t)(i-1)*edge].pos
                                          : terrain_pos((i - 1 - off) * frac, (j - off) * frac);
            glm::vec3 y2 = (i + 1 < off + size) ? geom.verts[(size_t)j + (size_t)(i+1)*edge].pos
                                                : terrain_pos((i + 1 - off) * frac, (j - off) * frac);
            glm::vec3 n = glm::normalize(glm::cross(x2-x1, y2-y1));
            geom.verts[(size_t)j + (size_t)edge * (size_t)i].normal = -n;
        }
    }

    // skirt ring: flare one cell past the boundary, down to the patch's
    // lowest radius; copies normal/color from the adjacent edge vertex
    // (after the normal pass above, so it gets the final normals)
    if (has_skirt) {
        const float skirt_r = min_height * skirt_scale;
        auto skirt_vertex = [&](int i, int j, float u, float v, int si, int sj) {
            glm::vec3 sphere_p = terrainSpherePoint(p1, p2, p3, p4, u, v);
            const TerrVert &src = geom.verts[(size_t)sj + (size_t)edge * (size_t)si];
            geom.verts[(size_t)j + (size_t)edge * (size_t)i] =
                TerrVert(sphere_p * skirt_r, src.normal, src.color);
        };
        for (int j = off; j < off + size; j++) {
            skirt_vertex(off - 1, j, -frac, (j - off)*frac, off, j);
            skirt_vertex(off + size, j, 1.0f + frac, (j - off)*frac, off + size - 1, off + size - 1);
        }
        for (int i = off; i < off + size; i++) {
            skirt_vertex(i, off - 1, (i - off)*frac, -frac, i, off);
            skirt_vertex(i, off + size, (i - off)*frac, 1.0f + frac, i, off + size - 1);
        }
        // corners: duplicate the neighbouring skirt vertex
        geom.verts[(size_t)(off - 1) + (size_t)edge * (off - 1)] = geom.verts[(size_t)off + (size_t)edge * (off - 1)];
        geom.verts[(size_t)(off + size) + (size_t)edge * (off - 1)] = geom.verts[(size_t)(off + size - 1) + (size_t)edge * (off - 1)];
        geom.verts[(size_t)(off - 1) + (size_t)edge * (off + size)] = geom.verts[(size_t)off + (size_t)edge * (off + size)];
        geom.verts[(size_t)(off + size) + (size_t)edge * (off + size)] = geom.verts[(size_t)(off + size - 1) + (size_t)edge * (off + size)];
    }

    // inner terrain quads first, then the skirt-ring quads: DrawSkirt()
    // renders only the tail, after the terrain has written depth
    unsigned int i = 0;
    for (int y = off; y < off + size - 1; y++) {
        for (int x = off; x < off + size - 1; x++) {
            geom.indices.push_back((unsigned int)((y + 1) * edge + x + 1));
            geom.indices.push_back((unsigned int)(y * edge + x + 1));
            geom.indices.push_back((unsigned int)(y * edge + x));

            geom.indices.push_back((unsigned int)((y + 1) * edge + x));
            geom.indices.push_back((unsigned int)((y + 1) * edge + x + 1));
            geom.indices.push_back((unsigned int)(y * edge + x));
            i++;   // one quad (6 indices) per step
        }
    }
    geom.num_inner = has_skirt ? i * 6 : 0;
    if (has_skirt) {
        for (int y = 0; y < edge - 1; y++) {
            for (int x = 0; x < edge - 1; x++) {
                if (x >= off && x < off + size - 1 && y >= off && y < off + size - 1) {
                    continue;
                }
                geom.indices.push_back((unsigned int)((y + 1) * edge + x + 1));
                geom.indices.push_back((unsigned int)(y * edge + x + 1));
                geom.indices.push_back((unsigned int)(y * edge + x));

                geom.indices.push_back((unsigned int)((y + 1) * edge + x));
                geom.indices.push_back((unsigned int)((y + 1) * edge + x + 1));
                geom.indices.push_back((unsigned int)(y * edge + x));
            }
        }
    }

    return geom;
}
