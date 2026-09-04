// terragen.h -- terrain generation as pure math (glm + STL only: no GL,
// no Bullet, no game state), so tests/ can pin it without the render
// chain (like surfmap.h) and terrain work can iterate on it in isolation.
//
//   Surface / TerrainParams   per-body terrain + color params (the
//                             "surface" JSON block); value-copyable so a
//                             worker thread can hold a snapshot (the async
//                             subdivision job does -- see terrain.h)
//   terrainHeight(...)        the analytic height function, full detail
//                             (physics, spawning, shadows, surface map)
//   terrainSurfaceColor(...)  the exact per-vertex color the grid bakes
//                             (palette / band, sea, contrast)
//   buildGridGeom(...)        the grid a GeoPatch draws: size x size
//                             terrain vertices + normals + colors, an
//                             optional skirt ring, and the indices
//
// The game-side half (GL upload into a Mesh, Bullet collision, the patch
// tree, LOD) lives in terrain.h / terrain.cpp: the GeoPatch ctor consumes
// a GridGeom on the main thread.
//
// Height model (written from scratch -- the old noise aliased into a
// stepped, voxel-like surface at coarse LOD and its cubic height rescale
// left flat plains with knife-sharp hills):
//
//   relief = amplitude/2.1 * (0.7*continents + 1.4*mask*mountains)
//
//   continents   smooth simplex FBM, octaves 0..5 (noise scales 2..64):
//                planet-wide landmasses down to ~10 km rolling hills on a
//                Kerbin-sized body
//   mountains    a smooth fold (1 - m^2) of a mid-band FBM (octaves 2..):
//                broad rounded crests and gradual flanks -- big and soft
//                on purpose, the shading is normal-based
//   mask         smoothstep of the continents: mountains only rise where
//                the base ground is already high, so lowlands and coasts
//                stay gentle
//
// The noise lives on the UNIT SPHERE (the direction rotated by the
// body's seed, times frequency), so feature sizes grow with the body's
// radius, and the mesh LOD does the same (TerrainBody::max_depth adds
// one level per radius doubling): every body ends up with comparable
// mesh + feature density per surface area.
//
// Band-limited grids: buildGridGeom() fades every octave whose wavelength
// is shorter than ~2 grid cells (the grid's Nyquist limit). A coarse
// patch therefore drops exactly the detail it cannot resolve; sampling it
// anyway is what aliased into the stepped surface near the ground. The
// fade is a continuous weight in the octave index, so neighbouring depths
// agree to within the one partial octave (the skirt covers that seam),
// and a max-depth patch still bakes every octave -- its grid resolves
// them all -- so the visual surface and the analytic height function
// physics uses agree where it matters (landed).

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
struct Surface {
    float amplitude = 2500.0f;   // [m] tallest relief above the base radius
    int octaves = 9;             // noise octaves: continents 0..5, mountains 2..N-1
    float persistence = 0.5f;    // octave amplitude falloff
    float frequency = 1.0f;      // feature-size multiplier (unit-sphere noise)
    bool has_sea = false;
    float sea_level = 0.0f;      // [m] above base radius; floor is flat here
    glm::vec3 sea_color = glm::vec3(0.1f, 0.1f, 0.8f);
    std::vector<PaletteStop> palette;  // empty => type-based default palette
    float max_height = 1.0f;     // [m] highest relief above sea level
                                 // (measured numerically by load_system)
    // Per-body noise orientation (set by load_system from "seed"). A
    // rotation, not an additive offset: adding seed*100 to the sample
    // point pushed the high-octave noise coordinates into the float
    // quantization range, which tiled the surface in lattice-aligned
    // terrace stripes (worst on the high-seed bodies).
    glm::mat3 seed_rot = glm::mat3(1.0f);
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

// ---------------------------------------------------------------------------
// The noise: octaves of 3-D simplex on the unit sphere. Octave i runs at
// noise scale 2^(i+1) (octave 0 ~ 1-radian features), so its wavelength on
// the body is radius / 2^(i+1) -- feature sizes track the body's radius.
// The first `base_octaves` octaves are smooth FBM (the continents/hills);
// the rest are the mountains.
// ---------------------------------------------------------------------------

// glm::simplex ranks its skew-space components with step() comparisons;
// wherever two components tie (exactly axis-aligned directions -- the
// patch face centers, edge midpoints), a 1-ulp input perturbation flips
// the corner ranking and the value jumps ~0.1, i.e. ulp-wide cliffs in
// the heightfield along the symmetric directions. A fixed off-axis
// offset moves the samples away from the ties (the old additive seed
// offset did this by accident; the per-body rotation alone does not).
static const glm::vec3 terrain_noise_off(0.173f, 0.291f, 0.417f);

// One octave loop over [first, last): sum += amp * simplex, amplitudes
// falling by persistence. `fade` band-limits the sum: octave i's weight
// ramps 1 -> 0 as i goes fade-1 -> fade, so callers drop exactly the
// wavelengths a grid cannot resolve. Octaves past the fade are skipped
// outright (weights only ever decrease with i).
inline float terrainFbmOctaves(const glm::vec3& q, int first, int last,
                               float persistence, float fade) {
    float sum = 0.0f, norm = 0.0f, amp = 1.0f;
    glm::vec3 p = q * 2.0f;   // octave 0 scale = 2
    for (int i = 0; i < last; i++, p *= 2.0f, amp *= persistence) {
        if (i < first) { continue; }
        const float w = glm::clamp(fade - (float)i, 0.0f, 1.0f);
        if (w <= 0.0f) { break; }
        sum += amp * w * glm::simplex(p);
        norm += amp;
    }
    return (norm > 0.0f) ? sum / norm : 0.0f;   // [-1, 1]
}

// Signed relief [m] relative to the base radius at unit direction p
// (before the sea-floor clamp): continents in [-0.7, 0.7]*A plus broad
// mountains up to +1.2*A riding the high ground, normalized by 1.9 so
// `amplitude` is the tallest peak. The mountains are a smooth fold
// (1 - m^2) of a mid-band FBM: rounded crests and gradual flanks (the
// shading is normal-based, so knife-edge ridged noise read as spikes),
// and the spectrum stops at ~radius/256, so features stay big and soft.
// `fade` band-limits both noises (see terrainFbmOctaves); pass octaves
// (or more) for full detail.
inline float terrainRelief(const glm::vec3& p, const TerrainParams& t,
                           float fade) {
    const Surface &s = t.surface;
    const glm::vec3 q = (s.seed_rot * p) * s.frequency + terrain_noise_off;
    const int base_octaves = std::min(s.octaves, 6);

    const float continents =
        terrainFbmOctaves(q, 0, base_octaves, s.persistence, fade);
    float h = 0.7f * continents;
    if (s.octaves > 2) {
        const float mask = glm::smoothstep(0.0f, 0.6f, continents);
        const float m = terrainFbmOctaves(q, 2, s.octaves, s.persistence,
                                          fade);
        h += 1.4f * mask * (1.0f - m * m);
    }
    return h * (s.amplitude / 2.1f);
}

// Height (m, from the body center) at a unit direction, band-limited by
// `fade`. Gas giants are a smooth sphere. The sea floor is flat at
// sea_level (that IS the sea: nothing renders below it).
inline float terrainHeightFade(const glm::vec3& p, const TerrainParams& t,
                               float fade) {
    const Surface &s = t.surface;
    if (s.bands) {
        return t.radius;
    }
    float relief = terrainRelief(p, t, fade);
    if (s.has_sea && relief < s.sea_level) {
        relief = s.sea_level;
    }
    return t.radius + relief;
}

// The full-detail height (every octave): what physics, spawning, shadows
// and the surface map query. A max-depth patch bakes this same function
// (its grid resolves every octave), so the walked and the rendered
// surfaces agree where the ship is.
inline float terrainHeight(const glm::vec3& p, const TerrainParams& t) {
    return terrainHeightFade(p, t, (float)t.surface.octaves);
}

// The surface color at a unit direction in the body's rotating frame:
// the exact per-vertex color the grid bakes (palette / band, sea,
// contrast), so the 2-D surface map (surfmap.cpp) matches the rendered
// surface. Colors are evaluated at FULL height detail even on coarse
// grids: the height fade would shift palette bands between LOD depths
// and paint visible seam lines; full-detail color has no such
// discontinuity. The one exception is the color JITTER: it is spatial
// detail, so the grid passes a band-limited noise scale for it (a scale
// finer than the grid cells moires into the dotted lowland pattern seen
// on small bodies); <= 0 means the full meter-scale speckle (the surface
// map).
inline glm::vec3 terrainSurfaceColor(const glm::vec3& p, const TerrainParams& t,
                                     float jitter_scale = -1.0f) {
    const Surface &s = t.surface;
    if (s.bands) {
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

    const float height = terrainHeight(p, t);

    // color jitter so wide palette bands don't look flat
    if (jitter_scale <= 0.0f) {
        jitter_scale = t.radius;
    }
    const float jitter =
        glm::simplex((s.seed_rot * p) * jitter_scale + terrain_noise_off) * 50.0f;
    const float h = height + jitter;

    COLOUR c;
    if (s.palette.empty()) {
        // no palette in the JSON: fall back to the type-based default
        c = (*t.colour_func)(h, t.radius - 1, t.radius + 3000);
    } else {
        const float tt = (h - (t.radius + s.sea_level)) / s.max_height;
        c = s.PaletteColor(tt);
    }

    glm::vec3 color = glm::vec3(c.r, c.g, c.b);
    float brightness = (c.r + c.g + c.b) / 6;
    // lower contrast, increase brightness
    color = float(0.5) * color + glm::vec3(brightness,
                                           brightness,
                                           brightness);

    if (s.has_sea && height <= t.radius + s.sea_level) {
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

// ---------------------------------------------------------------------------
// The patch grid
// ---------------------------------------------------------------------------

// Bilinear point on the quad (v0, v1, v2, v3) at (x, y) in [0,1]^2,
// normalized back onto the sphere (the patch's surface patch). Tolerates
// x/y slightly outside [0,1] (the normal stencil reaches one cell past
// the boundary; the heightfield there is the same analytic function the
// neighbour patch bakes, so seam normals match).
inline glm::vec3 terrainSpherePoint(const glm::vec3& v0, const glm::vec3& v1,
                                    const glm::vec3& v2, const glm::vec3& v3,
                                    const float x, const float y)
{
    return glm::normalize(v0 +
                          x * (1.0f - y) * (v1 - v0) +
                          x * y * (v2 - v0) +
                          (1.0f - x) * y * (v3 - v0));
}

// The octave fade for one grid cell: octave i's wavelength on the unit
// sphere is ~1/2^(i+1) (times the frequency multiplier); keep wavelengths
// >= 2 cells (Nyquist) -> i <= log2(1/(2*cell*frequency)) - 1, with the
// boundary octave partially weighted. A zero/tiny cell (deep patches,
// where the corner dots below round to 1) means "resolve everything".
inline float terrainGridFade(float cell_angle, float frequency) {
    if (!(cell_angle > 0.0f) || !std::isfinite(cell_angle)) {
        return 1e30f;
    }
    return std::log2(1.0f / (2.0f * cell_angle * frequency)) - 1.0f;
}

// The fade for every patch at a subdivision depth. Heights must be a
// pure function of (position, depth) -- like Pioneer's terrain, which
// samples one deterministic heightfield at every LOD -- or neighbouring
// patches disagree where they share an edge. A per-patch fade (from the
// actual corner angles, which vary between sibling quads) painted seam
// lines along same-depth boundaries, so the cell angle is the nominal
// one for the depth: the root cube-face edge (acos 1/3) halved per level.
inline float terrainDepthFade(int depth, int grid_size, float frequency) {
    const float root_angle = 1.2310f;   // acos(1/3), root cube-face edge
    const float cell = root_angle / (float)(1 << (depth - 1))
                       / (float)(grid_size - 1);
    return terrainGridFade(cell, frequency);
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
                              int depth, glm::vec3 p1, glm::vec3 p2,
                              glm::vec3 p3, glm::vec3 p4)
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

    // Band-limit the heightfield to this grid (see terrainGridFade): the
    // fade is the same on every patch at this depth, so seam heights and
    // normals agree between same-depth neighbours. The color jitter gets
    // the same treatment (4 cells here), capped at a 16 m wavelength so
    // even max-depth grids stay above their cell size.
    const float fade = terrainDepthFade(depth, size, t.surface.frequency);
    const float jitter_scale =
        std::min(t.radius / 16.0f, std::pow(2.0f, fade - 2.0f));
    auto height_at = [&](const glm::vec3 &d) {
        return terrainHeightFade(d, t, fade);
    };

    float min_height = HUGE_VALF;

    // inner grid at grid coords [off..off+size-1]^2
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            const glm::vec3 d = terrainSpherePoint(p1, p2, p3, p4,
                                                   i*frac, j*frac);
            const float height = height_at(d);
            min_height = std::min(min_height, height);

            // The vertex color (palette / band, sea, contrast), with the
            // jitter band-limited to this grid; the 2-D surface map uses
            // the same function at full speckle.
            const glm::vec3 color = terrainSurfaceColor(d, t, jitter_scale);
            geom.verts[(size_t)(j + off) + (size_t)edge * (i + off)] =
                TerrVert(d * height, d, color);
        }
    }

    // normals: central differences over the whole inner grid. Stencils
    // that reach past the patch sample the (band-limited) heightfield one
    // cell outside -- it's analytic and shared, and same-depth neighbours
    // use the same fade, so seam normals match and no line shows at the
    // border. The skirt copies the edge normals; its dropped-down
    // vertices never enter a stencil.
    auto pos_at = [&](float u, float v) {
        const glm::vec3 d = terrainSpherePoint(p1, p2, p3, p4, u, v);
        return d * height_at(d);
    };
    for (int i = off; i < off + size; i++) {
        for (int j = off; j < off + size; j++) {
            // x along j, y along i: the cross product sign matters for
            // lighting
            const glm::vec3 x1 = (j - 1 >= off)
                ? geom.verts[(size_t)(j-1) + (size_t)i*edge].pos
                : pos_at((i - off) * frac, (j - 1 - off) * frac);
            const glm::vec3 x2 = (j + 1 < off + size)
                ? geom.verts[(size_t)(j+1) + (size_t)i*edge].pos
                : pos_at((i - off) * frac, (j + 1 - off) * frac);
            const glm::vec3 y1 = (i - 1 >= off)
                ? geom.verts[(size_t)j + (size_t)(i-1)*edge].pos
                : pos_at((i - 1 - off) * frac, (j - off) * frac);
            const glm::vec3 y2 = (i + 1 < off + size)
                ? geom.verts[(size_t)j + (size_t)(i+1)*edge].pos
                : pos_at((i + 1 - off) * frac, (j - off) * frac);
            const glm::vec3 n = glm::normalize(glm::cross(x2-x1, y2-y1));
            geom.verts[(size_t)j + (size_t)edge * (size_t)i].normal = -n;
        }
    }

    // skirt ring: flare one cell past the boundary, down to the patch's
    // lowest radius; copies normal/color from the adjacent edge vertex
    // (after the normal pass above, so it gets the final normals)
    if (has_skirt) {
        const float skirt_r = min_height * skirt_scale;
        auto skirt_vertex = [&](int i, int j, float u, float v, int si, int sj) {
            const glm::vec3 d = terrainSpherePoint(p1, p2, p3, p4, u, v);
            const TerrVert &src = geom.verts[(size_t)sj + (size_t)edge * (size_t)si];
            geom.verts[(size_t)j + (size_t)edge * (size_t)i] =
                TerrVert(d * skirt_r, src.normal, src.color);
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
