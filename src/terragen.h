// terragen.h -- terrain generation as pure math (glm + STL only: no GL, no Bullet,
// no game state), so tests can pin it without the render chain. The game-side half
// (GL upload, collision, the patch tree) is terrain.h.
//
// Height model: relief = amplitude/2.1 * (0.7*continents + 1.4*mask*mountains),
// sampled on the UNIT SPHERE so feature sizes grow with the body's radius.
// buildGridGeom() band-limits octaves shorter than ~2 grid cells (Nyquist),
// identically for every patch at a depth, so shared edges agree.

#pragma once

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/noise.hpp>

#include "constants.h"  // kAtmoScaleHeights
#include "equirect.h"   // equirectLonLat / equirectPixel

typedef struct {
    float r, g, b;
} COLOUR;

// One stop on a body's land-color ramp: elevation fraction t in [0,1].
// Fields initialized (#230): an unset color is read per vertex.
struct PaletteStop {
    float t = 0.0f;
    glm::vec3 color = glm::vec3(0.0f);
};

/* How a body's sea renders (system JSON "surface.ocean"; default "mesh").
   Mesh: true relief everywhere, a transparent shell sphere at sea_level paints
   the water. Flat: the terrain is clamped to sea_level and painted sea_color --
   scale-proof, so a big body opts out of the shell (its flat faces sit well below
   the sea-level sphere, so a shallow sea floor would stick out). DrawOcean
   depth-tests but never depth-writes: the floor reads through by design. */
enum class OceanMode : unsigned char {
    Mesh,
    Flat,
};

// A switch, not a ternary: a new mode must not silently report "mesh".
inline const char *oceanModeName(OceanMode m) {
    switch(m) {
    case OceanMode::Mesh: return "mesh";
    case OceanMode::Flat: return "flat";
    }
    assert(!"ocean mode has no name");
    return "?";
}

// Throws on an unrecognised value: a typo must not silently swap the renderer.
inline OceanMode parseOceanMode(const std::string &v, const std::string &body) {
    if(v == "mesh") return OceanMode::Mesh;
    if(v == "flat") return OceanMode::Flat;
    throw std::runtime_error("system: '" + body + "': unknown \"ocean\" mode \""
                             + v + "\" (expected \"mesh\" or \"flat\")");
}

/* Per-body atmosphere (optional "surface.atmosphere"), in two independent halves:
   the RENDER half (a Fresnel limb-glow shell) and the PHYSICAL half that src/drag.h
   integrates. A body may have either alone. */
struct AtmosphereParams {
    bool enabled = false;
    glm::vec3 color = glm::vec3(0.3f, 0.5f, 1.0f);  // rim tint (N2/O2 blue)
    float thickness = 0.0f;  // RENDER ONLY: rim above the highest ground
    float power = 3.0f;       // Fresnel falloff (higher = tighter rim)
    float intensity = 1.0f;   // overall alpha scale
    /* The physical half (src/drag.h): rho(alt) = sea_level_density *
       exp(-alt / scale_height), cut to zero at top(); 0 density = no drag. */
    double sea_level_density = 0.0;  // kg/m^3 at the surface; 0 = no drag
    double scale_height = 0.0;       // [m]; the density /e-fold altitude
    double height = 0.0;  // hard top [m above sea level]; authored, 0 = top() derives

    /* Authored height, else scale_height * kAtmoScaleHeights. One home so drag and
       "is this in space?" cannot disagree. */
    double top() const {
        if(height > 0.0) { return height; }
        return scale_height > 0.0 ? scale_height * kAtmoScaleHeights : 0.0;
    }
};

/* Per-body cloud deck (optional "surface.clouds"): one shell at `height` above the
   highest terrain. Coverage is a seeded FBM (cloudCover), baked at load into an
   equirectangular map the shader fetches. */
struct CloudParams {
    bool enabled = false;
    float height = 2500.0f;   // [m] above the highest ground (datum + max_height)
    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f);  // cloud tint (white)
    float coverage = 0.6f;    // 0..1: how much of the deck is cloudy
    float freq = 10.0f;       // pattern scale (unit-sphere noise coords)
    float drift = 0.0f;       // [pattern units/s] wind: pattern creep vs ground
};

/* One ring band ("surface.rings"): a flat annulus in the equatorial plane.
   inner/outer [m] from the body centre; thickness [m] is visual-only. */
struct RingParams {
    std::string name;
    double inner = 0.0;    // [m] from body centre
    double outer = 0.0;    // [m] from body centre
    double thickness = 0.0;// [m] band width
    float albedo = 0.5f;   // band brightness
    float opacity = 0.8f;  // 0..1 band transparency
};

/* Authored equirectangular elevation (metres vs sea level), shared across
   TerrainParams snapshots and worker threads (ptr copied by value, samples
   immutable). Layout is equirect.h: lon 0 at the LEFT edge, north up.
   Absent = procedural FBM. */
struct Heightmap {
    int w = 0, h = 0;
    std::vector<int16_t> m;   // metres vs sea level, row-major, row 0 = north

    // Exact sample range (for max_height / palette); cached by recomputeRange().
    float minMetres() const { return range_.first; }
    float maxMetres() const { return range_.second; }
    void recomputeRange() const {
        if(m.empty()) { range_ = { 0.0f, 0.0f }; return; }
        const auto mm = std::minmax_element(m.begin(), m.end());
        range_ = { (float)*mm.first, (float)*mm.second };
    }

    // Bilinear sample at a unit direction in the body's ROTATING frame.
    // Longitude wraps, latitude clamps at the poles. Does NOT apply seed_rot.
    float sample(const glm::vec3 &p) const {
        assert(w >= 2 && h >= 2 && (int)m.size() == w * h);
        if(w < 2 || h < 2) { return 0.0f; }
        /* equirectLonLat does not normalize, so a non-finite p makes (int)u /
           (int)v undefined -- memory unsafety, not a wrong height. Asserts are
           live in every config, release included. */
        assert(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)
               && p.x * p.x + p.y * p.y + p.z * p.z > 0.0f);
        float lon, lat, u, v;
        equirectLonLat(p, lon, lat);
        equirectPixel(lon, lat, w, h, u, v);
        v = glm::clamp(v, 0.0f, (float)h - 1.000001f);
        const int i0 = ((int)u % w + w) % w;
        const int j0 = std::min((int)v, h - 2);
        const float fu = u - std::floor(u);
        const float fv = v - (float)j0;
        const int i1 = (i0 + 1) % w;
        const float a = (float)m[(size_t)j0 * w + i0] * (1.0f - fu)
                      + (float)m[(size_t)j0 * w + i1] * fu;
        const float b = (float)m[(size_t)(j0 + 1) * w + i0] * (1.0f - fu)
                      + (float)m[(size_t)(j0 + 1) * w + i1] * fu;
        return a * (1.0f - fv) + b * fv;
    }

private:
    // Filled by recomputeRange().
    mutable std::pair<float, float> range_ = { 0.0f, 0.0f };
};

// Throws on a bad file: a named heightmap is data, no silent fallback to noise.
inline std::shared_ptr<const Heightmap> loadHeightmap(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    if(!f.is_open()) {
        throw std::runtime_error("heightmap: cannot open " + path
                                 + " (bake with utils/heightmaps/gen_earth_hm.py)");
    }
    char magic[4];
    uint32_t w = 0, h = 0, fmt = 0;
    f.read(magic, 4);
    f.read(reinterpret_cast<char *>(&w), 4);
    f.read(reinterpret_cast<char *>(&h), 4);
    f.read(reinterpret_cast<char *>(&fmt), 4);
    if(!f || std::memcmp(magic, "HM16", 4) != 0) {
        throw std::runtime_error("heightmap: " + path + " is not an HM16 file");
    }
    if(fmt != 1) {
        throw std::runtime_error("heightmap: " + path
                                 + ": unknown format " + std::to_string(fmt));
    }
    if(w < 2 || h < 2 || w > 65536 || h > 65536) {
        throw std::runtime_error("heightmap: " + path + ": bad size");
    }
    auto hm = std::make_shared<Heightmap>();
    hm->w = (int)w;
    hm->h = (int)h;
    hm->m.resize((size_t)w * (size_t)h);
    f.read(reinterpret_cast<char *>(hm->m.data()),
           (std::streamsize)(hm->m.size() * sizeof(int16_t)));
    if(!f) {
        throw std::runtime_error("heightmap: " + path + ": truncated samples");
    }
    hm->recomputeRange();
    return hm;
}

// Per-body terrain + color parameters (the optional "surface" JSON block).
struct Surface {
    float amplitude = 2500.0f;   // [m] tallest relief above the base radius
    int octaves = 9;             // noise octaves: continents 0..5, mountains 2..N-1
    float persistence = 0.5f;    // octave amplitude falloff
    float frequency = 1.0f;      // feature-size multiplier (unit-sphere noise)
    /* Authored macro relief ("surface.heightmap"): replaces the continents FBM;
       `detail_amplitude` [m] is the fine residual on top. No mountain fold. */
    std::shared_ptr<const Heightmap> heightmap;
    float detail_amplitude = 250.0f;
    bool has_sea = false;
    float sea_level = 0.0f;      // [m] above base radius; floor is flat here
    glm::vec3 sea_color = glm::vec3(0.1f, 0.1f, 0.8f);
    OceanMode ocean_mode = OceanMode::Mesh;  // how the sea renders (see above)
    std::vector<PaletteStop> palette;  // empty => type-based default palette
    float max_height = 1.0f;     // [m] highest relief above sea level,
                                 // measured by TerrainBody::BuildRootGeoms
    /* Per-body noise orientation (load_system from "seed"). A rotation, not an
       additive offset: offsetting the sample point tiles the surface in lattice
       stripes (float quantization at high octaves). */
    glm::mat3 seed_rot = glm::mat3(1.0f);
    bool bands = false;          // gas giant: smooth sphere, latitude bands
    int band_count = 9;          // stripes pole to pole (odd => bright equator)
    AtmosphereParams atmosphere; // optional rim; enabled => the JSON block exists
    CloudParams clouds;          // optional deck; enabled => body has clouds
    std::vector<RingParams> rings;  // optional; flat annuli in the equator

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

    // Gas-giant color at unit direction p: a triangle wave through the
    // palette (first stop = dark, last = light).
    COLOUR BandColor(const glm::vec3& p) const {
        float y = glm::clamp(p.y, -1.0f, 1.0f);
        float u = 0.5f + 0.5f * y;           // 0 = south pole, 1 = north
        float x = u * (float)band_count;
        float v = 1.0f - std::fabs(2.0f * (x - std::floor(x)) - 1.0f);
        return PaletteColor(v);
    }
};

/* Cloud deck coverage: a port of the deck shader's FBM (value noise, 4 octaves).
   DIRECTION ONLY, so the equirectangular bake has no seam. */
inline float cloudFract(float x) { return x - std::floor(x); }

inline float cloudHash13(const glm::vec3 &q) {
    glm::vec3 p = q * 0.3183099f;
    p = glm::vec3(cloudFract(p.x), cloudFract(p.y), cloudFract(p.z));
    const float d = p.x * (p.z + 19.19f) + p.y * (p.y + 19.19f)
                  + p.z * (p.x + 19.19f);
    return cloudFract((p.x + p.y + 2.0f * d) * (p.z + d));
}

inline float cloudValueNoise(const glm::vec3 &q) {
    const glm::vec3 i = glm::floor(q);
    glm::vec3 f = q - i;
    f = f * f * (3.0f - 2.0f * f);
    const float a = cloudHash13(i + glm::vec3(0.0f, 0.0f, 0.0f));
    const float b = cloudHash13(i + glm::vec3(1.0f, 0.0f, 0.0f));
    const float c = cloudHash13(i + glm::vec3(0.0f, 1.0f, 0.0f));
    const float d = cloudHash13(i + glm::vec3(1.0f, 1.0f, 0.0f));
    const float e = cloudHash13(i + glm::vec3(0.0f, 0.0f, 1.0f));
    const float g = cloudHash13(i + glm::vec3(1.0f, 0.0f, 1.0f));
    const float h = cloudHash13(i + glm::vec3(0.0f, 1.0f, 1.0f));
    const float n = cloudHash13(i + glm::vec3(1.0f, 1.0f, 1.0f));
    const float ab = a + (b - a) * f.x, cd = c + (d - c) * f.x;
    const float ag = e + (g - e) * f.x, hn = h + (n - h) * f.x;
    const float xy = ab + (cd - ab) * f.y, zn = ag + (hn - ag) * f.y;
    return xy + (zn - xy) * f.z;
}

inline float cloudFbm(const glm::vec3 &q) {
    float v = 0.0f, amp = 0.5f;
    glm::vec3 p = q;
    for (int i = 0; i < 4; i++) {
        v += amp * cloudValueNoise(p);
        p = p * 2.03f + glm::vec3(11.7f, 7.3f, 5.9f);
        amp *= 0.5f;
    }
    return v;
}

// Deck coverage at a unit direction (body frame): 0 = clear, 1 = cloud.
// The threshold slides with `coverage`.
inline float cloudCover(const glm::vec3 &dir, const glm::mat3 &seedRot,
                        const CloudParams &c) {
    const float n = cloudFbm(seedRot * (dir * c.freq));
    const float T = 0.78f + (0.38f - 0.78f) * c.coverage;
    const float x = (n - (T - 0.12f)) / 0.24f;   // the T +/- 0.12 smoothstep
    const float t = glm::clamp(x, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* Everything the height/color functions and the grid builder need from a body, as
   VALUES: the async terrain job snapshots one for the worker. Members are
   initialized so a hand-built one reads 0/nullptr, not stack garbage (#227). */
struct TerrainParams {
    Surface surface;
    float radius = 0.0f;
    COLOUR (*colour_func)(float, float, float) = nullptr;
    /* The mean-sea sphere [m] (#404): the datum every altitude in the game is
       measured from, so relief/sea/palette math and orbit apsides agree. Doubles
       because the ADD is not float math: radius and sea_level are floats and a
       planetary sum would drop the metres we print. TerrainBody has the twin
       accessor over its own live fields -- bodies read that one, worker math
       read this one (the snapshot is empty until refreshParamsCache()). */
    double seaLevelRadius() const { return (double)radius + (double)surface.sea_level; }
};

// ---------------------------------------------------------------------------
// The noise: octaves of 3-D simplex on the unit sphere. Octave i runs at noise
// scale 2^(i+1); the first `base_octaves` are continents, the rest mountains.
// ---------------------------------------------------------------------------

/* glm::simplex ranks its skew-space components with step(): exactly axis-aligned
   inputs tie, and a 1-ulp perturbation then flips the corner ranking (a value
   jump). A fixed off-axis offset avoids the ties. */
static const glm::vec3 terrain_noise_off(0.173f, 0.291f, 0.417f);

// One octave loop over [first, last): amplitudes fall by persistence; `fade` ramps
// octave i's weight 1 -> 0 as i goes fade-1 -> fade.
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

// Signed relief [m] relative to the base radius at unit direction p (before the
// sea-floor clamp). Heightmap body: authored metres + a fine residual, no fold.
inline float terrainRelief(const glm::vec3& p, const TerrainParams& t,
                           float fade) {
    const Surface &s = t.surface;
    const glm::vec3 q = (s.seed_rot * p) * s.frequency + terrain_noise_off;

    if (s.heightmap) {
        float h = s.heightmap->sample(p);
        if (s.detail_amplitude > 0.0f && s.octaves > 2) {
            h += terrainFbmOctaves(q, 2, s.octaves, s.persistence, fade)
               * s.detail_amplitude;
        }
        return h;
    }

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

/* Height (m, from the body center) at a unit direction, band-limited by `fade`.
   A Flat-sea body clamps its sea floor to sea_level: the clamp IS the sea, so
   physics, spawning and rendering agree the water is solid there. A Mesh-sea body
   keeps its true relief below the shell. */
inline float terrainHeightFade(const glm::vec3& p, const TerrainParams& t,
                               float fade) {
    const Surface &s = t.surface;
    if (s.bands) {
        return t.radius;
    }
    float relief = terrainRelief(p, t, fade);
    if (s.ocean_mode == OceanMode::Flat && s.has_sea && relief < s.sea_level) {
        relief = s.sea_level;
    }
    return t.radius + relief;
}

/* The full-detail height (every octave): what physics, spawning, shadows and the
   surface map query. A max-depth patch bakes the same function, so the walked and
   rendered surfaces agree. */
inline float terrainHeight(const glm::vec3& p, const TerrainParams& t) {
    return terrainHeightFade(p, t, (float)t.surface.octaves);
}

/* The surface color at a unit direction in the body's rotating frame: the exact
   per-vertex color the grid bakes, so the 2-D surface map matches the render.
   Height is FULL detail even on coarse grids (a fade would shift palette bands
   between LOD depths); only the color JITTER is band-limited, being spatial
   detail (jitter_scale <= 0 = the full speckle, i.e. the surface map). */
inline glm::vec3 terrainSurfaceColor(const glm::vec3& p, const TerrainParams& t,
                                     float jitter_scale = -1.0f) {
    const Surface &s = t.surface;
    if (s.bands) {
        COLOUR cb = s.BandColor(p);
        glm::vec3 color = glm::vec3(cb.r, cb.g, cb.b);
        float brightness = (cb.r + cb.g + cb.b) / 6;
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
        // no palette: the type-based default from load_system
        assert(t.colour_func != nullptr);
        c = (*t.colour_func)(h, t.radius - 1, t.radius + 3000);
    } else {
        const float sea_r = (float)t.seaLevelRadius();
        const float tt = (h - sea_r) / s.max_height;
        c = s.PaletteColor(tt);
    }

    glm::vec3 color = glm::vec3(c.r, c.g, c.b);
    float brightness = (c.r + c.g + c.b) / 6;
    color = float(0.5) * color + glm::vec3(brightness,
                                           brightness,
                                           brightness);

    /* A Flat-sea body paints its own sea. Decided at FULL height detail while the
       height clamp runs at the grid's band-limited detail, so with a non-zero
       sea_level a coarse patch gets a sea-level plain in its inland palette (#193). */
    if (s.ocean_mode == OceanMode::Flat && s.has_sea
        && height <= (float)t.seaLevelRadius()) {
        color = s.sea_color;
    }
    return color;
}

// ---------------------------------------------------------------------------
// Biomes: altitude above sea level relative to the body's measured max_height.
// SOLID bodies only (#52). max_height is MEASURED (1.0 until the heavy phase
// lands), so classify only a body you know is ready (#54).
// ---------------------------------------------------------------------------

enum class Biome : unsigned char {
    None,      // banded gas giant: no solid surface to stand on
    Ocean,
    Lowland,
    Midlands,
    Mountain,
};

// Altitude above SEA level [m] (negative = below). Dry bodies have no ocean;
// max_height <= 0 is all lowland, else every point would be a mountain.
inline Biome biomeFromAltitude(double alt, const Surface &s) {
    if(s.bands) { return Biome::None; }
    if(s.has_sea && alt <= 0.0) { return Biome::Ocean; }
    const double mh = (double)s.max_height;
    if(mh <= 0.0) { return Biome::Lowland; }
    if(alt >= 0.8 * mh) { return Biome::Mountain; }
    if(alt >= 0.5 * mh) { return Biome::Midlands; }
    return Biome::Lowland;
}

// Biome at a unit direction in the body's rotating frame (full-detail height).
inline Biome biomeAt(const glm::vec3 &p, const TerrainParams &t) {
    const double alt = (double)terrainHeight(p, t) - t.seaLevelRadius();
    return biomeFromAltitude(alt, t.surface);
}

// The biome's display name (the UI readouts).
inline const char *biomeName(Biome b) {
    switch(b) {
    case Biome::Ocean:    return "ocean";
    case Biome::Lowland:  return "lowlands";
    case Biome::Midlands: return "midlands";
    case Biome::Mountain: return "mountains";
    default:              return "none";
    }
}

// Elevation palette samplers, one per body type (load_system assigns colour_func).
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

// Bilinear point on the quad (v0, v1, v2, v3) at (x, y) in [0,1]^2, normalized
// onto the sphere. Tolerates x/y slightly outside [0,1]: the heightfield there is
// the same analytic function the neighbour patch bakes, so seam normals match.
inline glm::vec3 terrainSpherePoint(const glm::vec3& v0, const glm::vec3& v1,
                                    const glm::vec3& v2, const glm::vec3& v3,
                                    const float x, const float y)
{
    return glm::normalize(v0 +
                          x * (1.0f - y) * (v1 - v0) +
                          x * y * (v2 - v0) +
                          (1.0f - x) * y * (v3 - v0));
}

// The octave fade for one grid cell: keep wavelengths >= 2 cells (Nyquist),
// boundary octave partially weighted. A zero/tiny cell = "resolve everything".
inline float terrainGridFade(float cell_angle, float frequency) {
    if (!(cell_angle > 0.0f) || !std::isfinite(cell_angle)) {
        return 1e30f;
    }
    return std::log2(1.0f / (2.0f * cell_angle * frequency)) - 1.0f;
}

/* The fade for every patch at a subdivision depth. Heights must be a pure function
   of (position, depth) or neighbours disagree on a shared edge (a per-patch fade
   painted seam lines), hence the nominal cell angle: the root cube-face edge
   (acos 1/3) halved per level. */
inline float terrainDepthFade(int depth, int grid_size, float frequency) {
    const float root_angle = 1.2310f;   // acos(1/3), root cube-face edge
    const float cell = root_angle / (float)(1 << (depth - 1))
                       / (float)(grid_size - 1);
    return terrainGridFade(cell, frequency);
}

// One terrain-grid vertex (the GL-free half of Mesh's PosNorColVertex).
struct TerrVert {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec3 color;

    TerrVert() {}
    TerrVert(const glm::vec3& p, const glm::vec3& n, const glm::vec3& c)
        : pos(p), normal(n), color(c) {}
};

/* The grid a GeoPatch draws: size x size terrain vertices (or (size+2)^2 with the
   skirt ring) + indices. num_inner != 0 => the first num_inner indices are the
   terrain, the tail the skirt.
   Positions are ANCHOR-RELATIVE: `anchor` (body-frame, DOUBLE, the patch centroid
   at the band-limited height) is subtracted from every baked pos and added back
   in double only. A body-centred float32 vertex rounds at ULP(radius), which
   jittered the terrain with every camera move. */
struct GridGeom {
    std::vector<TerrVert> verts;
    std::vector<unsigned int> indices;
    unsigned int num_inner = 0;
    glm::dvec3 anchor = glm::dvec3(0.0);   // body-frame [m]; verts are relative
};

/* The patch grid (pure math; the GeoPatch ctor does the GL upload + Bullet
   collision). The skirt ring is one grid cell OUTSIDE the patch boundary, hiding
   cracks between neighbours at different subdivision depths; with backface culling
   it only rasterises at the limb. Every patch gets one, roots included. */
inline GridGeom buildGridGeom(const TerrainParams& t, bool has_skirt,
                              int depth, glm::vec3 p1, glm::vec3 p2,
                              glm::vec3 p3, glm::vec3 p4)
{
    GridGeom geom;
    // Grid size sets the on-screen triangle density; the patch COUNT comes from the
    // --terrain-px LOD budget.
    const int size = 49;
    const int off = has_skirt ? 1 : 0;
    const int edge = size + 2 * off;
    const float frac = 1.0f / (size - 1);

    // Sized for the skirted grid; a skirtless caller uses the first edge*edge.
    geom.verts.resize((size_t)edge * (size_t)edge);
    // Exact reserve: no geometric growth on the worker thread.
    geom.indices.reserve((size_t)(has_skirt ? edge - 1 : size - 1)
                         * (size_t)(has_skirt ? edge - 1 : size - 1) * 6);

    /* Band-limit the heightfield to this grid: the fade is the same on every patch
       at this depth, so seam heights and normals agree. Same for the color jitter. */
    const float fade = terrainDepthFade(depth, size, t.surface.frequency);
    const float jitter_scale =
        std::min(t.radius / 16.0f, std::pow(2.0f, fade - 2.0f));
    auto height_at = [&](const glm::vec3 &d) {
        return terrainHeightFade(d, t, fade);
    };

    // The patch anchor (see GridGeom): quad centroid at the band-limited height.
    const glm::dvec3 cdir = glm::normalize(glm::dvec3(p1) + glm::dvec3(p2)
                                         + glm::dvec3(p3) + glm::dvec3(p4));
    const glm::dvec3 anchor = cdir * (double)height_at(glm::vec3(cdir));
    geom.anchor = anchor;
    auto anchored = [&](const glm::vec3 &d, float h) {
        return glm::vec3(glm::dvec3(d) * (double)h - anchor);
    };

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            const glm::vec3 d = terrainSpherePoint(p1, p2, p3, p4,
                                                   i*frac, j*frac);
            const float height = height_at(d);

            // Vertex color (palette / band / sea / contrast), jitter band-limited.
            const glm::vec3 color = terrainSurfaceColor(d, t, jitter_scale);
            geom.verts[(size_t)(j + off) + (size_t)edge * (i + off)] =
                TerrVert(anchored(d, height), d, color);
        }
    }

    /* Normals: central differences. Stencils past the patch sample the band-limited
       heightfield one cell outside -- same-depth neighbours use the same fade, so
       seam normals match. The skirt copies the edge normals. */
    auto pos_at = [&](float u, float v) {
        const glm::vec3 d = terrainSpherePoint(p1, p2, p3, p4, u, v);
        return anchored(d, height_at(d));
    };
    for (int i = off; i < off + size; i++) {
        for (int j = off; j < off + size; j++) {
            // x along j, y along i: the cross product sign matters for lighting
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

    /* Skirt ring: a 45° wall from the patch boundary. Both legs are one grid cell
       of the patch edge (a relief-based length lost that scaling); the flare hides
       under the neighbouring surface. Normal/colour copy the adjacent edge vertex,
       hence this runs after the normal pass. */
    if (has_skirt) {
        const float edge_angle = std::acos(glm::clamp(glm::dot(p1, p2), -1.0f, 1.0f));
        auto skirt_vertex = [&](int i, int j, float u, float v, int si, int sj) {
            const TerrVert &src = geom.verts[(size_t)sj + (size_t)edge * (size_t)si];
            // Edge direction + radius recomputed analytically: src.pos is
            // anchor-relative, so its length is no longer the radius.
            const glm::vec3 d_edge = terrainSpherePoint(p1, p2, p3, p4,
                                                        (si - off) * frac,
                                                        (sj - off) * frac);
            const float h_edge = height_at(d_edge);
            // Outward edge tangent: one cell past the boundary, radial removed.
            const glm::vec3 d_out = terrainSpherePoint(p1, p2, p3, p4, u, v);
            glm::vec3 tang = d_out - d_edge;
            tang -= d_edge * glm::dot(tang, d_edge);
            tang = glm::normalize(tang);
            const float one_cell = edge_angle * h_edge / (float)(size - 1);
            const float flare = one_cell;   // patch-proportional length
            const float drop = flare;       // 45° wall
            const glm::vec3 pos = anchored(d_edge, h_edge)
                                + flare * tang
                                - drop * d_edge;
            geom.verts[(size_t)j + (size_t)edge * (size_t)i] =
                TerrVert(pos, src.normal, src.color);
        };
        for (int j = off; j < off + size; j++) {
            skirt_vertex(off - 1, j, -frac, (j - off)*frac, off, j);
            skirt_vertex(off + size, j, 1.0f + frac, (j - off)*frac, off + size - 1, j);
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

    // Inner terrain quads first, then the skirt-ring quads: DrawSkirt() renders
    // only the tail, after the terrain has written depth.
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

// ---------------------------------------------------------------------------
// The patch tree: subdivision geometry + the LOD measure (pure math, so tests can
// pin the numbers every subdivide/collapse decision is made from).
// ---------------------------------------------------------------------------

// The four children's corner quads: edge midpoints + the shared center cn,
// renormalized onto the unit sphere.
inline void subdivideCorners(const glm::vec3 &v0, const glm::vec3 &v1,
                             const glm::vec3 &v2, const glm::vec3 &v3,
                             glm::vec3 quad[4][4]) {
    const glm::vec3 v01 = glm::normalize(v0+v1);
    const glm::vec3 v12 = glm::normalize(v1+v2);
    const glm::vec3 v23 = glm::normalize(v2+v3);
    const glm::vec3 v30 = glm::normalize(v3+v0);
    const glm::vec3 cn  = glm::normalize(v0+v1+v2+v3);

    quad[0][0] = v0;  quad[0][1] = v01; quad[0][2] = cn;  quad[0][3] = v30;
    quad[1][0] = v01; quad[1][1] = v1;  quad[1][2] = v12; quad[1][3] = cn;
    quad[2][0] = cn;  quad[2][1] = v12; quad[2][2] = v2;  quad[2][3] = v23;
    quad[3][0] = v30; quad[3][1] = cn;  quad[3][2] = v23; quad[3][3] = v3;
}

/* A patch's characteristic size on the UNIT sphere (x radius = metres): the mean
   of its four edge chords. Corner-symmetric because the midpoint scheme leaves
   unequal edges, which biased the LOD threshold between siblings. */
inline double patchWidthUnit(const glm::vec3 &v0, const glm::vec3 &v1,
                             const glm::vec3 &v2, const glm::vec3 &v3) {
    return 0.25 * ((double)glm::length(v1 - v0) + (double)glm::length(v2 - v1)
                 + (double)glm::length(v3 - v2) + (double)glm::length(v0 - v3));
}

/* Screen pixels per radian of a perspective camera whose fov is VERTICAL
   (camera.cpp's projection). A square patch subtends the same angle on both axes,
   so the aspect cancels: it must NOT appear here. */
inline double lodPxPerRad(int viewport_h, float fov_vertical) {
    return 0.5 * (double)viewport_h / std::tan((double)fov_vertical * 0.5);
}

/* The patch's projected screen extent [px]. Exact, not small-angle:
   tan(theta/2) == (width_m/2)/dist. Residual second-order error: curvature/tilt
   errs toward subdividing, slant range to the centroid errs against it off-axis. */
inline double lodPxWidth(double width_m, double dist, double px_per_rad) {
    return (width_m / dist) * px_per_rad;
}

/* The camera position in BODY-FIXED axes: `transform` carries the body's SPIN,
   and the patch corners/centroid are body-fixed, so a render-frame camera would
   measure to a phantom camera rotated away by the spin. */
inline glm::dvec3 cameraInBodyFrame(const glm::dmat4 &transform,
                                    const glm::dvec3 &cam_rf) {
    const glm::dmat3 rot(transform);
    return glm::transpose(rot) * (cam_rf - glm::dvec3(transform[3]));
}
