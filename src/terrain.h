// terrain.h -- terrain data + geometry types.
//
//   COLOUR             rgb triplet (legacy name).
//   PaletteStop        one stop on a body's land-color ramp.
//   AtmosphereParams   per-body Fresnel limb-glow rim.
//   Surface            per-body terrain + color params (the "surface" JSON
//                      block); holds the palette + atmosphere.
//   GeoPatch           one subdividable spherical patch (terrain LOD node).
//   TerrainBody        a celestial body's terrain: 6 root patches, color
//                      ramp, atmosphere shell, height/shadow queries.

#pragma once

#include <cmath>
#include <vector>

#include <glm/glm.hpp>

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
