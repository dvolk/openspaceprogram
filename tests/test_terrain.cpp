// test_terrain.cpp -- unit tests for the pure terrain core (src/terragen.h,
// glm + STL only): the height function (sea floor + altitude rescale),
// the surface color (sea, palette, gas-giant bands), and the grid builder
// (vertex/index counts, on-surface vertices, index range, and the skirt
// ring dropped below the terrain). Links no GL / Bullet / imgui -- the
// game-side half (GL upload, collision, the patch tree) stays in
// terrain.cpp and is exercised by the e2e battery.

#include "terragen.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <vector>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

static bool finite_v(const glm::vec3 &v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// A spread of unit directions (the axes, the octants, some in between).
static std::vector<glm::vec3> sampleDirs() {
    std::vector<glm::vec3> d;
    d.push_back(glm::vec3(1, 0, 0));
    d.push_back(glm::vec3(-1, 0, 0));
    d.push_back(glm::vec3(0, 1, 0));
    d.push_back(glm::vec3(0, -1, 0));
    d.push_back(glm::vec3(0, 0, 1));
    d.push_back(glm::vec3(0, 0, -1));
    d.push_back(glm::normalize(glm::vec3(1, 1, 1)));
    d.push_back(glm::normalize(glm::vec3(-1, 1, 1)));
    d.push_back(glm::normalize(glm::vec3(1, -1, 1)));
    d.push_back(glm::normalize(glm::vec3(1, 1, -1)));
    d.push_back(glm::normalize(glm::vec3(0.3, 0.4, 0.9)));
    d.push_back(glm::normalize(glm::vec3(-0.7, 0.2, 0.5)));
    return d;
}

// Earth-like params (the defaults load_system would set).
static TerrainParams earth() {
    TerrainParams t;
    t.radius = 6000000.0f;
    t.surface.amplitude = 2500.0f;
    t.surface.octaves = 12;
    t.surface.persistence = 0.6f;
    t.surface.frequency = 1.0f;
    t.surface.power = 3;
    t.surface.has_sea = false;
    t.surface.sea_level = 0.0f;
    t.surface.max_height = 10000.0f;
    t.colour_func = &GetColourEarth;
    return t;
}

int main() {
    const std::vector<glm::vec3> dirs = sampleDirs();

    // 1. The height: finite everywhere, relief bounded by the noise
    //    amplitude (the octave sum magnitude is a loose upper bound).
    {
        const TerrainParams t = earth();
        bool ok = true;
        for(const auto &p : dirs) {
            float h = terrainHeight(p, t);
            if(!std::isfinite(h)) { ok = false; break; }
            if(std::fabs(h - t.radius) > t.surface.amplitude * 10.0f) { ok = false; break; }
        }
        check(ok, "height: finite and bounded");
    }

    // 2. The sea floor: with the sea level ABOVE the max noise, every
    //    point sits exactly on the (rescaled) sea floor.
    {
        TerrainParams t = earth();
        t.surface.has_sea = true;
        t.surface.sea_level = 10000.0f; // above |fbm| * amplitude
        const float floor_h = t.radius + terrainScaleHeightNoise(t.surface.sea_level, t);
        bool ok = true;
        for(const auto &p : dirs) {
            float h = terrainHeight(p, t);
            if(!std::isfinite(h) || h < floor_h * (1.0f - 1e-4f)) { ok = false; break; }
        }
        check(ok, "sea floor: height >= the rescaled sea level");
    }

    // 3. The surface color: finite and in a sane range for a body with
    //    the type-based (GetColourEarth) fallback.
    {
        const TerrainParams t = earth();
        bool ok = true;
        for(const auto &p : dirs) {
            glm::vec3 c = terrainSurfaceColor(p, t);
            if(!finite_v(c)) { ok = false; break; }
            if(c.x < -1e-6f || c.y < -1e-6f || c.z < -1e-6f) { ok = false; break; }
            if(c.x > 5.0f || c.y > 5.0f || c.z > 5.0f) { ok = false; break; }
        }
        check(ok, "color: finite and in a sane range");
    }

    // 4. An all-sea body paints exactly the sea color everywhere.
    {
        TerrainParams t = earth();
        t.surface.has_sea = true;
        t.surface.sea_level = 10000.0f; // above |fbm| * amplitude
        t.surface.sea_color = glm::vec3(0.1f, 0.2f, 0.9f);
        bool ok = true;
        for(const auto &p : dirs) {
            glm::vec3 c = terrainSurfaceColor(p, t);
            if(glm::distance(c, t.surface.sea_color) > 1e-6f) { ok = false; break; }
        }
        check(ok, "all-sea: the color is exactly the sea color");
    }

    // 5. Gas giant (bands): a smooth sphere, color by latitude band.
    {
        TerrainParams t = earth();
        t.surface.bands = true;
        t.surface.band_count = 9;
        t.surface.palette.push_back(PaletteStop{0.0f, glm::vec3(0.1f, 0.1f, 0.3f)});
        t.surface.palette.push_back(PaletteStop{1.0f, glm::vec3(0.9f, 0.8f, 0.6f)});
        // Height is exactly the radius (no noise).
        bool ok = true;
        for(const auto &p : dirs) {
            float h = terrainHeight(p, t);
            if(!std::isfinite(h) || std::fabs(h - t.radius) > 1e-6f) { ok = false; break; }
        }
        check(ok, "bands: height == radius");
        // With 9 bands the equator sits at a band center (light) and the
        // poles at band edges (dark) -- the two must differ.
        glm::vec3 eq = terrainSurfaceColor(glm::vec3(0, 1, 0), t);
        glm::vec3 pole = terrainSurfaceColor(glm::vec3(0, 0, 1), t);
        check(finite_v(eq) && finite_v(pole), "bands: colors finite");
        check(glm::distance(eq, pole) > 0.1f, "bands: equator != pole color");
    }

    // A root-style patch quad (the same corners TerrainBody::Create uses).
    const glm::vec3 p1 = glm::normalize(glm::vec3( 1, 1, 1));
    const glm::vec3 p2 = glm::normalize(glm::vec3(-1, 1, 1));
    const glm::vec3 p3 = glm::normalize(glm::vec3(-1,-1, 1));
    const glm::vec3 p4 = glm::normalize(glm::vec3( 1,-1, 1));

    // 6. The grid without a skirt (a root patch): 25x25 vertices, 24x24
    //    quads, every vertex on the height field, indices in range.
    {
        const TerrainParams t = earth();
        GridGeom g = buildGridGeom(t, false, p1, p2, p3, p4);
        check(g.verts.size() == 25 * 25, "grid: 25x25 vertices (no skirt)");
        check(g.indices.size() == 24 * 24 * 6, "grid: 24x24 quads * 6 (no skirt)");
        check(g.num_inner == 0, "grid: no skirt -> num_inner == 0");
        bool ok = true;
        for(const auto &v : g.verts) {
            if(!finite_v(v.pos) || !finite_v(v.normal) || !finite_v(v.color)) { ok = false; break; }
            const glm::vec3 d = glm::normalize(v.pos);
            const float h = terrainHeight(d, t);
            const float r = glm::length(v.pos);
            if(std::fabs(r - h) > h * 1e-4f) { ok = false; break; }
        }
        check(ok, "grid: vertices on the height field");
        bool idx_ok = true;
        for(const unsigned int ix : g.indices) {
            if(ix >= g.verts.size()) { idx_ok = false; break; }
        }
        check(idx_ok, "grid: indices in range");
    }

    // 7. The grid WITH a skirt (a child patch): 27x27 vertices, the inner
    //    quads first (num_inner), and the skirt ring strictly below the
    //    lowest terrain vertex.
    {
        const TerrainParams t = earth();
        GridGeom g = buildGridGeom(t, true, p1, p2, p3, p4);
        const int edge = 27;
        check(g.verts.size() == (size_t)edge * edge, "skirt: 27x27 vertices");
        check(g.indices.size() == 26 * 26 * 6, "skirt: 26x26 quads * 6");
        check(g.num_inner == 24 * 24 * 6, "skirt: inner index count");
        float skirt_max = 0.0f;
        float inner_min = HUGE_VALF;
        for(int i = 0; i < edge; i++) {
            for(int j = 0; j < edge; j++) {
                const float r = glm::length(g.verts[(size_t)j + (size_t)i * edge].pos);
                if(i >= 1 && i <= 25 && j >= 1 && j <= 25) {
                    inner_min = std::min(inner_min, r);
                } else {
                    skirt_max = std::max(skirt_max, r);
                }
            }
        }
        check(skirt_max < inner_min, "skirt: ring dropped below the terrain");
    }

    if(g_failures == 0) {
        std::printf("test_terrain: all checks passed\n");
        return 0;
    }
    std::printf("test_terrain: %d check(s) failed\n", g_failures);
    return 1;
}
