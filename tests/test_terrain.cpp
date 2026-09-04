// test_terrain.cpp -- unit tests for the pure terrain core (src/terragen.h,
// glm + STL only): the height model (bounds, sea floor, band-limit fade),
// the surface color (sea, palette, gas-giant bands), and the grid builder
// (vertex/index counts, band-limited on-surface vertices, index range,
// the skirt ring dropped below the terrain). Links no GL / Bullet / imgui
// -- the game-side half (GL upload, collision, the patch tree) stays in
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

// Kerbin-like params (close to what load_system builds for the home body).
static TerrainParams kerbin() {
    TerrainParams t;
    t.radius = 600000.0f;
    t.surface.amplitude = 2500.0f;
    t.surface.octaves = 12;
    t.surface.persistence = 0.5f;
    t.surface.frequency = 1.0f;
    t.surface.has_sea = false;
    t.surface.sea_level = 0.0f;
    t.surface.max_height = 2500.0f;
    t.colour_func = &GetColourEarth;
    return t;
}

int main() {
    const std::vector<glm::vec3> dirs = sampleDirs();

    // 1. The height: finite everywhere, relief bounded by the amplitude
    //    (the model normalizes its max to exactly amplitude).
    {
        const TerrainParams t = kerbin();
        bool ok = true;
        for(const auto &p : dirs) {
            const float h = terrainHeight(p, t);
            if(!std::isfinite(h)) { ok = false; break; }
            if(std::fabs(h - t.radius) > t.surface.amplitude * 1.01f) { ok = false; break; }
        }
        check(ok, "height: finite and bounded by the amplitude");
    }

    // 2. The sea floor: with the sea level ABOVE the max relief, every
    //    point sits exactly on the flat sea floor.
    {
        TerrainParams t = kerbin();
        t.surface.has_sea = true;
        t.surface.sea_level = 10000.0f;   // above the max relief
        bool ok = true;
        for(const auto &p : dirs) {
            const float h = terrainHeight(p, t);
            if(!std::isfinite(h) || std::fabs(h - (t.radius + 10000.0f)) > 1e-3f) {
                ok = false; break;
            }
        }
        check(ok, "sea floor: height == radius + sea level");
    }

    // 3. The band-limit fade: a finer grid (smaller cell angle) keeps at
    //    least as many octaves; a max-depth-sized cell keeps them all.
    {
        const TerrainParams t = kerbin();
        const float coarse = terrainGridFade(1e-2f, 1.0f);
        const float fine   = terrainGridFade(1e-5f, 1.0f);
        check(fine > coarse, "fade: finer grid keeps more octaves");
        // Kerbin max-depth leaf cell (~5 m): every octave survives, so
        // the walked surface and the analytic one agree where landed.
        const float leaf = terrainGridFade(5.0f / t.radius, 1.0f);
        check(leaf >= (float)t.surface.octaves,
              "fade: max-depth cells keep every octave");
    }

    // 4. The surface color: finite and in a sane range for a body with
    //    the type-based (GetColourEarth) fallback.
    {
        const TerrainParams t = kerbin();
        bool ok = true;
        for(const auto &p : dirs) {
            const glm::vec3 c = terrainSurfaceColor(p, t);
            if(!finite_v(c)) { ok = false; break; }
            if(c.x < -1e-6f || c.y < -1e-6f || c.z < -1e-6f) { ok = false; break; }
            if(c.x > 1.0f + 1e-6f || c.y > 1.0f + 1e-6f || c.z > 1.0f + 1e-6f) { ok = false; break; }
        }
        check(ok, "color: finite and in [0,1]");
    }

    // 5. An all-sea body paints exactly the sea color everywhere.
    {
        TerrainParams t = kerbin();
        t.surface.has_sea = true;
        t.surface.sea_level = 10000.0f;   // above the max relief
        t.surface.sea_color = glm::vec3(0.1f, 0.2f, 0.9f);
        bool ok = true;
        for(const auto &p : dirs) {
            const glm::vec3 c = terrainSurfaceColor(p, t);
            if(glm::distance(c, t.surface.sea_color) > 1e-6f) { ok = false; break; }
        }
        check(ok, "all-sea: the color is exactly the sea color");
    }

    // 6. Gas giant (bands): a smooth sphere, color by latitude band.
    {
        TerrainParams t = kerbin();
        t.surface.bands = true;
        t.surface.band_count = 9;
        t.surface.palette.push_back(PaletteStop{0.0f, glm::vec3(0.1f, 0.1f, 0.3f)});
        t.surface.palette.push_back(PaletteStop{1.0f, glm::vec3(0.9f, 0.8f, 0.6f)});
        bool ok = true;
        for(const auto &p : dirs) {
            const float h = terrainHeight(p, t);
            if(!std::isfinite(h) || std::fabs(h - t.radius) > 1e-6f) { ok = false; break; }
        }
        check(ok, "bands: height == radius");
        // With 9 bands the equator sits at a band center (light) and the
        // poles at band edges (dark) -- the two must differ.
        const glm::vec3 eq = terrainSurfaceColor(glm::vec3(0, 1, 0), t);
        const glm::vec3 pole = terrainSurfaceColor(glm::vec3(0, 0, 1), t);
        check(finite_v(eq) && finite_v(pole), "bands: colors finite");
        check(glm::distance(eq, pole) > 0.1f, "bands: equator != pole color");
    }

    // A root-style patch quad (the same corners TerrainBody::Create uses).
    const glm::vec3 p1 = glm::normalize(glm::vec3( 1, 1, 1));
    const glm::vec3 p2 = glm::normalize(glm::vec3(-1, 1, 1));
    const glm::vec3 p3 = glm::normalize(glm::vec3(-1,-1, 1));
    const glm::vec3 p4 = glm::normalize(glm::vec3( 1,-1, 1));

    // 7. The grid without a skirt (a root patch): 49x49 vertices, 48x48
    //    quads, every vertex on the BAND-LIMITED height field (the same
    //    fade the builder computes for the quad), indices in range.
    {
        const TerrainParams t = kerbin();
        const GridGeom g = buildGridGeom(t, false, 1, p1, p2, p3, p4);
        check(g.verts.size() == 49 * 49, "grid: 49x49 vertices (no skirt)");
        check(g.indices.size() == 48 * 48 * 6, "grid: 48x48 quads * 6 (no skirt)");
        check(g.num_inner == 0, "grid: no skirt -> num_inner == 0");
        const float fade = terrainDepthFade(1, 49, t.surface.frequency);
        check(fade > 0.0f && fade < (float)t.surface.octaves,
              "grid: a root patch band-limits (partial octave set)");
        bool ok = true;
        for(const auto &v : g.verts) {
            if(!finite_v(v.pos) || !finite_v(v.normal) || !finite_v(v.color)) { ok = false; break; }
            const glm::vec3 d = glm::normalize(v.pos);
            const float h = terrainHeightFade(d, t, fade);
            if(std::fabs(glm::length(v.pos) - h) > h * 1e-4f) { ok = false; break; }
        }
        check(ok, "grid: vertices on the band-limited height field");
        bool idx_ok = true;
        for(const unsigned int ix : g.indices) {
            if(ix >= g.verts.size()) { idx_ok = false; break; }
        }
        check(idx_ok, "grid: indices in range");
    }

    // 8. The grid WITH a skirt (a child patch): 51x51 vertices, the inner
    //    quads first (num_inner), and the skirt ring strictly below the
    //    lowest terrain vertex.
    {
        const TerrainParams t = kerbin();
        const GridGeom g = buildGridGeom(t, true, 2, p1, p2, p3, p4);
        const int edge = 51;
        check(g.verts.size() == (size_t)edge * edge, "skirt: 51x51 vertices");
        check(g.indices.size() == 50 * 50 * 6, "skirt: 50x50 quads * 6");
        check(g.num_inner == 48 * 48 * 6, "skirt: inner index count");
        float skirt_max = 0.0f;
        float inner_min = HUGE_VALF;
        for(int i = 0; i < edge; i++) {
            for(int j = 0; j < edge; j++) {
                const float r = glm::length(g.verts[(size_t)j + (size_t)i * edge].pos);
                if(i >= 1 && i <= 49 && j >= 1 && j <= 49) {
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
