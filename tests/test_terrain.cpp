// test_terrain.cpp -- unit tests for the pure terrain core (src/terragen.h,
// glm + STL only): the height model (bounds, band-limit fade), the surface
// color (palette, gas-giant bands), the grid builder (vertex/index
// counts, band-limited on-surface vertices, the anchor-relative bake --
// patch-scale vertex data with a sub-cm double round trip -- index range,
// the skirt ring dropped below the terrain), and the patch-tree LOD maths
// (the size measure, the projected-pixel measure, the body-frame camera).
// Links camera.o (no GL / Bullet / imgui) so the LOD's pixel measure is
// pinned against the projection matrix the renderer really builds
// -- the game-side half (GL upload, collision, the patch tree) stays in
// terrain.cpp and is exercised by the e2e battery (--terrain-log).

#include "terragen.h"

#include "camera.h"

#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>   // translate / rotate (the LOD frame test)

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

// The true (spherical) area of a patch quad on the UNIT sphere: the solid
// angle of the four triangles fanned from the quad's centroid, each by
// Van Oosterom-Strackee. sqrt(area) is the patch's real linear size, which
// is what a LOD size measure has to track -- the midpoint subdivision makes
// same-depth patches differ in area by up to ~1.8x, so a measure can only be
// judged against the area, not against its siblings.
// DOUBLE throughout: at max_depth the corners are ~5e-4 apart, where the
// float32 cross product in `num` has lost most of its digits and the area
// comes out noise (which reads as a size-measure spread that isn't there).
static double sphericalArea(const glm::vec3 q[4]) {
    const glm::dvec3 p0(q[0]), p1(q[1]), p2(q[2]), p3(q[3]);
    const glm::dvec3 c = glm::normalize(p0 + p1 + p2 + p3);
    const glm::dvec3 v[5] = { c, p0, p1, p2, p3 };
    double a = 0.0;
    for(int i = 0; i < 4; i++) {
        const glm::dvec3 &u = v[0], &w = v[1 + i], &x = v[1 + (i + 1) % 4];
        const double num = std::fabs(glm::dot(u, glm::cross(w, x)));
        const double den = 1.0 + glm::dot(u, w) + glm::dot(w, x) + glm::dot(x, u);
        a += 2.0 * std::atan2(num, den);
    }
    return a;
}

// The old size measure, for comparison: one arbitrary edge chord (v0-v3).
static double edge03(const glm::vec3 q[4]) {
    return glm::length(glm::dvec3(q[0]) - glm::dvec3(q[3]));
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

    // 2. No sea floor clamp: terrain renders at its true height even
    //    below sea level (the ocean mesh covers it).
    {
        TerrainParams t = kerbin();
        t.surface.has_sea = true;
        t.surface.sea_level = 10000.0f;   // above the max relief
        bool ok = true;
        for(const auto &p : dirs) {
            const float h = terrainHeight(p, t);
            if(!std::isfinite(h)) { ok = false; break; }
            // Heights must NOT be clamped to sea_level -- the terrain
            // renders its true relief (well below 10000 here).
            if(std::fabs(h - (t.radius + 10000.0f)) < 1.0f) { ok = false; break; }
        }
        check(ok, "sea floor: terrain is NOT clamped to sea level");
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

    // 5. Sea color is NOT baked into terrain vertices (the ocean mesh
    //    provides the water color); below-sea terrain keeps palette colors.
    {
        TerrainParams t = kerbin();
        t.surface.has_sea = true;
        t.surface.sea_level = 10000.0f;   // above the max relief
        t.surface.sea_color = glm::vec3(0.1f, 0.2f, 0.9f);
        bool ok = true;
        for(const auto &p : dirs) {
            const glm::vec3 c = terrainSurfaceColor(p, t);
            // The color must NOT be the sea color -- terrain keeps its
            // palette/type-based color; the ocean shell paints the water.
            if(glm::distance(c, t.surface.sea_color) < 1e-6f) { ok = false; break; }
            if(!finite_v(c)) { ok = false; break; }
        }
        check(ok, "sea color: terrain is NOT painted with sea_color");
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

    // 7. The grid without a skirt (has_skirt=false -- no live caller,
    //    roots and children both build skirted; kept as the num_inner == 0
    //    convention check): 49x49 vertices, 48x48 quads, every vertex on
    //    the BAND-LIMITED height field (the same fade the builder computes
    //    for the quad, with the anchor added back in double), indices in
    //    range.
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
            const glm::dvec3 w = glm::dvec3(v.pos) + g.anchor;
            const glm::vec3 d = glm::normalize(glm::vec3(w));
            const float h = terrainHeightFade(d, t, fade);
            if(std::fabs(glm::length(w) - (double)h) > (double)h * 1e-4) { ok = false; break; }
        }
        check(ok, "grid: vertices on the band-limited height field");
        // The anchor: the quad's sphere centroid at (about) terrain radius.
        const glm::dvec3 cdir = glm::normalize(glm::dvec3(p1) + glm::dvec3(p2)
                                             + glm::dvec3(p3) + glm::dvec3(p4));
        bool anchor_ok = std::isfinite(g.anchor.x) && std::isfinite(g.anchor.y)
                      && std::isfinite(g.anchor.z);
        anchor_ok = anchor_ok
            && glm::dot(glm::normalize(g.anchor), cdir) > 0.999
            && std::fabs(glm::length(g.anchor) - (double)t.radius)
               < (double)t.surface.amplitude * 1.01;
        check(anchor_ok, "grid: anchor at the patch centroid, radius scale");
        bool idx_ok = true;
        for(const unsigned int ix : g.indices) {
            if(ix >= g.verts.size()) { idx_ok = false; break; }
        }
        check(idx_ok, "grid: indices in range");
    }

    // 7b. The anchor-relative bake (the float32 jitter fix): a deep patch's
    //     vertex data is PATCH-scale, not radius-scale, and adding the
    //     anchor back in double recovers the body-frame surface point to
    //     well under a centimetre-scale quantum. A body-centred float bake
    //     quantized at ULP(radius) (~6 cm on Kerbin, ~70 cm on Jool), which
    //     made the terrain swim around the launch pad as the camera moved.
    {
        const TerrainParams t = kerbin();
        const int depth = 9;
        // Corners of a REAL depth-9 patch: the first child of the root
        // face, subdivided down (the midpoint scheme subdivideCorners uses).
        glm::vec3 q0 = p1, q1 = p2, q2 = p3, q3 = p4;
        for(int lvl = 1; lvl < depth; lvl++) {
            const glm::vec3 v01 = glm::normalize(q0 + q1);
            const glm::vec3 v30 = glm::normalize(q3 + q0);
            const glm::vec3 cn  = glm::normalize(q0 + q1 + q2 + q3);
            q1 = v01; q2 = cn; q3 = v30;   // q0 stays: child quad[0]
        }
        const GridGeom g = buildGridGeom(t, true, depth, q0, q1, q2, q3);
        const float fade = terrainDepthFade(depth, 49, t.surface.frequency);
        // Patch extent bound: the root-face edge angle / 2^(depth-1) times
        // the radius (the lateral half-span), plus the full relief swing
        // (the anchor sits at the centroid's height).
        const double extent = (double)t.radius * 1.2310 / (1 << (depth - 1))
                              + 2.0 * (double)t.surface.amplitude;
        bool small = true;
        for(const auto &v : g.verts) {
            if((double)glm::length(v.pos) > extent) { small = false; break; }
        }
        check(small, "anchor: vertex data is patch-scale, not radius-scale");
        // Round trip: re-derive the baker's own float direction + height,
        // and the anchored vertex must sit within float-at-patch-scale
        // rounding of it (sub-mm here; a body-centred bake would be off
        // by ~6 cm, orders past the tolerance).
        const float frac = 1.0f / 48.0f;
        bool precise = true;
        for(int i = 1; i <= 49 && precise; i++) {
            for(int j = 1; j <= 49; j++) {
                const TerrVert &v = g.verts[(size_t)j + (size_t)i * 51];
                const glm::vec3 d = terrainSpherePoint(q0, q1, q2, q3,
                                                       (i - 1) * frac,
                                                       (j - 1) * frac);
                const double h = (double)terrainHeightFade(d, t, fade);
                const glm::dvec3 w = glm::dvec3(v.pos) + g.anchor;
                if(glm::length(w - glm::dvec3(d) * h) > 1e-2) { precise = false; break; }
            }
        }
        check(precise, "anchor: round trip to the height field is sub-cm");
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
                // radius = the anchored position added back in double
                const float r = (float)glm::length(
                    glm::dvec3(g.verts[(size_t)j + (size_t)i * edge].pos)
                    + g.anchor);
                if(i >= 1 && i <= 49 && j >= 1 && j <= 49) {
                    inner_min = std::min(inner_min, r);
                } else {
                    skirt_max = std::max(skirt_max, r);
                }
            }
        }
        check(skirt_max < inner_min, "skirt: ring dropped below the terrain");
        // Each ring vertex copies the normal/color of the ADJACENT inner
        // edge vertex (the seam it fills), so the skirt shades exactly
        // like the terrain boundary -- a ring that copied one corner's
        // shading painted a faint line along that edge.
        bool seam_ok = true;
        for(int j = 1; j <= 49 && seam_ok; j++) {
            const TerrVert &l = g.verts[(size_t)j + (size_t)0 * edge];
            const TerrVert &r = g.verts[(size_t)j + (size_t)50 * edge];
            const TerrVert &li = g.verts[(size_t)j + (size_t)1 * edge];
            const TerrVert &ri = g.verts[(size_t)j + (size_t)49 * edge];
            seam_ok = (l.normal == li.normal && l.color == li.color)
                   && (r.normal == ri.normal && r.color == ri.color);
        }
        for(int i = 1; i <= 49 && seam_ok; i++) {
            const TerrVert &top = g.verts[(size_t)0 + (size_t)i * edge];
            const TerrVert &bot = g.verts[(size_t)50 + (size_t)i * edge];
            const TerrVert &ti = g.verts[(size_t)1 + (size_t)i * edge];
            const TerrVert &bi = g.verts[(size_t)49 + (size_t)i * edge];
            seam_ok = (top.normal == ti.normal && top.color == ti.color)
                   && (bot.normal == bi.normal && bot.color == bi.color);
        }
        check(seam_ok, "skirt: ring vertices copy the adjacent edge's normal/color");
    }

    // 9. The cloud deck coverage (baked into the deck texture at load):
    //    in [0,1], varies across the surface (not a constant), and the
    //    coverage parameter shifts the mean monotonically (more
    //    coverage -> more cloud).
    {
        CloudParams c;
        c.coverage = 0.6f;
        c.freq = 10.0f;
        bool ok = true, varied = false;
        float first = -1.0f;
        for(const auto &p : dirs) {
            const float v = cloudCover(p, glm::mat3(1.0f), c);
            if(!std::isfinite(v) || v < -1e-6f || v > 1.0f + 1e-6f) { ok = false; break; }
            if(first < 0.0f) { first = v; }
            else if(std::fabs(v - first) > 1e-3f) { varied = true; }
        }
        check(ok, "clouds: coverage finite and in [0,1]");
        check(varied, "clouds: the pattern varies across the surface");
        // A spread of directions (Fibonacci, like load_system's max_height):
        // the higher coverage setting must average more cloud.
        float mean_lo = 0.0f, mean_hi = 0.0f;
        const int N = 256;
        CloudParams lo = c; lo.coverage = 0.2f;
        CloudParams hi = c; hi.coverage = 0.9f;
        for(int i = 0; i < N; i++) {
            const float y = 1.0f - 2.0f * (i + 0.5f) / (float)N;
            const float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
            const glm::vec3 d(rr * std::cos(i * 2.39996322972865332f), y,
                              rr * std::sin(i * 2.39996322972865332f));
            mean_lo += cloudCover(d, glm::mat3(1.0f), lo);
            mean_hi += cloudCover(d, glm::mat3(1.0f), hi);
        }
        check(mean_hi > mean_lo, "clouds: more coverage -> more cloud");
        // Determinism: the same direction/params give the same value
        // (the bake and any re-bake must agree).
        check(cloudCover(dirs[3], glm::mat3(1.0f), c)
              == cloudCover(dirs[3], glm::mat3(1.0f), c),
              "clouds: deterministic");
    }

    // 10. The LOD's projected-pixel measure, pinned against the projection
    //     matrix the renderer actually builds (camera.cpp's reverse-Z
    //     perspective). A `size`-metre object square-on at `dist` metres must
    //     cover exactly lodPxWidth(size, dist, lodPxPerRad(vh, fov)) pixels --
    //     horizontally AND vertically (a square patch subtends the same angle
    //     both ways, so the window aspect cancels), and across the FOV range
    //     the in-game slider sweeps. The old measure divided the VERTICAL
    //     pixel count by the HORIZONTAL fov, rescaling the whole budget by
    //     2*tan(fov/2)/fov_h: 0.65..1.38 over fov 45..120 deg at 16:9, i.e.
    //     --terrain-px 512 behaved like ~710 px and changed meaning with the
    //     window shape.
    {
        const double dist = 5000.0, size = 1000.0;
        bool ok = true;
        // Heights whose product with every aspect below is a whole pixel
        // count, and rounded (a truncated viewport_w would leak the aspect
        // back in through the pixel scale).
        for(const int vh : {720, 1080, 1440}) {
            for(const float aspect : {1.0f, 16.0f / 9.0f, 21.0f / 9.0f}) {
                for(const float fovdeg : {20.0f, 60.0f, 100.0f}) {
                    const float fov = (float)glm::radians(fovdeg);
                    Camera cam(glm::dvec3(0.0), fov, aspect, 1.0f, 1e13f);
                    cam.setViewport((int)llround((double)vh * aspect), vh);
                    const glm::mat4 P = cam.GetProjection();
                    // view space: the object at -dist, centred on the axis.
                    auto px = [&](double x, double y) {
                        const glm::vec4 c = P * glm::vec4((float)x, (float)y,
                                                          (float)-dist, 1.0f);
                        const glm::vec3 ndc = glm::vec3(c) / c.w;
                        return glm::dvec2((ndc.x * 0.5 + 0.5) * cam.viewport_w,
                                          (0.5 - ndc.y * 0.5) * cam.viewport_h);
                    };
                    // `want` has no aspect in it, so this loop IS the
                    // aspect-cancellation check: the same fov + viewport
                    // height must give the same px at 1:1, 16:9 and 21:9.
                    const double want = lodPxWidth(size, dist,
                                                   lodPxPerRad(vh, fov));
                    const glm::dvec2 l = px(-size * 0.5, 0), r = px(size * 0.5, 0);
                    const glm::dvec2 b = px(0, -size * 0.5), t = px(0, size * 0.5);
                    if(std::fabs(std::fabs(r.x - l.x) - want) > want * 1e-4) { ok = false; }
                    if(std::fabs(std::fabs(t.y - b.y) - want) > want * 1e-4) { ok = false; }
                }
            }
        }
        check(ok, "lod px: matches the real projection, both axes, all aspects/fovs");
        // Halving the viewport halves the measure; doubling the fov does not
        // double it (tan, not the angle) -- the budget is in px, so it tracks
        // resolution exactly and FOV correctly.
        const double p60 = lodPxPerRad(1080, (float)glm::radians(60.0));
        check(p60 > 900.0 && p60 < 950.0,
              "lod px: 1080p at 60 deg vertical fov is ~935 px/rad");
        check(std::fabs(lodPxPerRad(540, (float)glm::radians(60.0)) - p60 * 0.5) < 1e-9,
              "lod px: scales linearly with the viewport height");
        check(lodPxPerRad(1080, (float)glm::radians(120.0)) < p60 * 2.0,
              "lod px: wider fov -> fewer px per rad (tan, not linear)");
    }

    // 11. The LOD's SIZE measure. patchWidthUnit must track the patch's true
    //     linear size (sqrt of its spherical area) to within a few percent at
    //     every depth, and be symmetric in the corner labelling. One edge
    //     chord -- what width_m used to be -- is neither: the four children of
    //     one parent measured 0.606/0.765/0.765/0.606 (26% apart) despite
    //     having IDENTICAL areas, so two of them subdivided a whole level
    //     later than their brothers. That is the "one coarse patch inside a
    //     detailed square" sighting.
    {
        // The four children of the root face: equal area, unequal v0-v3.
        glm::vec3 quad[4][4];
        subdivideCorners(p1, p2, p3, p4, quad);
        double wmin = 1e30, wmax = 0, emin = 1e30, emax = 0;
        for(int q = 0; q < 4; q++) {
            const double w = patchWidthUnit(quad[q][0], quad[q][1],
                                            quad[q][2], quad[q][3]);
            const double e = edge03(quad[q]);
            wmin = std::min(wmin, w); wmax = std::max(wmax, w);
            emin = std::min(emin, e); emax = std::max(emax, e);
        }
        check(wmax / wmin < 1.001, "lod size: one parent's four children measure equal");
        check(emax / emin > 1.2, "lod size: ...while a single edge chord does not (the old bias)");
        // Corner-labelling symmetry: cycling and reversing the quad must not
        // change the measure (a patch's LOD cannot depend on which corner the
        // subdivision happened to call v0).
        {
            const glm::vec3 q[4] = { quad[1][0], quad[1][1], quad[1][2], quad[1][3] };
            const double base = patchWidthUnit(q[0], q[1], q[2], q[3]);
            bool sym = true;
            for(int s = 0; s < 4; s++) {
                if(std::fabs(patchWidthUnit(q[s], q[(s+1)%4], q[(s+2)%4], q[(s+3)%4]) - base) > 1e-12) { sym = false; }
            }
            if(std::fabs(patchWidthUnit(q[0], q[3], q[2], q[1]) - base) > 1e-12) { sym = false; }
            check(sym, "lod size: symmetric under cycling/reversing the corners");
        }
        // Tracks the true linear size all the way down the tree (to depth 12,
        // a Kerbin-sized body's max_depth): the ratio's spread stays under
        // 10% at every depth -- measured 1.00 at depth 2, 1.063 at 6, 1.071
        // saturating from depth 8 -- where a single edge chord swings 28%.
        // Each level is subsampled to <= 1024 quads so the walk stays bounded
        // (the stats are taken over the level BEFORE the stride).
        const int max_walk_depth = 12;
        std::vector<std::array<glm::vec3, 4> > level = { { { p1, p2, p3, p4 } } };
        std::vector<double> means;   // the mean measure, one per depth
        bool tracks = true, chord_worse = false;
        for(int depth = 1; depth <= max_walk_depth && tracks; depth++) {
            std::vector<std::array<glm::vec3, 4> > next;
            double rmin = 1e30, rmax = 0, cmin = 1e30, cmax = 0, sum = 0;
            for(const auto &q : level) {
                glm::vec3 kids[4][4];
                subdivideCorners(q[0], q[1], q[2], q[3], kids);
                for(int k = 0; k < 4; k++) { next.push_back({ { kids[k][0], kids[k][1], kids[k][2], kids[k][3] } }); }
                const double lin = std::sqrt(sphericalArea(q.data()));
                const double w = patchWidthUnit(q[0], q[1], q[2], q[3]);
                const double r = w / lin;
                const double c = edge03(q.data()) / lin;
                sum += w;
                rmin = std::min(rmin, r); rmax = std::max(rmax, r);
                cmin = std::min(cmin, c); cmax = std::max(cmax, c);
            }
            means.push_back(sum / (double)level.size());
            if(rmax / rmin > 1.10) { tracks = false; }
            if(depth >= 3 && cmax / cmin > 1.2) { chord_worse = true; }
            if(next.size() > 1024) {
                // Deterministic UNBIASED subsample (xorshift64 over the index
                // range). A fixed stride would follow the quadtree ordering
                // and sample one region of the face, which skews the spread.
                std::vector<std::array<glm::vec3, 4> > picked;
                picked.reserve(1024);
                uint64_t s = 0x2545F4914F6CDD1Dull;
                while(picked.size() < 1024) {
                    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
                    picked.push_back(next[(size_t)(s % next.size())]);
                }
                next.swap(picked);
            }
            level.swap(next);
        }
        check(tracks, "lod size: mean edge chord tracks sqrt(area) within 10% to max_depth");
        check(chord_worse, "lod size: ...a single edge chord does not (the old measure)");
        // Halves per level (the tree's scale invariant: one level = half the
        // size, which is what makes max_depth a metre-scale statement). The
        // root -> child step is the exception (0.59: a cube-face corner chord
        // is 2/sqrt(3) while its children's are ~1), so check from depth 2.
        bool halves = true;
        for(size_t i = 2; i < means.size(); i++) {
            const double r = means[i] / means[i - 1];
            if(r < 0.45 || r > 0.55) { halves = false; }
        }
        check(halves, "lod size: one subdivision halves the measure below the root");
    }

    // 12. The LOD's camera position must be in BODY-FIXED axes. `transform`
    //     is body-fixed -> render frame and carries the body's spin, so
    //     subtracting only its translation compares a render-frame camera
    //     against body-fixed patch corners -- for a body seen from an
    //     inertial render frame that measures the distance to a phantom
    //     camera rotated away by the spin (up to 2*|p|: the detail lands at
    //     the wrong longitude, and the ground under you stays coarse).
    {
        // The local body seen from an INERTIAL render frame (any ship above
        // the rotating frame's SOI): transform is the pure spin, no
        // translation. Camera 100 km ASL over a 600 km body, spun 37 deg
        // about Y and tilted 11 deg about X -- the shape render.cpp builds.
        const glm::dmat4 T =
            glm::rotate(glm::dmat4(1.0), glm::radians(37.0), glm::dvec3(0, 1, 0))
            * glm::rotate(glm::dmat4(1.0), glm::radians(11.0), glm::dvec3(1, 0, 0));
        const glm::dvec3 cam_rf = 700000.0 * glm::normalize(glm::dvec3(1.0, 0.05, 0.05));
        const glm::dvec3 p_bf = 600000.0 * glm::normalize(glm::dvec3(0.98, 0.15, 0.1));
        const glm::dvec3 cam_bf = cameraInBodyFrame(T, cam_rf);
        // Round trip: the body-frame camera maps back to the render-frame one.
        const glm::dvec3 back = glm::dmat3(T) * cam_bf + glm::dvec3(T[3]);
        check(glm::length(back - cam_rf) < 1e-6, "lod frame: body-frame camera round trips");
        // The invariant the LOD needs: a patch centroid at p_bf is DRAWN at
        // R*p_bf, so the distance the LOD uses must equal the distance to
        // where the patch actually is on screen.
        const double want = glm::length(cam_rf - (glm::dmat3(T) * p_bf + glm::dvec3(T[3])));
        const double got = glm::length(cam_bf - p_bf);
        check(std::fabs(got - want) < 1e-6, "lod frame: distance matches the drawn patch");
        // ...which the translation-only form gets wrong by hundreds of km
        // here (it would read 124 km instead of 378 km): the detail lands at
        // the wrong longitude and the ground under the camera stays coarse.
        // This is the assertion that pins the bug the fix is for.
        const double stale = glm::length((cam_rf - glm::dvec3(T[3])) - p_bf);
        check(std::fabs(stale - want) > 100000.0,
              "lod frame: translation-only (the old code) is 100s of km wrong");
        // A body that is also OFFSET in the render frame (every non-local
        // body): the same invariant, translation and spin together.
        const glm::dmat4 T2 = glm::translate(glm::dvec3(3.0e7, -1.2e7, 4.5e7)) * T;
        const glm::dvec3 cam2(3.01e7, -1.19e7, 4.51e7);
        const glm::dvec3 cb2 = cameraInBodyFrame(T2, cam2);
        const double want2 = glm::length(cam2 - (glm::dmat3(T2) * p_bf + glm::dvec3(T2[3])));
        check(std::fabs(glm::length(cb2 - p_bf) - want2) < 1e-6,
              "lod frame: offset + spinning body round trips too");
        // Identity rotation (a landed ship: its render frame IS the body's
        // spin frame) must reduce to the plain translation subtract.
        const glm::dmat4 Ti = glm::translate(glm::dvec3(1.0, 2.0, 3.0));
        check(glm::length(cameraInBodyFrame(Ti, cam_rf) - (cam_rf - glm::dvec3(1.0, 2.0, 3.0))) < 1e-9,
              "lod frame: identity spin reduces to subtracting the position");
    }

    // 13. Biome classification.
    //     biomeFromAltitude takes altitude ABOVE SEA LEVEL [m] and the body's
    //     Surface: <= 0 with a sea is Ocean; the solid biomes cut the
    //     [0, max_height] range at 50% (Midlands) and 80% (Mountain); a
    //     banded body (no solid surface) is None. biomeAt is that same
    //     decision applied to terrainHeight at a direction.
    {
        Surface oceanic = kerbin().surface;   // max_height 2500
        oceanic.has_sea = true;
        Surface dry = oceanic;
        dry.has_sea = false;                  // a basin is land, not sea
        Surface unmeasured = dry;             // pre-AttachRoot (Surface's default)
        unmeasured.max_height = 1.0f;
        Surface flat = dry;
        flat.max_height = 0.0f;               // defensive guard: no real body gets here
        Surface banded = dry;
        banded.bands = true;                  // a gas giant

        check(biomeFromAltitude(-50.0, oceanic) == Biome::Ocean, "biome: below sea is ocean");
        check(biomeFromAltitude(0.0, oceanic) == Biome::Ocean, "biome: at sea level is ocean");
        check(biomeFromAltitude(0.0, dry) == Biome::Lowland, "biome: dry body at sea level is lowland");
        check(biomeFromAltitude(-50.0, dry) == Biome::Lowland, "biome: dry body's basin is lowland");

        const double mh = 2500.0;
        check(biomeFromAltitude(0.1, oceanic) == Biome::Lowland,
              "biome: just above sea level is lowland");
        check(biomeFromAltitude(0.5 * mh - 1.0, oceanic) == Biome::Lowland,
              "biome: just under 50% is lowland");
        check(biomeFromAltitude(0.5 * mh, oceanic) == Biome::Midlands,
              "biome: at 50% is midlands");
        check(biomeFromAltitude(0.8 * mh - 1.0, oceanic) == Biome::Midlands,
              "biome: just under 80% is midlands");
        check(biomeFromAltitude(0.8 * mh, oceanic) == Biome::Mountain,
              "biome: at 80% is mountain");
        check(biomeFromAltitude(2.0 * mh, oceanic) == Biome::Mountain,
              "biome: above the range is still mountain");

        check(biomeFromAltitude(100.0, banded) == Biome::None, "biome: banded body has none");
        // Unmeasured (pre-AttachRoot): max_height is the 1.0f default, so the
        // bands scale to metres -- a high sample reads mountain. Documented,
        // not special-cased: callers classify bodies after the heavy phase.
        check(biomeFromAltitude(100.0, unmeasured) == Biome::Mountain,
              "biome: unmeasured body scales by the default max_height");
        check(biomeFromAltitude(100.0, flat) == Biome::Lowland, "biome: zero max_height is lowland");

        // biomeAt: the same decision on a real height field -- must agree
        // with biomeFromAltitude fed terrainHeight minus the sea-level datum.
        const TerrainParams kp = kerbin();
        const double alt = (double)terrainHeight({0.0, 0.0, 1.0}, kp)
                         - (double)kp.radius - (double)kp.surface.sea_level;
        check(biomeAt({0.0, 0.0, 1.0}, kp) == biomeFromAltitude(alt, kp.surface),
              "biome: biomeAt matches the altitude-based decision");
        TerrainParams banded_body = kp;
        banded_body.surface = banded;
        check(biomeAt({0.0, 0.0, 1.0}, banded_body) == Biome::None,
              "biome: biomeAt sees the banded surface too");

        check(std::string(biomeName(Biome::Ocean)) == "ocean", "biome: name ocean");
        check(std::string(biomeName(Biome::Lowland)) == "lowlands", "biome: name lowlands");
        check(std::string(biomeName(Biome::Midlands)) == "midlands", "biome: name midlands");
        check(std::string(biomeName(Biome::Mountain)) == "mountains", "biome: name mountains");
        check(std::string(biomeName(Biome::None)) == "none", "biome: name none");
    }

    // 14. The atmosphere top. An authored height wins; without one the top
    //     derives from scale_height * kAtmoScaleHeights (the e^-10 density
    //     altitude), so a body with a drag model still gets a hard edge of
    //     space; a body with neither has no top. (The cutoff airDensity
    //     applies at that altitude is pinned in test_drag.cpp.)
    {
        AtmosphereParams a;
        a.sea_level_density = 1.225;
        a.scale_height = 5500.0;              // Kerbin's, height unauthored
        check(a.top() == 5500.0 * kAtmoScaleHeights,
              "atmo: an unauthored height derives scale_height * 10");
        a.height = 70000.0;                   // Kerbin's authored top
        check(a.top() == 70000.0,
              "atmo: an authored height wins over the derivation");
        AtmosphereParams none;                // no drag model at all
        check(none.top() == 0.0, "atmo: no atmosphere has no top");
    }

    if(g_failures == 0) {
        std::printf("test_terrain: all checks passed\n");
        return 0;
    }
    std::printf("test_terrain: %d check(s) failed\n", g_failures);
    return 1;
}
