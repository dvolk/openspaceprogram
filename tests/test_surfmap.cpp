// test_surfmap.cpp -- unit tests for the equirectangular projection
// (src/equirect.h) plus the Surface Map's shade/wrap helpers
// (src/surfmap.h). Header-only pure math (glm + <cmath>). Pins the
// lon/lat convention (lon 0 = +X = rail lon 0, +90 deg = -Z east, north
// = +Y -- see equirect.h / #200), the pixel <-> direction round-trip, the
// cloud-deck mesh-phi UV bridge (cloudShader.vs), a heightmap column
// round-trip, the terminator's shade range, and the antimeridian wrap
// detection. Links no imgui / Bullet / GL.

#include "equirect.h"
#include "surfmap.h"
#include "terragen.h"

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <cstring>
#include <memory>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

static bool near_d(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

static bool near_v(const glm::dvec3 &a, const glm::dvec3 &b, double tol) {
    return near_d(a.x, b.x, tol) && near_d(a.y, b.y, tol)
        && near_d(a.z, b.z, tol);
}

int main() {
    const int w = 16, h = 8;

    // 1. The pixel direction convention (equirect.h, the same pair
    //    render.cpp reads off the ship's position).
    //    lon 0 is +X (the left edge), +90 deg is -Z (east), north is +Y.
    {
        // A non-pole row: the direction is
        //   (cos lat . sin lon, sin lat, cos lat . cos lon)
        // with lat = pi/2 - pi * j / (h-1). (An even h has no exact
        // equator row -- j = (h-1)/2 is half-integer -- so the expected
        // lat is computed, not assumed.)
        const int j = h / 2;
        const double lat = M_PI * 0.5 - M_PI * (double)j / (double)(h - 1);
        const double cl = std::cos(lat), sl = std::sin(lat);
        check(near_v(equirectDirAt(0, j, w, h), glm::dvec3(cl, sl, 0), 1e-9),
              "dir: lon 0 -> +X hemisphere");
        check(near_v(equirectDirAt(w / 4, j, w, h), glm::dvec3(0, sl, -cl), 1e-9),
              "dir: lon 90 -> -Z hemisphere");
        check(near_v(equirectDirAt(w / 2, j, w, h), glm::dvec3(-cl, sl, 0), 1e-9),
              "dir: lon 180 -> -X hemisphere");
        check(near_v(equirectDirAt(3 * w / 4, j, w, h), glm::dvec3(0, sl, cl), 1e-9),
              "dir: lon 270 -> +Z hemisphere");
    }
    {
        // Poles: every column of row 0 is the north pole (lat +pi/2),
        // row h-1 the south.
        check(near_v(equirectDirAt(0, 0, w, h), glm::dvec3(0, 1, 0), 1e-9),
              "dir: row 0 -> north (+Y)");
        check(near_v(equirectDirAt(w / 3, 0, w, h), glm::dvec3(0, 1, 0), 1e-9),
              "dir: row 0, any column -> north");
        check(near_v(equirectDirAt(0, h - 1, w, h), glm::dvec3(0, -1, 0), 1e-9),
              "dir: row h-1 -> south (-Y)");
    }
    {
        // Latitude at a mid-column (lon 0, so the direction is (x, y, 0)):
        // row 1 of 8 -> lat = pi/2 - pi/7.
        const glm::dvec3 d = equirectDirAt(0, 1, w, h);
        const double lat = M_PI * 0.5 - M_PI * (double)1 / (double)(h - 1);
        check(near_v(d, glm::dvec3(std::cos(lat), std::sin(lat), 0), 1e-9),
              "dir: latitude at row 1");
    }

    // 2. equirectLonLat: the inverse (render.cpp's atan2(x, z), asin(y),
    //    with lon unwrapped to [0, 2pi)).
    {
        double lon, lat;
        equirectLonLat(glm::dvec3(1, 0, 0), lon, lat);
        check(near_d(lon, 0.0, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: +X -> (0, 0)");
        equirectLonLat(glm::dvec3(0, 0, -1), lon, lat);
        check(near_d(lon, M_PI / 2, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: -Z -> (pi/2, 0)");
        equirectLonLat(glm::dvec3(-1, 0, 0), lon, lat);
        check(near_d(lon, M_PI, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: -X -> (pi, 0)");
        equirectLonLat(glm::dvec3(0, 0, 1), lon, lat);
        check(near_d(lon, 3.0 * M_PI / 2, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: +Z -> (3pi/2, 0) (unwrapped, not -pi/2)");
        equirectLonLat(glm::dvec3(0, 1, 0), lon, lat);
        check(near_d(lat, M_PI / 2, 1e-12), "lonlat: north -> +pi/2");
        equirectLonLat(glm::dvec3(0, -1, 0), lon, lat);
        check(near_d(lat, -M_PI / 2, 1e-12), "lonlat: south -> -pi/2");
        equirectLonLat(glm::dvec3(0, 0.5, 0.8660254037844387), lon, lat);
        check(near_d(lat, M_PI / 6, 1e-9), "lonlat: latitude 30 deg N");
    }

    // 3. Round-trip: direction -> (lon, lat) -> pixel -> direction
    //    recovers the original (within one pixel cell).
    {
        const double cell_ang = 2.0 * M_PI / (double)w;  // one cell in lon
        bool ok = true;
        for(const glm::dvec3 d : {
                glm::dvec3(0.577, 0.577, 0.577),     // an octant direction
                glm::dvec3(0, -0.7071, 0.7071),      // south, lon 180
                glm::dvec3(-0.7071, 0.3827, 0.5946), // a generic direction
                glm::dvec3(1, 0, 0),                 // lon 0, equator
        }) {
            double lon, lat;
            equirectLonLat(d, lon, lat);
            double u, v;
            equirectPixel(lon, lat, w, h, u, v);
            const int i = (int)(u + 0.5);
            const int j = (int)(v + 0.5);
            const glm::dvec3 back = equirectDirAt(i, j, w, h);
            // Angle between the two (acos of the dot; both are unit).
            const double ang = std::acos(glm::clamp(
                glm::dot(glm::dvec3(d), back), -1.0, 1.0));
            if(ang > 2.0 * cell_ang) { ok = false; }
        }
        check(ok, "roundtrip: dir -> pixel -> dir (within a cell)");
    }

    // 4. equirectPixel: the (lon, lat) -> (u, v) mapping.
    {
        double u, v;
        equirectPixel(0.0, M_PI / 2, w, h, u, v);
        check(near_d(u, 0.0, 1e-12) && near_d(v, 0.0, 1e-12),
              "pixel: (lon 0, north) -> (0, 0) top-left");
        equirectPixel(M_PI, 0.0, w, h, u, v);
        check(near_d(u, w / 2.0, 1e-9) && near_d(v, (h - 1) / 2.0, 1e-9),
              "pixel: (lon 180, equator) -> center");
        equirectPixel(0.0, -M_PI / 2, w, h, u, v);
        check(near_d(u, 0.0, 1e-12) && near_d(v, (double)(h - 1), 1e-9),
              "pixel: (lon 0, south) -> bottom-left");
    }

    // 4b. Grid round-trip: for every non-pole pixel (i, j), the pixel
    //     direction -> (lon, lat) -> pixel is EXACTLY (i, j). (The pole
    //     rows are excluded: every column of a pole row is the same
    //     direction, so lon reads 0 there by convention -- test 1 pins it.)
    {
        bool ok = true;
        for(int j = 1; j < h - 1 && ok; j++) {
            for(int i = 0; i < w; i++) {
                const glm::dvec3 d = equirectDirAt(i, j, w, h);
                double lon, lat;
                equirectLonLat(d, lon, lat);
                double u, v;
                equirectPixel(lon, lat, w, h, u, v);
                if(!near_d(u, (double)i, 1e-9)
                   || !near_d(v, (double)j, 1e-9)) {
                    ok = false;
                    break;
                }
            }
        }
        check(ok, "grid: pixel -> dir -> lonlat -> pixel is exact");
    }

    // 5. surfmapShade: the terminator's light factor.
    {
        const glm::dvec3 n(1, 0, 0);
        check(near_d(surfmapShade(n, glm::dvec3(1, 0, 0)), 1.0, 1e-6),
              "shade: facing the sun -> 1.0");
        check(near_d(surfmapShade(n, glm::dvec3(-1, 0, 0)), 0.30, 1e-6),
              "shade: facing away -> 0.30 (ambient floor)");
        // n . sun = 0: t = clamp(0.05 / 0.20) = 0.25 -> 0.30 + 0.70 * 0.25.
        check(near_d(surfmapShade(n, glm::dvec3(0, 1, 0)),
                     0.30 + 0.70 * 0.25, 1e-6),
              "shade: terminator band at n . sun = 0");
        // Range: always within [0.30, 1.00].
        bool in_range = true;
        for(int k = 0; k < 24 && in_range; k++) {
            const double a = 2.0 * M_PI * k / 24;
            const float s = surfmapShade(glm::dvec3(1, 0, 0),
                                         glm::dvec3(std::cos(a), std::sin(a), 0));
            if(!(s >= 0.30f - 1e-6 && s <= 1.0f + 1e-6)) { in_range = false; }
        }
        check(in_range, "shade: range [0.30, 1.00]");
    }

    // 6. surfmapWraps: consecutive lons cross the antimeridian when their
    //    difference exceeds half a turn.
    {
        check(!surfmapWraps(0.1, 0.2), "wrap: small step -> false");
        check(!surfmapWraps(1.0, 1.1), "wrap: small step mid-map -> false");
        check(surfmapWraps(0.1, 0.9 * 2.0 * M_PI), "wrap: 0.1 -> 324 deg -> true");
        check(surfmapWraps(0.9 * 2.0 * M_PI, 0.1), "wrap: 324 -> 0.1 deg -> true");
        check(!surfmapWraps(0.0, M_PI), "wrap: exactly half a turn -> false");
        check(surfmapWraps(0.0, M_PI + 0.1), "wrap: half a turn + -> true");
    }

    // 7. Cloud-deck UV bridge (cloudShader.vs vs create_atmosphere_mesh).
    //    Mesh param phi = atan2(z, x) from +X toward +Z; surface lon is
    //    pi/2 - phi; bake u = lon/2pi (lon 0 at the LEFT edge); the shader
    //    therefore samples u = 0.25 - phi/2pi. Pin the full chain: the
    //    direction the mesh builds at phi MUST be the direction the bake
    //    writes into the column the shader's u names.
    {
        const int W = 64, H = 32;
        bool ok = true;
        for(int k = 0; k < 16 && ok; k++) {
            const float phi = 2.0f * (float)M_PI * (float)k / 16.0f;
            const float theta = 1.1f;   // mesh COLATITUDE (0 = north)
            // create_atmosphere_mesh's vertex direction at (theta, phi).
            const glm::vec3 mesh_dir(std::sin(theta) * std::cos(phi),
                                     std::cos(theta),
                                     std::sin(theta) * std::sin(phi));
            // cloudShader.vs: u = -phi/2pi
            const double u = -(double)phi / (2.0 * M_PI);
            // Bake column at that u (REPEAT wrap) -> pixel-centre lon.
            // Mesh theta is colatitude; equirectDir wants latitude.
            const double u01 = u - std::floor(u);
            const int pxi = (int)(u01 * W) % W;
            const double lon_bake = ((pxi + 0.5) / (double)W) * 2.0 * M_PI;
            const double lat_bake = 0.5 * M_PI - (double)theta;
            const glm::vec3 bake_dir = equirectDir((float)lon_bake,
                                                   (float)lat_bake);
            // One bake column is 2pi/W in lon; allow a full cell of slack
            // (the bake samples pixel centres, the mesh does not).
            if(glm::length(mesh_dir - bake_dir) > 2.0f * (float)M_PI / (float)W) {
                ok = false;
                break;
            }
            // And the closed form: lon = -phi (mod 2pi).
            double lon, lat;
            equirectLonLat(glm::dvec3(mesh_dir), lon, lat);
            double want = std::fmod(-(double)phi, 2.0 * M_PI);
            if(want < 0.0) { want += 2.0 * M_PI; }
            if(std::fabs(lon - want) > 1e-5) { ok = false; }
        }
        check(ok, "cloud UV: mesh phi -> shader u -> bake column -> dir");
        // The shader constant is NOT the formula above -- a leftover 0.25
        // or 0.75 in cloudShader.vs would not fail the chain. Pin the file.
        {
            FILE *f = std::fopen("res/shaders/cloudShader.vs", "r");
            check(f != nullptr, "cloud UV: cloudShader.vs readable");
            if(f) {
                char buf[4096];
                const size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
                buf[n] = '\0';
                std::fclose(f);
                check(std::strstr(buf, "uv0 = vec2(-uvParam.x, uvParam.y);")
                      != nullptr,
                      "cloud UV: cloudShader.vs samples u = -phi/2pi");
            }
        }
    }

    // 8. Heightmap column round-trip (the layout Heightmap::sample uses).
    //    A synthetic 4x3 grid: sample(equirectDir(lon, lat)) at the grid
    //    points must read that cell exactly (bilinear is exact there).
    {
        auto hm = std::make_shared<Heightmap>();
        hm->w = 4;
        hm->h = 3;
        hm->m = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
        hm->recomputeRange();
        bool ok = true;
        // Skip pole rows: no unique lon there (test 1 pins them), and the
        // round-trip would only pass via float dust in cos(pi/2).
        for(int j = 1; j < hm->h - 1 && ok; j++) {
            for(int i = 0; i < 4 && ok; i++) {
                const glm::dvec3 d = equirectDirAt(i, j, 4, 3);
                const float s = hm->sample(glm::vec3(d));
                const float want = (float)hm->m[(size_t)j * 4 + i];
                if(std::fabs(s - want) > 0.5f) { ok = false; }
            }
        }
        check(ok, "heightmap: sample(equirectDir(i, j)) reads cell (i, j)");
    }

    if(g_failures == 0) {
        std::printf("test_surfmap: all checks passed\n");
        return 0;
    }
    std::printf("test_surfmap: %d check(s) failed\n", g_failures);
    return 1;
}
