// test_orbitmap.cpp -- unit tests for the pure-math part of the orbital map
// (src/orbitmap.h). project() maps a 3D point in the focus body's inertial
// frame to 2D map coordinates by dropping the map normal (+Y) and scaling the
// reference plane (XZ) by meters-per-pixel. Only the header's math is
// exercised, so this needs no rendering and links no imgui / Bullet / GL.

#include "orbitmap.h"

#include <cmath>

#include <cstdio>

static int g_failures = 0;

static void expect_near(double got, double want, const char *what) {
    if(std::fabs(got - want) > 1e-9) {
        std::printf("FAIL %s: got %g, want %g\n", what, got, want);
        ++g_failures;
    }
}

int main() {
    OrbitMap m;
    m.cx = 100.0;
    m.cy = 200.0;
    m.scale = 50.0;  // meters per pixel

    // The origin (the focus) maps to the map center.
    {
        const glm::dvec2 p = m.project(glm::dvec3(0, 0, 0));
        expect_near(p.x, 100.0, "origin x");
        expect_near(p.y, 200.0, "origin y");
    }

    // X and Z are scaled by m/px; +Y (the map normal) is dropped entirely.
    {
        const glm::dvec2 p = m.project(glm::dvec3(100, 500, -150));
        // x = 100 + 100/50 = 102 ; y = 200 + (-150)/50 = 197 ; the +500 is gone.
        expect_near(p.x, 102.0, "project x");
        expect_near(p.y, 197.0, "project y");
    }

    // px() is the same projection, returned as an ImVec2 (floats).
    {
        const ImVec2 q = m.px(glm::dvec3(100, 0, -150));
        expect_near(q.x, 102.0, "px x");
        expect_near(q.y, 197.0, "px y");
    }

    // contrastingColor(): a light background yields dark ink and vice versa,
    // so the orbit stays visible in both the light and dark ImGui styles.
    {
        // Rec. 709 luminance of a packed ImU32 (0xAABBGGRR) in [0,1].
        auto lum = [](ImU32 c) -> float {
            return (0.2126f * (c & 0xFF) + 0.7152f * ((c >> 8) & 0xFF)
                    + 0.0722f * ((c >> 16) & 0xFF)) / 255.0f;
        };
        const ImU32 onLight = contrastingColor(ImVec4(0.9f, 0.9f, 0.9f, 1.0f));
        const ImU32 onDark  = contrastingColor(ImVec4(0.1f, 0.1f, 0.1f, 1.0f));
        if(lum(onLight) >= 0.5f) {
            std::printf("FAIL contrastingColor: light bg should give dark ink (lum %g)\n",
                        (double)lum(onLight));
            ++g_failures;
        }
        if(lum(onDark) <= 0.5f) {
            std::printf("FAIL contrastingColor: dark bg should give light ink (lum %g)\n",
                        (double)lum(onDark));
            ++g_failures;
        }
    }

    if(g_failures == 0) {
        std::printf("test_orbitmap: all checks passed\n");
        return 0;
    }
    std::printf("test_orbitmap: %d check(s) failed\n", g_failures);
    return 1;
}
