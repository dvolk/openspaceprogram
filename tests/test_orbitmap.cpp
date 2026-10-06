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

// For expectations copied from shipped-system geometry: the inputs are only
// quoted to ~6 digits, so the derived basis can't be pinned to 1e-9.
static void expect_close(double got, double want, double tol, const char *what) {
    if(std::fabs(got - want) > tol) {
        std::printf("FAIL %s: got %g, want %g (tol %g)\n", what, got, want, tol);
        ++g_failures;
    }
}

/* The sweep sense (#181). A prograde body moves along n x r_hat, so at +e1 its
   screen velocity is (n x e1) read on (e1, e2): it must come out (0, -1) --
   counter-clockwise -- for every path that builds the basis FROM the normal.
   Equivalent to e1 x e2 = -n, but stated as the thing a player can see. The
   derived path used e2 = n x e1, so an inclined ship's Orbital view swept the
   other way from its own Equatorial view: same orbit, mirrored, one combo
   click apart. */
static void expect_sweep(const OrbitMap &s, const char *what) {
    expect_near(glm::dot(glm::cross(s.n, s.e1), s.e1), 0.0, what);
    expect_near(glm::dot(glm::cross(s.n, s.e1), s.e2), -1.0, what);
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

    // bodyRadiusPx(): a world radius in pixels (radius/scale), floored at
    // min_px so a body stays a visible dot at system scale.
    {
        expect_near(m.bodyRadiusPx(1000.0, 0.0f), 20.0, "bodyRadiusPx exact");
        expect_near(m.bodyRadiusPx(100.0, 0.0f), 2.0, "bodyRadiusPx no floor");
        expect_near(m.bodyRadiusPx(100.0, 3.0f), 3.0, "bodyRadiusPx below floor");
        expect_near(m.bodyRadiusPx(150.0, 3.0f), 3.0, "bodyRadiusPx at floor");
        expect_near(m.bodyRadiusPx(1000.0, 3.0f), 20.0, "bodyRadiusPx above floor");
        expect_near(m.bodyRadiusPx(0.0, 3.0f), 3.0, "bodyRadiusPx zero radius");
        expect_near(m.bodyRadiusPx(-100.0, 3.0f), 3.0, "bodyRadiusPx negative radius");
    }

    // setPlane(): with no x_axis a +Y normal keeps the canonical X/Z basis. An
    // explicit x_axis pins screen-x INSIDE the plane, so an equatorial view
    // shares its "east" with the ecliptic view instead of inventing one from
    // the normal (issue #173), and its handedness (e2 = e1 x n) matches the
    // canonical case.
    {
        OrbitMap s;
        s.setPlane(glm::dvec3(0, 1, 0));
        expect_near(s.e1.x, 1.0, "derived +Y e1 x");
        expect_near(s.e2.z, 1.0, "derived +Y e2 z");
        // Same normal with x pinned to (1,0,0) must give the SAME basis, so an
        // untilted focus (every shipped star) draws exactly as before.
        s.setPlane(glm::dvec3(0, 1, 0), glm::dvec3(1, 0, 0));
        expect_near(s.e1.x, 1.0, "pinned +Y e1 x");
        expect_near(s.e2.z, 1.0, "pinned +Y e2 z");
    }
    {
        // A 30 deg tilt about +Z: pole (sin, cos, 0), node line (cos, -sin, 0).
        const double t = std::acos(-1.0) / 6.0;
        const double st = std::sin(t), ct = std::cos(t);
        OrbitMap s;
        s.setPlane(glm::dvec3(st, ct, 0.0));
        expect_near(s.e1.z, -1.0, "derived tilt e1 z");   // Y x n -> -Z
        expect_near(s.e2.x, ct, "derived tilt e2 x");     // e1 x n
        expect_near(s.e2.y, -st, "derived tilt e2 y");
        expect_sweep(s, "derived tilt sweep");
        s.setPlane(glm::dvec3(st, ct, 0.0), glm::dvec3(ct, -st, 0.0));
        expect_near(s.e1.x, ct, "pinned tilt e1 x");      // the node line
        expect_near(s.e1.y, -st, "pinned tilt e1 y");
        expect_near(s.e2.z, 1.0, "pinned tilt e2 z");     // e1 x n
        expect_sweep(s, "pinned tilt sweep");
        // An x_axis parallel to the normal is unusable: fall back to derived.
        s.setPlane(glm::dvec3(st, ct, 0.0), glm::dvec3(st, ct, 0.0));
        expect_near(s.e1.z, -1.0, "degenerate x_axis falls back");
        expect_sweep(s, "degenerate x_axis sweep");
    }
    {
        /* Every normal the game can hand setPlane(), through both the derived
           and the pinned path -- including normals below the rail plane (a
           ship that has slewed past retrograde). The sweep must read the same
           way at 10 deg from +Y and at 170. */
        const double kAng[] = { 0.17, 1.0, 1.4, 2.0, 3.0, -1.2 };
        for(const double ang : kAng) {
            const glm::dvec3 n = glm::normalize(
                glm::dvec3(std::sin(ang), std::cos(ang), 0.2 * std::sin(3.0 * ang)));
            char what[64];
            OrbitMap d;
            d.setPlane(n);
            snprintf(what, sizeof what, "derived sweep ang %.2f", ang);
            expect_sweep(d, what);
            /* An x_axis that is exactly IN the plane (cross(n, ref) is, by
               construction), so its in-plane part has length 1 and the pinned
               path always runs. (Using Y x n here instead lands within 0.17 of
               the normal at the near-polar angles and quietly falls back to
               derived -- which is how this test first fooled itself.) */
            const glm::dvec3 ref = (std::fabs(n.z) < 0.5) ? glm::dvec3(0.0, 0.0, 1.0)
                                                          : glm::dvec3(1.0, 0.0, 0.0);
            OrbitMap p;
            p.setPlane(n, glm::cross(n, ref));
            snprintf(what, sizeof what, "pinned sweep ang %.2f", ang);
            expect_sweep(p, what);
        }
    }
    {
        /* The claim #181 is about, in the game's own terms: a Kerbin-ish focus
           (23.44 deg tilt) with a ship inclined 45 deg to the system plane.
           All three combo slots then take a DIFFERENT path -- Equatorial pins
           the basis, Ecliptic takes the canonical one, Orbital derives it --
           and all three must draw that orbit sweeping the same way. Before
           #181 the Orbital view alone read +1. */
        const glm::dvec3 pole(-0.347824, 0.917477, 0.193014);
        const double t = std::acos(-1.0) / 4.0;
        OrbitMap eq, ec, ob;
        eq.setPlane(pole, glm::dvec3(1, 0, 0));
        ec.setPlane(glm::dvec3(0.0, 1.0, 0.0));
        ob.setPlane(glm::dvec3(std::sin(t), std::cos(t), 0.0));
        expect_sweep(eq, "combo Equatorial sweep");
        expect_sweep(ec, "combo Ecliptic sweep");
        expect_sweep(ob, "combo Orbital sweep");
    }
    {
        /* The canonical branch is a FIXED screen basis for near-polar normals,
           not one built from n: it stays put as a normal crosses +Y or -Y, so
           the picture does not mirror mid-slew. That is why the sweep check
           above does not cover it for a normal pointing below the plane, and
           the fixed basis is pinned here so changing it is a decision rather
           than a surprise. */
        OrbitMap s;
        s.setPlane(glm::dvec3(0.0, 1.0, 0.0));
        expect_near(s.e1.x, 1.0, "canonical +Y e1 x");
        expect_near(s.e2.z, 1.0, "canonical +Y e2 z");
        s.setPlane(glm::dvec3(0.0, -1.0, 0.0));
        expect_near(s.e1.x, 1.0, "canonical -Y e1 x");
        expect_near(s.e2.z, 1.0, "canonical -Y e2 z");
    }
    {
        // The map's real call: an equatorial plane (Kerbin's pole, from
        // ksp_system.json's 23.44 deg tilt) with the focus frame's +X as the
        // screen-x reference.
        const glm::dvec3 pole(-0.347824, 0.917477, 0.193014);
        OrbitMap s;
        s.setPlane(pole, glm::dvec3(1, 0, 0));
        expect_close(s.e1.x, 0.937560, 1e-5, "kerbin eq e1 x");
        expect_close(s.e1.y, 0.340373, 1e-5, "kerbin eq e1 y");
        expect_close(s.e1.z, 0.071606, 1e-5, "kerbin eq e1 z");
        // screen-x stays 20 deg from the ecliptic view's +X: switching planes
        // tilts the picture by the axial tilt, it does not rotate it. (Pinning
        // to the equator frame's own +X instead put it 143 deg away.)
        expect_close(glm::dot(s.e1, glm::dvec3(1, 0, 0)), 0.937560, 1e-5,
                     "kerbin eq shares east");
        expect_sweep(s, "kerbin eq prograde sense");
    }
    {
        // Uranus (97.8 deg tilt): its pole sits 8 deg from the reference +X, so
        // projecting +X into the equator plane leaves only 0.15 of direction --
        // too little to pin screen-x on. Fall back to the node line.
        const glm::dvec3 pole(0.989164, -0.135197, -0.057248);
        OrbitMap s;
        s.setPlane(pole, glm::dvec3(1, 0, 0));
        expect_close(s.e1.x, -0.057778, 1e-5, "uranus eq e1 x");
        expect_close(s.e1.y, 0.0, 1e-5, "uranus eq e1 y");
        expect_close(s.e1.z, -0.998329, 1e-5, "uranus eq e1 z");
        expect_sweep(s, "uranus eq prograde sense");
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
