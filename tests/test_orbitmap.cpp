// test_orbitmap.cpp -- unit tests for the pure-math part of the orbital map
// (src/orbitmap.h). project() maps a 3D point in the focus body's inertial
// frame to 2D map coordinates by dropping the map normal (+Y) and scaling the
// reference plane (XZ) by meters-per-pixel. Only the header's math is
// exercised, so this needs no rendering and links no imgui / Bullet / GL.

#include "orbitmap.h"

#include <algorithm>
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
   counter-clockwise -- for every path through setPlane. Equivalent to
   e1 x e2 = -n, but stated as the thing a player can see. The derived path
   used e2 = n x e1, so an inclined ship's Orbital view swept the other way
   from its own Equatorial view: same orbit, mirrored, one combo click apart.
   The first line is the orthogonality half -- note it must be dot(n, e1), NOT
   dot(cross(n, e1), e1), which is identically zero for any pair of vectors and
   would test nothing. */
static void expect_sweep(const OrbitMap &s, const char *what) {
    expect_near(glm::dot(s.n, s.e1), 0.0, what);
    expect_near(glm::dot(glm::cross(s.n, s.e1), s.e2), -1.0, what);
}

/* The same claim, SIGN only. The canonical branch keeps a FIXED basis rather
   than one built from n, so e1 is not exactly in-plane unless the normal is
   exactly +-Y: Venus's pole sits 0.85 deg off -Y, which leaves 0.9989 of the
   sweep and 0.015 of e1 out of the plane. That is the branch working as
   designed, not a defect, so what gets pinned here is the thing a player
   actually sees -- which way the orbit travels. */
static void expect_sweep_sign(const OrbitMap &s, const char *what) {
    const double sw = glm::dot(glm::cross(s.n, s.e1), s.e2);
    if(sw >= 0.0) {
        std::printf("FAIL %s: sweep %+.6f, want negative\n", what, sw);
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
        expect_sweep(s, "canonical +Y sweep");
        /* Below the rail plane the fixed basis keeps its axes but flips e2's
           SIGN, so the handedness still reads -n. Venus is the shipped case:
           it spins retrograde, so an equatorial bed's h (the Orbital slot's
           normal) is ~-Y and lands here, while the Equatorial slot pins its
           basis to the focus +X. Before the sign followed n, those two slots
           drew the SAME plane with mirrored sweeps -- the #181 symptom again,
           found by --map-dump rather than by an eyeball. */
        s.setPlane(glm::dvec3(0.0, -1.0, 0.0));
        expect_near(s.e1.x, 1.0, "canonical -Y e1 x");
        expect_near(s.e2.z, -1.0, "canonical -Y e2 z");
        expect_sweep(s, "canonical -Y sweep");
        /* At the edge of the band (8 deg off the rail normal, so |n.Y| = 0.9903
           > 0.99) the fixed basis is 8 deg out of plane by design, so only the
           sweep sign is pinned -- and it still reads -n below the plane. This is
           the Ecliptic slot's case on a near-ecliptic focus; the Orbital slot
           left it in #185. */
        const double r = 8.0 * std::acos(-1.0) / 180.0;
        OrbitMap edge;
        edge.setPlane(glm::dvec3(std::sin(r), std::cos(r), 0.0));
        expect_near(edge.e1.x, 1.0, "canonical band edge keeps e1 fixed");
        expect_sweep_sign(edge, "canonical band edge sweep");
        edge.setPlane(glm::dvec3(std::sin(r), -std::cos(r), 0.0));
        expect_near(edge.e2.z, -1.0, "canonical band edge below the plane");
        expect_sweep_sign(edge, "canonical band edge sweep below");
    }
    {
        /* Venus-shaped focus: pole ~ -Y (177 deg tilt), ship in an equatorial
           bed, so the Equatorial and Orbital slots share one normal. #185 put
           both on the pinned path, so they agree exactly rather than needing a
           sign fix-up on one of them -- which is how #181's symptom appeared,
           when the Orbital slot took the canonical branch instead. Numbers from
           --map-dump on solar_system. */
        const glm::dvec3 pole(-0.014899, -0.998939, 0.043584);
        OrbitMap eq, ob;
        eq.setPlane(pole, glm::dvec3(1, 0, 0));
        ob.setPlane(pole, glm::dvec3(1, 0, 0));
        expect_near(glm::dot(eq.n, ob.n), 1.0, "venus slots share the normal");
        /* Pinned to numbers, not to each other: two setPlane calls with
           identical arguments agree whatever the code does, so dotting their
           results tests nothing (a 30 deg rotation of the pinned e1 left that
           assertion green). These are the values --map-dump prints. */
        expect_close(ob.e1.x, 0.999889, 1e-5, "venus orb e1 x");
        expect_close(ob.e1.y, -0.014885, 1e-5, "venus orb e1 y");
        expect_close(ob.e1.z, 0.000649, 1e-5, "venus orb e1 z");
        expect_close(ob.e2.z, -0.999050, 1e-5, "venus orb e2 z");
        expect_sweep(eq, "venus Equatorial sweep");
        expect_sweep(ob, "venus Orbital sweep");
    }
    {
        /* #185: the Orbital slot's normal sweeping through the rail normal.
           With no x_axis the derived basis hands over to the canonical one at
           8.13 deg, and the two are a quarter turn apart there -- the map
           rotates under the player's cursor mid-burn. With the focus +X pinned
           (what mapPlaneBasis now passes) the basis is (cos t, -sin t, 0) all
           the way through, so it turns by exactly as much as the normal does.
           t = angle of the normal off the rail normal, about +Z. */
        const double kPi = std::acos(-1.0);
        auto basis = [&](double deg, bool pinned) {
            const double r = deg * kPi / 180.0;
            OrbitMap m;
            const glm::dvec3 n(std::sin(r), std::cos(r), 0.0);
            if(pinned) { m.setPlane(n, glm::dvec3(1.0, 0.0, 0.0)); }
            else       { m.setPlane(n); }
            return m;
        };
        // Exactly the rail normal: both spellings must give the same picture, so
        // an equatorial orbit's view does not change at all.
        {
            const OrbitMap a = basis(0.0, false), b = basis(0.0, true);
            expect_near(glm::dot(a.e1, b.e1), 1.0, "equatorial view unchanged");
            expect_near(glm::dot(a.e2, b.e2), 1.0, "equatorial view e2 too");
        }
        // The defect being fixed, as a number: no x_axis, 0.1 deg of slew across
        // 8.13 deg, and the picture turns 90 deg.
        expect_close(glm::dot(basis(8.1, false).e1, basis(8.2, false).e1),
                     0.0, 1e-3, "unpinned basis snaps 90 deg at 8.13 deg");
        // With the x_axis the same slew turns it by 0.1 deg, and e1 tracks the
        // normal exactly (e1 = (cos t, -sin t, 0) for this sweep).
        expect_close(std::acos(basis(8.1, true).e1.x) * 180.0 / kPi,
                     8.1, 1e-9, "pinned e1 tracks the normal");
        double worst_pinned = 0.0;
        for(double d = 0.0; d < 60.0; d += 0.1) {
            const double turn = std::acos(std::min(1.0, std::max(-1.0,
                glm::dot(basis(d, true).e1, basis(d + 0.1, true).e1))))
                * 180.0 / kPi;
            if(turn > worst_pinned) { worst_pinned = turn; }
        }
        if(worst_pinned > 0.2) {
            std::printf("FAIL pinned basis snaps: worst turn %.3f deg over a "
                        "0.1 deg slew\n", worst_pinned);
            ++g_failures;
        }
        /* The 0.44 floor #185 shipped with was the bigger hazard, not the
           singularity: it swapped to the node line 26 deg EARLY, and across that
           swap the two bases sit anywhere from 0 to 180 deg apart depending on
           where the cone is crossed. So an orbit whose h only passed NEAR +-X
           flipped for no reason. Sweep a normal 5 deg off the rail plane all the
           way round -- a polar orbit changing its node longitude, which never
           reaches +-X -- and it must stay smooth: 169.8 deg per 0.1 deg of slew
           with the floor, 1.1 deg without it (tmp/t185polar.cpp). */
        double worst_near_polar = 0.0;
        auto off_polar = [&](double deg) {
            const double r = deg * kPi / 180.0;
            return glm::normalize(glm::dvec3(std::cos(r),
                                             std::sin(5.0 * kPi / 180.0),
                                             std::sin(r)));
        };
        for(double d = 0.0; d < 360.0; d += 0.1) {
            OrbitMap a, b;
            a.setPlane(off_polar(d), glm::dvec3(1.0, 0.0, 0.0));
            b.setPlane(off_polar(d + 0.1), glm::dvec3(1.0, 0.0, 0.0));
            const double turn = std::acos(std::min(1.0, std::max(-1.0,
                glm::dot(a.e1, b.e1)))) * 180.0 / kPi;
            if(turn > worst_near_polar) { worst_near_polar = turn; }
        }
        if(worst_near_polar > 5.0) {
            std::printf("FAIL near-polar sweep snaps: worst turn %.1f deg per "
                        "0.1 deg of slew\n", worst_near_polar);
            ++g_failures;
        }
        /* The singularity itself, pinned so it stays a decision and not an
           accident: this sweep crosses h = +X at 90 deg, where +X has no
           in-plane part left, so e1 = (cos t, -sin t, 0) changes sign with
           cos t -- a full half turn. No rule for picking an in-plane basis is
           continuous over the whole sphere, so SOME crossing must flip; the
           question is only where. The node line flipped at h ~ +-Y, where every
           near-equatorial orbit lives; pinning to +X moves it to h ~ +X. */
        expect_close(glm::dot(basis(89.9, true).e1, basis(90.1, true).e1),
                     -0.999994, 1e-5, "pinned rule flips at h ~ +X");
        expect_close(glm::dot(basis(89.9, false).e1, basis(90.1, false).e1),
                     1.0, 1e-5, "node-line rule is smooth at h ~ +X");
        expect_sweep(basis(64.0, true), "64 deg tilt stays orthonormal");
    }
    {
        /* The consequence a player can click: an equatorial bed's h IS the focus
           pole, so the Equatorial and Orbital slots share a normal and both pin
           to the focus +X -- identical pictures, where the Orbital slot used to
           draw the same plane turned 58.8 deg. Kerbin's pole and the e1 that
           --map-dump now reports. */
        const glm::dvec3 pole(-0.347824, 0.917477, 0.193014);
        OrbitMap eq, ob;
        eq.setPlane(pole, glm::dvec3(1, 0, 0));
        ob.setPlane(pole, glm::dvec3(1, 0, 0));
        // The absolute numbers below are what do the work: eq and ob are the
        // same call, so comparing their bases would prove nothing.
        expect_close(ob.e1.x, 0.937560, 1e-5, "kerbin -eq bed orb e1 x");
        expect_close(ob.e1.y, 0.340373, 1e-5, "kerbin -eq bed orb e1 y");
        expect_close(ob.e1.z, 0.071606, 1e-5, "kerbin -eq bed orb e1 z");
        // What it used to be, kept as a number so nobody "simplifies" the
        // Orbital slot back to a derived basis and calls it equivalent.
        OrbitMap old;
        old.setPlane(pole);
        expect_close(glm::dot(eq.e1, old.e1), 0.517532, 1e-5,
                     "derived Orbital basis was 58.8 deg off Equatorial");
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
        /* Uranus (97.8 deg tilt): its pole sits 8.2 deg from the reference +X,
           so the in-plane part of +X is only 0.147 long. That used to be treated
           as too short to pin screen-x on and fell back to the node line --
           113.2 deg away, so Uranus's Equatorial view turned 113 deg for a
           conditioning worry that does not exist in doubles (1e-16 of pole
           jitter over |t| = 0.147 is 7e-16 rad). It pins now. */
        const glm::dvec3 pole(0.989164, -0.135197, -0.057248);
        OrbitMap s;
        s.setPlane(pole, glm::dvec3(1, 0, 0));
        expect_close(s.e1.x, 0.146818, 1e-5, "uranus eq e1 x");
        expect_close(s.e1.y, 0.910868, 1e-5, "uranus eq e1 y");
        expect_close(s.e1.z, 0.385699, 1e-5, "uranus eq e1 z");
        expect_sweep(s, "uranus eq prograde sense");
        // The node line it replaced, kept so the change in this view is visible
        // in the test rather than a silent edit of three numbers.
        OrbitMap old;
        old.setPlane(pole);
        expect_close(glm::dot(s.e1, old.e1), -0.394, 1e-3,
                     "Uranus moved 113 deg off the node line");
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
