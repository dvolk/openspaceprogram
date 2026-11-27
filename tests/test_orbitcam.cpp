// Pin the orbit camera's turntable behavior. The camera is a turntable:
//   - pitch is clamped just short of the pole (looking straight down the
//     ref up, where the projected-up would vanish), so pitching up over the
//     top keeps the stored up turning continuously (no sudden roll, no NaN)
//     and then stops at the clamp.
//   - the screen-up is a pure function of WHERE the camera is, not of the
//     yaw/pitch path taken to get there (no trackball holonomy). A closed
//     yaw/pitch loop that returns to the same offset must leave the up
//     unchanged -- the old trackball rolled the view by the loop's enclosed
//     solid angle (the "cockpit chairs tilt when I come back to the same
//     view" bug).
//   - fov is clamped to the range the CLI and the Settings slider use
//     ([10,120]deg), including a NaN, and setAspect STORES the aspect, so a
//     later setFov cannot rebuild the projection or the frustum half-angles
//     with a stale one.
//   - the orthographic box the VAB's six photo views (keys 5..0) are framed with:
//     scales in world metres, no perspective divide, depth landing on the
//     renderer's reverse-Z [0,1] window, cull angles opened so the draw pass
//     fails open, the fit keeping the SHORTER screen axis whole, and the wheel
//     resizing the box only while ortho is live.
// Exits nonzero on a jump, a NaN, a non-closing loop, a holonomy roll, an
// escaped fov, or a stale aspect.
//
// Build & run (from repo root):
/*   g++ -O2 -std=c++20 -I./src -I./middleware/glm/ tests/test_orbitcam.cpp \
       src/camera.cpp -o test_orbitcam
     ./test_orbitcam */

#include "camera.h"
#include <cstdio>
#include <cmath>
#include <limits>

static void printvec(const char* n, glm::dvec3 v) {
    printf("  %-8s (% .4f % .4f % .4f)\n", n, v.x, v.y, v.z);
}
static bool isbadm(glm::dmat4 m) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (glm::isinf(m[i][j]) || glm::isnan(m[i][j])) return true;
    return false;
}
static double angledeg(glm::dvec3 a, glm::dvec3 b) {
    return glm::degrees(std::acos(glm::clamp(glm::dot(a, b), -1.0, 1.0)));
}
static const double deg = M_PI / 180.0;

int main() {
    bool bad = false;

    printf("=== Orbit camera following a body at the ORIGIN ===\n");
    printf("Pitch toward the pole. The up vector must turn continuously\n");
    printf("(no sudden roll), stay finite, and clamp just short of the pole.\n\n");

    {
        Camera cam(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        cam.distance = 1000.0;
        cam.ComputeView();

        // Drive the camera with its own Pitch() (incremental), like the
        // game's RMB-drag does per frame. 100deg total; it must clamp before
        // the pole so the up never vanishes.
        const int steps = 400;
        const double step = 100.0 * deg / steps;
        glm::dvec3 prev_up = cam.up;
        double max_jump = 0.0;
        for (int i = 1; i <= steps; i++) {
            cam.Pitch(step);
            cam.ComputeView();
            if (isbadm(cam.view)) {
                printf("  ** NaN/Inf in view matrix at step %d **\n", i);
                bad = true;
            }
            const double jump = angledeg(prev_up, cam.up);
            if (jump > max_jump) { max_jump = jump; }
            if (i % 40 == 0) {
                printf("pitch %3.0f deg (up moved %.3f deg this step):\n",
                       i * glm::degrees(step), jump);
                printvec("pos", cam.pos);
                printvec("forward", cam.forward);
                printvec("up", cam.up);
            }
            prev_up = cam.up;
        }
        // A step pitches 0.25deg; the up vector may not jump by more than a
        // degree per step anywhere, including as it approaches the pole.
        printf("max up jump per 0.25deg pitch step: %.3f deg\n", max_jump);
        if (max_jump > 1.0) {
            printf("FAIL: up vector jumped %.2f deg in one step\n", max_jump);
            bad = true;
        }
        // The pole clamp: we pitched 100deg total, but the camera must not
        // run past the pole (forward parallel to the ref up, where the
        // projected-up vanishes). orbitPitch is capped just short of 90deg.
        printf("final orbitPitch = %.4f rad (%.2f deg; clamp just short of 90deg)\n",
               cam.orbitPitch, glm::degrees(cam.orbitPitch));
        if (cam.orbitPitch > 1.52 + 1e-9) {
            printf("FAIL: pitch ran past the clamp into the pole\n");
            bad = true;
        }
        // The up at the clamped position must still be a well-defined unit
        // vector (not vanishing, not NaN).
        const double up_len = glm::length(cam.up);
        if (glm::isnan(up_len) || up_len < 0.5 || up_len > 1.5) {
            printf("FAIL: up at the clamp is degenerate (%.4f)\n", up_len);
            bad = true;
        }
    }

    printf("\n=== Holonomy: a closed yaw/pitch loop must not roll the view ===\n");
    printf("Two paths to the same offset must give the same up. The old\n");
    printf("trackball rolled by the loop's enclosed solid angle here.\n\n");

    {
        Camera cam(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        cam.distance = 1000.0;
        cam.ComputeView();
        const glm::dvec3 up_start = cam.up;
        const glm::dvec3 off_start = cam.pos;   // focus is the origin

        // Path A: pitch straight out to (yaw=0, pitch=30).
        cam.Pitch(30.0 * deg);
        cam.ComputeView();
        const glm::dvec3 up_A = cam.up;
        const glm::dvec3 off_A = cam.pos;

        // Path B: the SAME final offset reached a different way -- yaw out,
        // pitch, yaw back, (pitch is already there). Ends at yaw=0, pitch=30.
        cam.Pitch(-30.0 * deg);   // back to start
        cam.ComputeView();
        cam.RotateY(60.0 * deg);
        cam.ComputeView();
        cam.Pitch(30.0 * deg);
        cam.ComputeView();
        cam.RotateY(-60.0 * deg);
        cam.ComputeView();
        const glm::dvec3 up_B = cam.up;
        const glm::dvec3 off_B = cam.pos;

        const double off_err = glm::length(glm::normalize(off_B) -
                                           glm::normalize(off_A));
        const double up_err = angledeg(up_A, up_B);
        printf("two paths to the same offset: offset err %.2g, up err %.3f deg\n",
               off_err, up_err);
        if (off_err > 1e-9) {
            printf("FAIL: the two paths did not land on the same offset\n");
            bad = true;
        }
        if (up_err > 1e-6) {
            printf("FAIL: same offset, different up -> trackball holonomy\n");
            bad = true;
        }

        // Full closed loop back to the exact starting offset: the up must be
        // the starting up (zero net roll).
        cam.Pitch(-30.0 * deg);
        cam.ComputeView();
        const double loop_roll = angledeg(cam.up, up_start);
        const double loop_off = glm::length(glm::normalize(cam.pos) -
                                            glm::normalize(off_start));
        printf("closed loop: offset err %.2g, net roll %.3f deg\n",
               loop_off, loop_roll);
        if (loop_off > 1e-9 || loop_roll > 1e-6) {
            printf("FAIL: closed loop left the view rolled\n");
            bad = true;
        }
    }

    printf("\n=== Yaw 360deg loop closes and stays finite ===\n");
    {
        Camera cam(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        cam.distance = 1000.0;
        cam.ComputeView();
        const glm::dvec3 off0 = cam.pos;
        for (int i = 0; i < 400; i++) {
            cam.RotateY(0.9 * deg);
            cam.ComputeView();
            if (isbadm(cam.view)) { bad = true; }
        }
        const double yaw_err = glm::length(glm::normalize(cam.pos) -
                                           glm::normalize(off0));
        printf("after a 360deg yaw loop: offset returned within %.2g (0 = exact)\n", yaw_err);
        if (yaw_err > 1e-9) { bad = true; }
    }

    printf("\n=== Wheel zoom is proportional and clamped ===\n");
    {
        Camera cam(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        for (int i = 0; i < 400; i++) { cam.wheel(1.0); }
        printf("after 400 zoom-in notches: distance %.3f (min 2)\n", cam.distance);
        if (cam.distance < 2.0) { bad = true; }
        for (int i = 0; i < 800; i++) { cam.wheel(-1.0); }
        printf("after 800 zoom-out notches: distance %.3g (max 1e9)\n", cam.distance);
        if (cam.distance > 1e9) { bad = true; }
    }

    printf("\n=== FOV is clamped to the CLI/slider range ===\n");
    {
        // One range everywhere: --fov validates [10,120] (cli.cpp), the Settings
        // slider is [10,120], and settings_read clamps the file's value to the
        // same. Camera's clamp is the last line of defence, and it matters
        // because outside (0,180)deg tan(fov/2) goes negative: the projection's
        // scale flips sign AND computeFrustumAngles' half-angle goes negative
        // with it, so every patchCulled() test in the draw pass rejects and the
        // world renders empty with nothing anywhere reporting an error.
        Camera cam(glm::dvec3(0, 0, 0), 200.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        printf("fov 200deg -> %.1f deg, projection x-scale %.6f\n",
               glm::degrees(cam.fov), cam.projection[0][0]);
        if (std::fabs(glm::degrees(cam.fov) - 120.0) > 1e-3) {
            printf("FAIL: 200deg did not clamp to the 120deg the slider allows\n");
            bad = true;
        }
        if (cam.projection[0][0] <= 0.0f || cam.projection[1][1] <= 0.0f) {
            printf("FAIL: a non-positive projection scale inverts the view\n");
            bad = true;
        }
        // The cull's side-plane test needs the half-angles inside (0,90)deg:
        // a negative sin/cos rejects every patch.
        if (cam.frustumSinX <= 0.0 || cam.frustumCosX <= 0.0 ||
            cam.frustumSinY <= 0.0 || cam.frustumCosY <= 0.0) {
            printf("FAIL: frustum half-angles left (0,90)deg\n");
            bad = true;
        }
        cam.setFov(-30.0f * deg);
        printf("setFov(-30deg) -> %.1f deg (floor 10)\n", glm::degrees(cam.fov));
        if (std::fabs(glm::degrees(cam.fov) - 10.0) > 1e-3 ||
            cam.frustumSinX <= 0.0 || cam.frustumCosX <= 0.0) {
            printf("FAIL: setFov escaped the range or the half-angles went negative\n");
            bad = true;
        }
        // std::clamp passes NaN straight through (both comparisons are false),
        // so the guard has to catch it before tan() does: a NaN fov is the same
        // silent empty world, only with a NaN projection instead of a negative
        // one. It falls back to the boot default rather than to a bound.
        cam.setFov(std::numeric_limits<float>::quiet_NaN());
        printf("setFov(NaN) -> %.1f deg (falls back to the default)\n",
               glm::degrees(cam.fov));
        if (!std::isfinite(cam.fov) ||
            std::fabs(glm::degrees(cam.fov) - 60.0) > 1e-3) {
            printf("FAIL: a NaN fov reached the projection\n");
            bad = true;
        }
    }

    printf("\n=== setAspect stores the aspect, not just uses it ===\n");
    {
        // Both the projection and the frustum angles read the `aspect`
        // member. setAspect used to pass its argument straight into the
        // projection without storing it, so a LATER setFov rebuilt both with
        // the boot aspect: resize the window, then move the FOV slider, and
        // the horizontal fov snapped back to its launch value.
        Camera cam(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        cam.setAspect(4.0f / 3.0f);
        cam.setFov(60.0f * deg);
        // projection col0 row0 = (1/tan(fov/2)) / aspect (fov is vertical).
        const double want = 1.0 / (std::tan(30.0 * deg) * (4.0 / 3.0));
        const double boot = 1.0 / (std::tan(30.0 * deg) * (16.0 / 9.0));
        const double got = cam.projection[0][0];
        printf("4/3 then setFov: x-scale %.6f (want %.6f, a stale 16/9 gives %.6f)\n",
               got, want, boot);
        if (!std::isfinite(got) || !std::isfinite(want) || std::fabs(got - want) > 1e-4) {
            printf("FAIL: setFov rebuilt the projection with a stale aspect\n");
            bad = true;
        }
        // And the cull's horizontal half-angle follows the current aspect.
        const double want_hx = std::atan(std::tan(30.0 * deg) * (4.0 / 3.0));
        if (!std::isfinite(cam.frustumSinX) || std::fabs(cam.frustumSinX - std::sin(want_hx)) > 1e-6) {
            printf("FAIL: frustumSinX does not follow the current aspect\n");
            bad = true;
        }
    }

    printf("\n=== Orthographic box: the matrix, the cull, the fit, the wheel ===\n");
    {
        /* The VAB's six photo views (keys 5..0) rest on this matrix. It has to
           agree with how the renderer is set up -- reverse-Z, with
           glClipControl(GL_ZERO_TO_ONE) and glDepthFunc(GEQUAL) (display.cpp) --
           so the NEAR edge of the depth window lands on 1 and the FAR edge on 0.
           Read that backwards and every ortho picture is an empty field: the
           subject fails GEQUAL and nothing anywhere reports an error. */
        Camera cam(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        const double hh = 5.0, nz = 2.0, fz = 40.0, asp = 16.0 / 9.0;
        cam.setOrtho(true, hh, nz, fz);
        const double hw = hh * asp;
        printf("box %.1f x %.1f m, depth window [%.1f, %.1f] m\n", 2 * hw, 2 * hh, nz, fz);
        if (std::fabs((double)cam.projection[0][0] - 1.0 / hw) > 1e-5 ||
            std::fabs((double)cam.projection[1][1] - 1.0 / hh) > 1e-5) {
            printf("FAIL: the x/y scales are not 1 over the frame's half-extents\n");
            bad = true;
        }
        /* w = 1: no perspective divide. This is the point of the whole feature --
           parallel spar edges keep their width from the near end of the hull to
           the far one, so a half-metre build error reads as half a metre. */
        if (std::fabs((double)cam.projection[3][3] - 1.0) > 1e-6 ||
            std::fabs((double)cam.projection[2][3]) > 1e-6) {
            printf("FAIL: the ortho matrix still carries a perspective divide\n");
            bad = true;
        }
        auto depth = [&](double zv) {
            return (double)cam.projection[2][2] * zv + (double)cam.projection[3][2];
        };
        printf("depth: view z -%.1f -> %.4f, -%.1f -> %.4f\n", nz, depth(-nz), fz, depth(-fz));
        if (std::fabs(depth(-nz) - 1.0) > 1e-5 || std::fabs(depth(-fz)) > 1e-5) {
            printf("FAIL: the depth window does not map onto [0,1] reverse-Z\n");
            bad = true;
        }
        if (!(depth(-nz) > depth(-fz))) {
            printf("FAIL: ortho depth runs the wrong way for GL_GEQUAL\n");
            bad = true;
        }
        if (!(depth(-2.0 * fz) < 0.0)) {
            printf("FAIL: past the far edge the depth still lands inside [0,1]\n");
            bad = true;
        }
        /* The draw cull is `zv*sin - |xv|*cos >= 0`, which cannot describe a
           parallel-walled frame, so ortho opens the side planes to the whole
           half-space in front of the camera. It must therefore never reject on a
           lateral coordinate: fail-open, spending the CPU skip rather than the
           geometry. A perspective half-angle left in place would cut the wings
           off a fitted hull. */
        if (cam.frustumSinX != 1.0 || cam.frustumCosX != 0.0 ||
            cam.frustumSinY != 1.0 || cam.frustumCosY != 0.0) {
            printf("FAIL: ortho left perspective frustum angles behind\n");
            bad = true;
        }
        for (double xv : { 0.0, 1.0e6 }) {
            const double keep = 1.0 * cam.frustumSinX - std::fabs(xv) * cam.frustumCosX;
            if (keep < 0.0) {
                printf("FAIL: an ortho patch at x=%.0f was culled\n", xv);
                bad = true;
            }
        }
        /* Fitting keeps the SHORTER screen axis whole, which is what "fitted" has
           to mean on a wide window: the box then holds the sphere on both axes. */
        const double fit = cam.fitHalfHeight(10.0);
        printf("fitHalfHeight(10 m) -> %.3f, so a %.1f m wide box\n", fit, 2 * fit * asp);
        if (fit < 10.0 || fit > 10.001) {
            printf("FAIL: a 10 m sphere does not fit the ortho box\n");
            bad = true;
        }
        // A resize must rebuild the live box with the live aspect: fitHalfHeight
        // only RECOMMENDS a size, so the projection is what proves the box moved.
        cam.setAspect(0.5f);
        if (std::fabs((double)cam.projection[0][0] - 1.0 / (hh * 0.5)) > 1e-5 ||
            std::fabs((double)cam.projection[1][1] - 1.0 / hh) > 1e-5) {
            printf("FAIL: setAspect did not rebuild the ortho box (x %.4f y %.4f)\n",
                   (double)cam.projection[0][0], (double)cam.projection[1][1]);
            bad = true;
        }
        const double fit2 = cam.fitHalfHeight(10.0);
        if (fit2 <= 10.0) {
            printf("FAIL: a portrait frame's fit does not bind on its width (%.3f)\n", fit2);
            bad = true;
        }
        cam.setAspect((float)asp);

        /* The wheel: under ortho there is no distance that changes a subject's
           size, so it scales the box. Same proportional step and same sign as the
           orbit zoom, or the two zooms feel like different controls. */
        const double before = cam.orthoHalfH;
        cam.toFree();
        cam.wheel(1.0);
        printf("wheel +1 (free, ortho): half-height %.4f -> %.4f\n", before, cam.orthoHalfH);
        if (std::fabs(cam.orthoHalfH - before * std::exp(-0.25)) > 1e-6) {
            printf("FAIL: the ortho wheel did not take the orbit camera's step\n");
            bad = true;
        }
        for (int i = 0; i < 300; i++) { cam.wheel(1.0); }
        if (cam.orthoHalfH < 0.25) {
            printf("FAIL: the ortho box zoomed below a part's own triangles (%.3f)\n", cam.orthoHalfH);
            bad = true;
        }
        for (int i = 0; i < 900; i++) { cam.wheel(-1.0); }
        if (cam.orthoHalfH > 5.0e4) {
            printf("FAIL: the ortho box zoomed past a body's horizon (%.3g)\n", cam.orthoHalfH);
            bad = true;
        }
        // Orbit mode keeps its own zoom: the box is the VAB photo's business.
        cam.setOrtho(true, 5.0, nz, fz);
        cam.toOrbit(glm::dvec3(0, 0, 0));
        cam.wheel(1.0);
        if (std::fabs(cam.orthoHalfH - 5.0) > 1e-9) {
            printf("FAIL: the orbit wheel resized the ortho box\n");
            bad = true;
        }
        // And a perspective free camera must not have its box touched either.
        Camera p(glm::dvec3(0, 0, 0), 60.0f * deg, 16.0f / 9.0f, 1.0f, 1e13f);
        p.toFree();
        p.setOrtho(false, 20.0, 0.05, 1000.0);
        for (int i = 0; i < 5; i++) { p.wheel(1.0); }
        if (std::fabs(p.orthoHalfH - 20.0) > 1e-9) {
            printf("FAIL: a perspective free camera resized an unused ortho box\n");
            bad = true;
        }
    }

    if (!bad) { printf("\nPASS\n"); }
    return bad ? 1 : 0;
}
