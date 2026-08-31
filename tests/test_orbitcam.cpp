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
// Exits nonzero on a jump, a NaN, a non-closing loop, or a holonomy roll.
//
// Build & run (from repo root):
//   g++ -O2 -std=c++11 -I./src -I./middleware/glm/ tests/test_orbitcam.cpp \
//       src/camera.cpp -o test_orbitcam
//   ./test_orbitcam

#include "camera.h"
#include <cstdio>
#include <cmath>

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
        Camera cam(glm::dvec3(0, 0, 0), 60.0f, 16.0f / 9.0f, 1.0f, 1e13f);
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
        Camera cam(glm::dvec3(0, 0, 0), 60.0f, 16.0f / 9.0f, 1.0f, 1e13f);
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
        Camera cam(glm::dvec3(0, 0, 0), 60.0f, 16.0f / 9.0f, 1.0f, 1e13f);
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
        Camera cam(glm::dvec3(0, 0, 0), 60.0f, 16.0f / 9.0f, 1.0f, 1e13f);
        for (int i = 0; i < 400; i++) { cam.wheel(1.0); }
        printf("after 400 zoom-in notches: distance %.3f (min 2)\n", cam.distance);
        if (cam.distance < 2.0) { bad = true; }
        for (int i = 0; i < 800; i++) { cam.wheel(-1.0); }
        printf("after 800 zoom-out notches: distance %.3g (max 1e9)\n", cam.distance);
        if (cam.distance > 1e9) { bad = true; }
    }

    if (!bad) { printf("\nPASS\n"); }
    return bad ? 1 : 0;
}
