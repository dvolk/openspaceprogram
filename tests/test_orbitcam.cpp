// Pin the orbit camera's over-the-top behavior: pitching past the pole
// must keep the stored up vector turning continuously (a sudden ~90deg
// roll there was the old radial-up "weird" behavior) and the view matrix
// NaN-free. Exits nonzero on a jump or a NaN so `make test` catches a
// regression.
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

int main() {
    printf("=== Orbit camera following a body at the ORIGIN ===\n");
    printf("Pitch from 0deg past the pole. The up vector must turn\n");
    printf("continuously (no sudden roll) and the view must stay finite.\n\n");

    Camera cam(glm::dvec3(0, 0, 0), 60.0f, 16.0f / 9.0f, 1.0f, 1e13f);
    cam.distance = 1000.0;
    cam.ComputeView();

    // Drive the camera with its own Pitch() (incremental), like the game's
    // RMB-drag does per frame.
    const int steps = 400;
    const double step = 100.0 * M_PI / 180.0 / steps;
    glm::dvec3 prev_up = cam.up;
    double max_jump = 0.0;
    bool bad = false;
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

    // A step pitches 0.25deg; the up vector may not lag or jump by more
    // than a degree per step anywhere, including over the pole.
    printf("\nmax up jump per 0.25deg pitch step: %.3f deg\n", max_jump);
    if (max_jump > 1.0) {
        printf("FAIL: up vector jumped %.2f deg in one step\n", max_jump);
        bad = true;
    }

    // Yaw (turntable) is a full 360deg loop: the offset must come back to
    // where it started and the view must stay finite the whole way.
    const glm::dvec3 off0 = cam.pos - glm::dvec3(0, 0, 0);
    for (int i = 0; i < 400; i++) {
        cam.RotateY(0.9 * M_PI / 180.0);
        cam.ComputeView();
        if (isbadm(cam.view)) { bad = true; }
    }
    const double yaw_err = glm::length(glm::normalize(cam.pos) - glm::normalize(off0));
    printf("after a 360deg yaw loop: offset returned within %.2g (0 = exact)\n", yaw_err);
    if (yaw_err > 1e-9) { bad = true; }

    // Wheel zoom is proportional and clamped: it can never cross the focus
    // (old: distance -= amt*sqrt(distance), unclamped -> flip through).
    for (int i = 0; i < 400; i++) { cam.wheel(1.0); }
    printf("after 400 zoom-in notches: distance %.3f (min 2)\n", cam.distance);
    if (cam.distance < 2.0) { bad = true; }
    for (int i = 0; i < 800; i++) { cam.wheel(-1.0); }
    printf("after 800 zoom-out notches: distance %.3g (max 1e9)\n", cam.distance);
    if (cam.distance > 1e9) { bad = true; }

    if (!bad) { printf("PASS\n"); }
    return bad ? 1 : 0;
}
