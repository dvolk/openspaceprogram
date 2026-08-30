// Trace OrbitCamera behavior as you pitch, to pin down the "weird" behavior
// when looking radially out/in.
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

// Faithfully drive the camera with its own Pitch() (incremental), accumulating
// a total pitch of `totalDeg` degrees.
static void pitchCam(Camera& cam, double totalDeg) {
    cam.orient = glm::dmat3(1.0);
    cam.ComputeView();   // refresh forward/up from the clean identity orient
    const int steps = 400;
    double step = totalDeg * M_PI / 180.0 / steps;
    for (int i = 0; i < steps; i++) {
        cam.Pitch(step);
        cam.ComputeView();   // update forward/up each step, as the game does per frame
    }
}

int main() {
    printf("=== Orbit camera following a body at the ORIGIN ===\n");
    printf("Pitch from 0deg toward the pole. A sudden ~90deg ROLL of the up\n");
    printf("vector is the reported 'weird' behavior.\n\n");

    Camera cam(glm::dvec3(0, 0, 0), 60.0f, 16.0f / 9.0f, 1.0f, 1e13f);
    cam.distance = 1000.0;

    for (int deg = 0; deg <= 100; deg += 10) {
        pitchCam(cam, deg);
        printf("pitch %3d deg:\n", deg);
        printvec("pos", cam.pos);
        printvec("forward", cam.forward);
        printvec("up", cam.up);
        if (isbadm(cam.view)) printf("  ** NaN/Inf in view matrix **\n");
        printf("\n");
    }

    return 0;
}
