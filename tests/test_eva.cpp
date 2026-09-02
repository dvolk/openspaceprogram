// Pin the pure EVA control-law geometry (src/evamath.h, glm-only): the
// Rodrigues rotation + its axis-angle decomposition (including the 180-deg
// fallback), the camera / upright target bases (orthonormal, det +1, the
// degenerate-hint guards), and the screen-axis helpers.
//
// Build & run (from repo root):
//   g++ -O2 -std=c++11 -I./src -I./middleware/glm/ tests/test_eva.cpp \
//       -o test_eva
//   ./test_eva

#include "evamath.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond) do { \
    if(!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while(0)

static double mlen(const glm::dmat3 &m) {
    double s = 0;
    for(int c = 0; c < 3; c++) { s += glm::length2(m[c]); }
    return std::sqrt(s);
}

int main() {
    // rotAbout: a quarter turn about +z takes x to y; axis-angle round-trip
    {
        const glm::dmat3 R = rotAbout(glm::dvec3(0, 0, 1), M_PI / 2);
        CHECK(glm::length(R * glm::dvec3(1, 0, 0) - glm::dvec3(0, 1, 0)) < 1e-9);
        glm::dvec3 axis;
        const double ang = evaRotAxisAngle(R, axis);
        CHECK(std::fabs(ang - M_PI / 2) < 1e-9);
        CHECK(glm::length(axis - glm::dvec3(0, 0, 1)) < 1e-9);
        CHECK(mlen(rotAbout(axis, ang) - R) < 1e-9);
    }

    // identity: angle 0, zero axis
    {
        glm::dvec3 axis(1, 2, 3);
        const double ang = evaRotAxisAngle(glm::dmat3(1.0), axis);
        CHECK(ang < 1e-9);
        CHECK(glm::length(axis) < 1e-9);
    }

    // 180 deg about each principal axis: the antisymmetric part vanishes,
    // the diagonal fallback must still reproduce R
    for(int i = 0; i < 3; i++) {
        glm::dvec3 a(0.0);
        a[i] = 1.0;
        const glm::dmat3 R = rotAbout(a, M_PI);
        glm::dvec3 axis;
        const double ang = evaRotAxisAngle(R, axis);
        CHECK(std::fabs(ang - M_PI) < 1e-6);
        CHECK(mlen(rotAbout(axis, ang) - R) < 1e-6);
    }

    // arbitrary axis/angle round-trip
    {
        const glm::dvec3 a = glm::normalize(glm::dvec3(0.3, -1.0, 0.5));
        const glm::dmat3 R = rotAbout(a, 1.234);
        glm::dvec3 axis;
        const double ang = evaRotAxisAngle(R, axis);
        CHECK(std::fabs(ang - 1.234) < 1e-9);
        CHECK(glm::length(axis - a) < 1e-9);
    }

    // evaCamBasis: orthonormal, det +1, nose = the camera forward
    {
        const glm::dvec3 fwd = glm::normalize(glm::dvec3(-0.3, 0.1, -1.0));
        const glm::dvec3 up(0.05, 1.0, 0.2);
        const glm::dmat3 B = evaCamBasis(fwd, up);
        CHECK(std::fabs(glm::determinant(B) - 1.0) < 1e-9);
        CHECK(glm::length(B[2] - fwd) < 1e-9);
        CHECK(std::fabs(glm::dot(B[1], B[2])) < 1e-9);
        CHECK(std::fabs(glm::length(B[1]) - 1.0) < 1e-9);
        // degenerate up (parallel to fwd) must stay a rotation, not NaN
        const glm::dmat3 B2 = evaCamBasis(fwd, fwd);
        CHECK(std::fabs(glm::determinant(B2) - 1.0) < 1e-9);
    }

    // evaStandTarget: long axis = the radial, face tangent, degenerate OK
    {
        const glm::dvec3 up = glm::normalize(glm::dvec3(0.1, 0.2, 1.0));
        const glm::dmat3 T = evaStandTarget(up, glm::dvec3(1, 0, 0));
        CHECK(glm::length(T[2] - up) < 1e-9);
        CHECK(std::fabs(glm::dot(T[1], up)) < 1e-9);
        CHECK(std::fabs(glm::determinant(T) - 1.0) < 1e-9);
        const glm::dmat3 T2 = evaStandTarget(up, up);
        CHECK(glm::length(T2[2] - up) < 1e-9);
        CHECK(std::fabs(glm::dot(T2[1], up)) < 1e-9);
        CHECK(std::fabs(glm::determinant(T2) - 1.0) < 1e-9);
    }

    // evaSpaceTarget: long axis = camera up, face toward the viewer
    {
        const glm::dmat3 B = evaCamBasis(glm::dvec3(0, 0, -1), glm::dvec3(0, 1, 0));
        const glm::dmat3 S = evaSpaceTarget(B);
        CHECK(glm::length(S[2] - B[1]) < 1e-9);         // long axis = cam up
        CHECK(glm::length(S[1] + B[2]) < 1e-9);         // face = -cam fwd
        CHECK(std::fabs(glm::determinant(S) - 1.0) < 1e-9);
    }

    // screen-right + plane projection
    {
        CHECK(glm::length(evaScreenRight(glm::dvec3(0, 0, -1), glm::dvec3(0, 1, 0))
                          - glm::dvec3(1, 0, 0)) < 1e-9);
        CHECK(glm::length(evaOntoPlane(glm::dvec3(1, 2, 3), glm::dvec3(0, 0, 1))
                          - glm::dvec3(1, 2, 0)) < 1e-9);
    }

    if(failures == 0) {
        printf("test_eva: all checks passed\n");
        return 0;
    }
    printf("test_eva: %d FAILURES\n", failures);
    return 1;
}
