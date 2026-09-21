//
// Audio positional math (src/audio.h, inline pure math): listenerRelative
// converts a source's WORLD position into the mixer's listener frame --
// the listener at the origin, looking down -z, +x right, +y up (the
// SDL_mixer / OpenAL convention) -- from the camera's pos/up/forward.
// Pinned here without a mixer, so the convention is independent of the
// audio backend (like test_drag / test_orbit).
//
//   listenerRelative:  axis-aligned camera (the default view); a listener
//                      offset; a rotated (yawed) listener; the degenerate
//                      up || forward and up == -forward (no NaN, sane
//                      fallback); non-unit forwards (normalized inside);
//                      orthonormality of the basis; |result| == |rel|.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include "audio.h"

#include <glm/glm.hpp>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                 \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_failures++;                                                     \
            printf("FAIL: %s\n", msg);                                        \
        }                                                                     \
    } while (0)

#define CHECK_VEC(actual, ex, ey, ez, msg)                                    \
    do {                                                                      \
        g_checks++;                                                           \
        const glm::dvec3 _a = (actual);                                       \
        const double _dx = _a.x - (ex), _dy = _a.y - (ey), _dz = _a.z - (ez);\
        if (!std::isfinite(_a.x) || !std::isfinite(_a.y) ||                   \
            !std::isfinite(_a.z) ||                                           \
            std::fabs(_dx) + std::fabs(_dy) + std::fabs(_dz) > 1e-9) {        \
            g_failures++;                                                     \
            printf("FAIL: %s (got (%.9g, %.9g, %.9g), want (%g, %g, %g))\n",  \
                   msg, _a.x, _a.y, _a.z, (double)(ex), (double)(ey),         \
                   (double)(ez));                                             \
        }                                                                     \
    } while (0)

/* Axis-aligned camera at the origin, looking down -z (the game's default
   view): the listener frame IS the world frame, except z is flipped
   (world -z = in front = mixer -z; world +z = behind = mixer +z). */
static void testAxisAligned() {
    const glm::dvec3 L(0.0), up(0.0, 1.0, 0.0), f(0.0, 0.0, -1.0);

    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(5.0, 0.0, 0.0)),
              5.0, 0.0, 0.0, "right stays right");
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(0.0, 10.0, 0.0)),
              0.0, 10.0, 0.0, "up stays up");
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(0.0, 0.0, -20.0)),
              0.0, 0.0, -20.0, "in front maps to -z");
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(0.0, 0.0, 30.0)),
              0.0, 0.0, 30.0, "behind maps to +z");
    // The world is y-up looking down -z -- the mixer's convention already,
    // so the default view is the identity (the flip only happens when the
    // camera rotates).
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(3.0, -4.0, 5.0)),
              3.0, -4.0, 5.0, "mixed point: unchanged for the default view");
}

/* A listener offset: only the RELATIVE vector matters. */
static void testListenerOffset() {
    const glm::dvec3 L(1.0, 2.0, 3.0), up(0.0, 1.0, 0.0), f(0.0, 0.0, -1.0);
    // source - L = (5, 10, -20): the axis-aligned expectation.
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(6.0, 12.0, -17.0)),
              5.0, 10.0, -20.0, "offset listener: rel vector, z flipped");
}

/* Yaw 180 degrees: looking down +z. Right now points along -x (the
   listener's own right), and "in front" is world +z. */
static void testYawed() {
    const glm::dvec3 L(0.0), up(0.0, 1.0, 0.0), f(0.0, 0.0, 1.0);
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(0.0, 0.0, 20.0)),
              0.0, 0.0, -20.0, "in front (world +z) maps to -z");
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(5.0, 0.0, 0.0)),
              -5.0, 0.0, 0.0, "world +x is the listener's LEFT now");
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(0.0, 0.0, -30.0)),
              0.0, 0.0, 30.0, "world -z is behind now");
}

/* Yaw 45 degrees: in-front / right are the rotated axes, exact values. */
static void testYaw45() {
    const double s = std::sqrt(0.5);
    const glm::dvec3 L(1.0, 2.0, 3.0);
    const glm::dvec3 up(0.0, 1.0, 0.0);
    const glm::dvec3 f(s, 0.0, -s);       // looking 45 deg to the right
    const glm::dvec3 r(s, 0.0, s);        // the listener's right axis
    CHECK_VEC(listenerRelative(L, up, f, L + 10.0 * f),
              0.0, 0.0, -10.0, "45 deg: 10 m in front -> (0,0,-10)");
    CHECK_VEC(listenerRelative(L, up, f, L + 5.0 * r),
              5.0, 0.0, 0.0, "45 deg: 5 m to the right -> (5,0,0)");
    CHECK_VEC(listenerRelative(L, up, f, L + glm::dvec3(0.0, 7.0, 0.0)),
              0.0, 7.0, 0.0, "45 deg: straight up is always (0,y,0)");
}

/* up || forward: cross(f, up) is zero; the fallback right axis must keep
   the basis orthonormal and the result finite (no NaN/Inf). */
static void testDegenerate() {
    const glm::dvec3 L(0.0), up(0.0, 1.0, 0.0), f(0.0, 1.0, 0.0);
    const glm::dvec3 out = listenerRelative(L, up, f, glm::dvec3(0.0, 2.0, 0.0));
    CHECK_TRUE(std::isfinite(out.x) && std::isfinite(out.y) &&
               std::isfinite(out.z), "degenerate: finite result");
    // Looking straight up: "in front" is world +y, so (0,2,0) -> (0,0,-2).
    CHECK_VEC(out, 0.0, 0.0, -2.0, "degenerate: ahead is +y, maps to -z");
    // And a sideways source stays sideways (the fallback right is +x).
    CHECK_VEC(listenerRelative(L, up, f, glm::dvec3(4.0, 0.0, 0.0)),
              4.0, 0.0, 0.0, "degenerate: +x source stays +x");
}

/* The other degenerate branch: looking straight DOWN (f = -up). The
   fallback basis must stay orthonormal and "in front" is world -y. */
static void testDegenerateDown() {
    const glm::dvec3 L(0.0), up(0.0, 1.0, 0.0), f(0.0, -1.0, 0.0);
    const glm::dvec3 out = listenerRelative(L, up, f, glm::dvec3(0.0, -2.0, 0.0));
    CHECK_TRUE(std::isfinite(out.x) && std::isfinite(out.y) &&
               std::isfinite(out.z), "degenerate down: finite result");
    CHECK_VEC(out, 0.0, 0.0, -2.0, "degenerate down: ahead is -y, maps to -z");
}

/* The function normalizes the forward itself: a non-unit forward must
   behave exactly like its unit version. */
static void testUnnormalizedForward() {
    const glm::dvec3 L(0.0), up(0.0, 1.0, 0.0);
    CHECK_VEC(listenerRelative(L, up, glm::dvec3(0.0, 0.0, -2.0), glm::dvec3(5.0, 10.0, -20.0)),
              5.0, 10.0, -20.0, "unnormalized forward == its unit version");
}

/* The basis (r, u, -f) is orthonormal for arbitrary listeners: the
   rotation must not stretch -- |result| == |source - listener| -- and
   the axes must stay perpendicular. */
static void testOrthonormality() {
    struct { glm::dvec3 up, f; } cases[] = {
        { {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0} },
        { {0.0, 1.0, 0.0}, {0.7071067811865476, 0.0, -0.7071067811865476} },
        { {0.0, 1.0, 0.0}, {-0.31, 0.27, 0.91} },
        { {0.8, 0.3, -0.5}, {0.1, 0.9, 0.3} },   // tilted up vector too
    };
    const glm::dvec3 srcs[] = {
        { 1.0, 2.0, 3.0 }, { -4.0, 0.0, 9.0 },
        { 0.0, -6.5, 0.0 }, { 12.0, 12.0, -12.0 },
    };
    for(const auto &c : cases) {
        const glm::dvec3 f = glm::normalize(c.f);
        glm::dvec3 up = glm::normalize(c.up);
        // The function normalizes f itself; keep up as given.
        const glm::dvec3 L(0.0);
        for(const glm::dvec3 s : srcs) {
            const glm::dvec3 out = listenerRelative(L, up, c.f, s);
            CHECK_TRUE(std::isfinite(out.x) && std::isfinite(out.y) &&
                       std::isfinite(out.z), "orthonormal: finite result");
            CHECK_TRUE(std::fabs(glm::length(out) - glm::length(s)) < 1e-9,
                       "orthonormal: rotation preserves length");
        }
    }
}

int main() {
    testAxisAligned();
    testListenerOffset();
    testYawed();
    testYaw45();
    testDegenerate();
    testDegenerateDown();
    testUnnormalizedForward();
    testOrthonormality();

    if(g_failures) {
        printf("test_audio: %d/%d FAILED\n", g_failures, g_checks);
        return 1;
    }
    printf("test_audio: OK (%d checks)\n", g_checks);
    return 0;
}
