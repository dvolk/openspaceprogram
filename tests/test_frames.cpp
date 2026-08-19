// Unit tests for the reference-frame math (src/frame.cpp).
//
// The frame code is pure math (no rendering, no Bullet), so it can be tested
// in isolation. These tests pin down the behaviour that the critical
// frame-switching crash depended on, and guard the transform helpers against
// regressions.
//
// Build & run (from repo root):
//   g++ -O2 -std=c++11 -I./src -I./middleware/glm/ -I./middleware/bullet3/ \
//       -I./middleware/bullet3/bullet \
//       tests/test_frames.cpp src/frame.cpp -o test_frames && ./test_frames

#include <cmath>
#include <cstdio>
#include <cstdlib>

#define GLM_ENABLE_EXPERIMENTAL
#include "frame.h"
#include <glm/gtx/transform.hpp>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_NEAR(a, b, eps, msg)                                              \
    do {                                                                       \
        g_checks++;                                                            \
        double _a = (a), _b = (b);                                             \
        if (std::fabs(_a - _b) > (eps)) {                                      \
            g_failures++;                                                      \
            printf("FAIL: %s  (got %.9g, want %.9g)\n", msg, _a, _b);          \
        }                                                                      \
    } while (0)

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("FAIL: %s\n", msg);                                         \
        }                                                                      \
    } while (0)

static bool dvec_close(glm::dvec3 a, glm::dvec3 b, double eps) {
    return glm::length(a - b) < eps;
}

static bool mat_close(glm::dmat3 a, glm::dmat3 b, double eps) {
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            if (std::fabs(a[c][r] - b[c][r]) > eps) return false;
    return true;
}

// Build the same frame tree the game uses:
//   sun (root, inertial)
//    +-- eerbon (inertial)  pos (0,0,-1e8)
//         +-- eerbon_rot (rotating)  rot around Y
//         +-- moon (inertial)  pos (-12e6,0,0)
//              +-- moon_rot (rotating)
static Frame *make_tree() {
    Frame *sun = new Frame;
    Frame *eerbon = new Frame;
    Frame *eerbon_rot = new Frame;
    Frame *moon = new Frame;
    Frame *moon_rot = new Frame;

    sun->name = "sun";
    sun->parent = NULL;
    sun->children = { eerbon };
    sun->rotating = false;
    sun->has_rot_frame = false;
    sun->pos = glm::dvec3(0);
    sun->orient = glm::dmat3();
    sun->vel = glm::dvec3(0);
    sun->rot_ang_speed = 0;
    sun->orb_ang_speed = 0;
    sun->root_pos = glm::dvec3(0);
    sun->root_vel = glm::dvec3(0);
    sun->root_orient = glm::dmat3();

    eerbon->name = "eerbon";
    eerbon->parent = sun;
    eerbon->children = { eerbon_rot, moon };
    eerbon->rotating = false;
    eerbon->has_rot_frame = true;
    eerbon->pos = glm::dvec3(0, 0, -1e8);
    eerbon->orient = glm::dmat3();
    eerbon->vel = glm::dvec3(100, 0, 0); // some orbital velocity
    eerbon->rot_ang_speed = 0;
    eerbon->orb_ang_speed = 0;
    eerbon->root_pos = glm::dvec3(0);
    eerbon->root_vel = glm::dvec3(0);
    eerbon->root_orient = glm::dmat3();

    // 20-degree rotation about Y
    const double ang = 20.0 * M_PI / 180.0;
    eerbon_rot->name = "eerbon_rot";
    eerbon_rot->parent = eerbon;
    eerbon_rot->children = {};
    eerbon_rot->rotating = true;
    eerbon_rot->has_rot_frame = false;
    eerbon_rot->pos = glm::dvec3(0);
    eerbon_rot->orient = glm::dmat3(glm::rotate(ang, glm::dvec3(0, 1, 0)));
    eerbon_rot->vel = glm::dvec3(0);
    eerbon_rot->rot_ang_speed = 0.001;
    eerbon_rot->orb_ang_speed = 0;
    eerbon_rot->root_pos = glm::dvec3(0);
    eerbon_rot->root_vel = glm::dvec3(0);
    eerbon_rot->root_orient = glm::dmat3();

    moon->name = "moon";
    moon->parent = eerbon;
    moon->children = { moon_rot };
    moon->rotating = false;
    moon->has_rot_frame = true;
    moon->pos = glm::dvec3(-12e6, 0, 0);
    moon->orient = glm::dmat3();
    moon->vel = glm::dvec3(0, 0, 50); // moon moves relative to its parent (eerbon)
    moon->rot_ang_speed = 0;
    moon->orb_ang_speed = 0;
    moon->root_pos = glm::dvec3(0);
    moon->root_vel = glm::dvec3(0);
    moon->root_orient = glm::dmat3();

    moon_rot->name = "moon_rot";
    moon_rot->parent = moon;
    moon_rot->children = {};
    moon_rot->rotating = true;
    moon_rot->has_rot_frame = false;
    moon_rot->pos = glm::dvec3(0);
    moon_rot->orient = glm::dmat3();
    moon_rot->vel = glm::dvec3(0);
    moon_rot->rot_ang_speed = 0.0005;
    moon_rot->orb_ang_speed = 0;
    moon_rot->root_pos = glm::dvec3(0);
    moon_rot->root_vel = glm::dvec3(0);
    moon_rot->root_orient = glm::dmat3();

    return sun;
}

int main() {
    Frame *sun = make_tree();
    // Recompute all root-relative values (parents before children, recursively).
    sun->UpdateOrbitRails(0.0, 0.0);

    // Grab the frames by walking the tree.
    Frame *eerbon = sun->children[0];
    Frame *eerbon_rot = eerbon->children[0];
    Frame *moon = eerbon->children[1];
    Frame *moon_rot = moon->children[0];

    const double E = 1e-6;

    printf("== UpdateRootRelative ==\n");
    // root_pos composes down the tree.
    CHECK_NEAR(glm::length(eerbon->root_pos), 1e8, E, "eerbon.root_pos magnitude == 1e8");
    CHECK_NEAR(glm::length(moon->root_pos), std::sqrt((12e6) * (12e6) + (1e8) * (1e8)), 1.0,
               "moon.root_pos magnitude");
    // root_vel composes: sun(0) + eerbon.vel
    CHECK_NEAR(glm::length(eerbon->root_vel), 100.0, E, "eerbon.root_vel magnitude == 100");
    // moon.root_vel = eerbon.root_vel + eerbon_orient*moon.vel = (100,0,50)
    CHECK_TRUE(dvec_close(moon->root_vel, glm::dvec3(100, 0, 50), 1e-9),
               "moon.root_vel == (100,0,50)");

    printf("== identity relations ==\n");
    CHECK_TRUE(dvec_close(sun->GetPositionRelTo(sun), glm::dvec3(0), E), "GetPos(A,A) == 0");
    CHECK_TRUE(dvec_close(eerbon->GetVelocityRelTo(eerbon), glm::dvec3(0), E), "GetVel(A,A) == 0");
    CHECK_TRUE(mat_close(eerbon->GetOrientRelTo(eerbon), glm::dmat3(), E), "GetOrient(A,A) == I");

    printf("== concrete positions ==\n");
    // eerbon relative to sun is just its pos.
    CHECK_TRUE(dvec_close(eerbon->GetPositionRelTo(sun), glm::dvec3(0, 0, -1e8), E),
               "eerbon rel sun == (0,0,-1e8)");
    // moon relative to eerbon is just its pos.
    CHECK_TRUE(dvec_close(moon->GetPositionRelTo(eerbon), glm::dvec3(-12e6, 0, 0), E),
               "moon rel eerbon == (-12e6,0,0)");
    // eerbon relative to moon is the negative.
    CHECK_TRUE(dvec_close(eerbon->GetPositionRelTo(moon), glm::dvec3(12e6, 0, 0), E),
               "eerbon rel moon == (12e6,0,0)");
    // moon relative to sun.
    CHECK_TRUE(dvec_close(moon->GetPositionRelTo(sun), glm::dvec3(-12e6, 0, -1e8), 1.0),
               "moon rel sun == (-12e6,0,-1e8)");

    printf("== orient transitivity: O(A,B)*O(B,C) == O(A,C) ==\n");
    {
        glm::dmat3 lhs = eerbon_rot->GetOrientRelTo(sun);
        glm::dmat3 rhs = eerbon_rot->GetOrientRelTo(eerbon) * eerbon->GetOrientRelTo(sun);
        CHECK_TRUE(mat_close(lhs, rhs, 1e-9), "O(rot,sun) == O(rot,eerbon)*O(eerbon,sun)");
        // O(rot, sun) should be the 20-degree Y rotation.
        glm::dmat3 expected = glm::dmat3(glm::rotate(20.0 * M_PI / 180.0, glm::dvec3(0, 1, 0)));
        CHECK_TRUE(mat_close(lhs, expected, 1e-9), "O(rot,sun) == 20deg about Y");
    }
    {
        glm::dmat3 lhs = moon_rot->GetOrientRelTo(sun);
        glm::dmat3 rhs = moon_rot->GetOrientRelTo(moon) * moon->GetOrientRelTo(sun);
        CHECK_TRUE(mat_close(lhs, rhs, 1e-9), "O(moonrot,sun) transitivity");
    }

    printf("== orient invertibility: O(A,B)*O(B,A) == I ==\n");
    {
        glm::dmat3 prod = eerbon_rot->GetOrientRelTo(sun) * sun->GetOrientRelTo(eerbon_rot);
        CHECK_TRUE(mat_close(prod, glm::dmat3(), 1e-9), "O(rot,sun)*O(sun,rot) == I");
    }

    printf("== velocity relative ==\n");
    // eerbon rel sun velocity == eerbon.vel (sun is inertial).
    CHECK_TRUE(dvec_close(eerbon->GetVelocityRelTo(sun), glm::dvec3(100, 0, 0), E),
               "eerbon vel rel sun == (100,0,0)");
    // moon rel eerbon == the moon's own velocity relative to its parent.
    // (The raw `vel` fields are each relative to a DIFFERENT parent and must
    // not be subtracted directly; the code correctly composes via root_vel.)
    CHECK_TRUE(dvec_close(moon->GetVelocityRelTo(eerbon), glm::dvec3(0, 0, 50), E),
               "moon vel rel eerbon == (0,0,50)");

    printf("== velocity rel a rotating frame (diff * root_orient) ==\n");
    // sun rel eerbon_rot: diff = sun.root_vel - eerbon_rot.root_vel = (0)-(100,0,0)
    //   = (-100,0,0); then transformed into eerbon_rot's frame: diff * R, where
    //   R = R_y(20deg). NOTE: GLM's (v * M) == transpose(M) * v (column-vector
    //   convention), so this equals R^T * diff = (-100cos a, 0, -100sin a).
    {
        const double a = 20.0 * M_PI / 180.0;
        glm::dvec3 expected = glm::dvec3(-100.0 * std::cos(a), 0.0, -100.0 * std::sin(a));
        CHECK_TRUE(dvec_close(sun->GetVelocityRelTo(eerbon_rot), expected, 1e-9),
                   "sun vel rel eerbon_rot == (-100cos20, 0, -100sin20)");
    }

    printf("== stasis velocity ==\n");
    // For a frame rotating about Y at rate w, stasis vel at (1,0,0) is (0,0,w).
    {
        double w = eerbon_rot->rot_ang_speed;
        glm::dvec3 sv = eerbon_rot->GetStasisVelocity(glm::dvec3(1, 0, 0));
        CHECK_NEAR(sv.x, 0.0, E, "stasis(1,0,0).x == 0");
        CHECK_NEAR(sv.y, 0.0, E, "stasis(1,0,0).y == 0");
        CHECK_NEAR(sv.z, w, E, "stasis(1,0,0).z == w");
    }
    {
        double w = eerbon_rot->rot_ang_speed;
        glm::dvec3 sv = eerbon_rot->GetStasisVelocity(glm::dvec3(0, 0, 1));
        CHECK_NEAR(sv.x, -w, E, "stasis(0,0,1).x == -w");
        CHECK_NEAR(sv.y, 0.0, E, "stasis(0,0,1).y == 0");
        CHECK_NEAR(sv.z, 0.0, E, "stasis(0,0,1).z == 0");
    }
    // A non-rotating frame has zero stasis velocity.
    CHECK_TRUE(dvec_close(sun->GetStasisVelocity(glm::dvec3(1, 2, 3)), glm::dvec3(0), E),
               "stasis vel of inertial frame == 0");

    printf("== getRotFrame / getNonRotFrame ==\n");
    CHECK_TRUE(eerbon->getRotFrame() == eerbon_rot, "eerbon.getRotFrame() == eerbon_rot");
    CHECK_TRUE(eerbon->getNonRotFrame() == eerbon, "eerbon.getNonRotFrame() == eerbon");
    CHECK_TRUE(eerbon_rot->getRotFrame() == eerbon_rot, "eerbon_rot.getRotFrame() == self");
    CHECK_TRUE(eerbon_rot->getNonRotFrame() == eerbon, "eerbon_rot.getNonRotFrame() == parent");

    // ---- summary ----
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
    } else {
        printf("SOME TESTS FAILED\n");
    }

    // Cleanup (delete children then parents).
    delete moon_rot; delete moon; delete eerbon_rot; delete eerbon; delete sun;
    return g_failures == 0 ? 0 : 1;
}
