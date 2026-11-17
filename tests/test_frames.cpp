// Unit tests for the reference-frame math (src/frame.cpp).
//
// The frame code is pure math (no rendering, no Bullet), so it can be tested
// in isolation. These tests pin down the behaviour that the critical
// frame-switching crash depended on, and guard the transform helpers against
// regressions.
//
// Build & run (from repo root):
//   g++ -O2 -std=c++20 -I./src -I./middleware/glm/ -I./middleware/bullet3/src \
//       tests/test_frames.cpp src/frame.cpp -o test_frames && ./test_frames

#include <cmath>
#include <cstdio>
#include <cstdlib>

#define GLM_ENABLE_EXPERIMENTAL
#include "frame.h"
#include "orbit.h"
#include "buildingorbit.h"
#include "buildingair.h"
#include <glm/gtx/transform.hpp>
#include <glm/gtx/norm.hpp>
#include "tchecks.h"

/* The frame tests link frame.o alone: StaticBuilding is only ever a marker
   pointer here, never dereferenced, so a stub definition stands in for the
   real one (terrain.h) without dragging in the physics chain. */
struct StaticBuilding {};

static bool dvec_close(glm::dvec3 a, glm::dvec3 b, double eps) {
    return glm::length(a - b) < eps;
}

static bool mat_close(glm::dmat3 a, glm::dmat3 b, double eps) {
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            if (!std::isfinite(a[c][r]) || !std::isfinite(b[c][r])
                || std::fabs(a[c][r] - b[c][r]) > eps) return false;
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
    sun->pos = glm::dvec3(0);
    sun->orient = glm::dmat3(1.0);
    sun->vel = glm::dvec3(0);
    sun->rot_ang_speed = 0;
    sun->orb_ang_speed = 0;
    sun->root_pos = glm::dvec3(0);
    sun->root_vel = glm::dvec3(0);
    sun->root_orient = glm::dmat3(1.0);

    eerbon->name = "eerbon";
    eerbon->parent = sun;
    eerbon->children = { eerbon_rot, moon };
    eerbon->rotating = false;
    eerbon->rot_frame = eerbon_rot;
    eerbon->pos = glm::dvec3(0, 0, -1e8);
    eerbon->orient = glm::dmat3(1.0);
    eerbon->vel = glm::dvec3(100, 0, 0); // some orbital velocity
    eerbon->rot_ang_speed = 0;
    eerbon->orb_ang_speed = 0;
    eerbon->root_pos = glm::dvec3(0);
    eerbon->root_vel = glm::dvec3(0);
    eerbon->root_orient = glm::dmat3(1.0);

    // 20-degree rotation about Y
    const double ang = 20.0 * M_PI / 180.0;
    eerbon_rot->name = "eerbon_rot";
    eerbon_rot->parent = eerbon;
    eerbon_rot->children = {};
    eerbon_rot->rotating = true;
    eerbon_rot->pos = glm::dvec3(0);
    // UpdateOrbitRails re-derives orient as initial_orient * rotate(ang, Y);
    // at time 0 that must reproduce the initial 20-degree orientation.
    eerbon_rot->initial_orient = glm::dmat3(glm::rotate(ang, glm::dvec3(0, 1, 0)));
    eerbon_rot->orient = glm::dmat3(glm::rotate(ang, glm::dvec3(0, 1, 0)));
    eerbon_rot->vel = glm::dvec3(0);
    eerbon_rot->rot_ang_speed = 0.001;
    eerbon_rot->orb_ang_speed = 0;
    eerbon_rot->root_pos = glm::dvec3(0);
    eerbon_rot->root_vel = glm::dvec3(0);
    eerbon_rot->root_orient = glm::dmat3(1.0);

    moon->name = "moon";
    moon->parent = eerbon;
    moon->children = { moon_rot };
    moon->rotating = false;
    moon->rot_frame = moon_rot;
    moon->pos = glm::dvec3(-12e6, 0, 0);
    moon->orient = glm::dmat3(1.0);
    moon->vel = glm::dvec3(0, 0, 50); // moon moves relative to its parent (eerbon)
    moon->rot_ang_speed = 0;
    moon->orb_ang_speed = 0;
    moon->root_pos = glm::dvec3(0);
    moon->root_vel = glm::dvec3(0);
    moon->root_orient = glm::dmat3(1.0);

    moon_rot->name = "moon_rot";
    moon_rot->parent = moon;
    moon_rot->children = {};
    moon_rot->rotating = true;
    moon_rot->pos = glm::dvec3(0);
    moon_rot->initial_orient = glm::dmat3(1.0);
    moon_rot->orient = glm::dmat3(1.0);
    moon_rot->vel = glm::dvec3(0);
    moon_rot->rot_ang_speed = 0.0005;
    moon_rot->orb_ang_speed = 0;
    moon_rot->root_pos = glm::dvec3(0);
    moon_rot->root_vel = glm::dvec3(0);
    moon_rot->root_orient = glm::dmat3(1.0);

    return sun;
}

int main() {
    /* #179: a Frame the loader never wired must read null/zero, not stack
       garbage. The const IS the guard -- a const object whose default
       constructor leaves any member unset is ill-formed, so this line stops
       compiling the moment anyone adds a Frame member without an
       initializer. The CHECKs only document which sentinel is which. */
    {
        const Frame f;
        CHECK_TRUE(f.parent == nullptr, "fresh Frame has no parent");
        CHECK_TRUE(f.body == nullptr, "fresh Frame has no body");
        CHECK_TRUE(f.rot_frame == nullptr, "fresh Frame has no spin frame");
        CHECK_TRUE(glm::length(f.root_pos) == 0.0, "fresh Frame root_pos is the origin");
        CHECK_TRUE(glm::length(f.root_vel) == 0.0, "fresh Frame root_vel is zero");
    }

    Frame *sun = make_tree();
    // Recompute all root-relative values (parents before children, recursively).
    sun->UpdateOrbitRails(0.0);

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
    CHECK_TRUE(mat_close(eerbon->GetOrientRelTo(eerbon), glm::dmat3(1.0), E), "GetOrient(A,A) == I");

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

    printf("== rel-to round trips across the whole tree ==\n");
    // For EVERY frame pair: relTo->root_orient * A->Get*RelTo(relTo) lands
    // back on A's universe-axis origin state. Pins the contract that the
    // result is expressed in relTo's OWN axes (the inertial-target case
    // used to return universe axes, which is only right while all of
    // relTo's ancestor orbits are uninclined).
    {
        Frame *frames[] = { sun, eerbon, eerbon_rot, moon, moon_rot };
        for (size_t a = 0; a < 5; a++) {
            for (size_t b = 0; b < 5; b++) {
                Frame *A = frames[a], *B = frames[b];
                glm::dvec3 pback = B->root_orient * A->GetPositionRelTo(B) + B->root_pos;
                CHECK_TRUE(dvec_close(pback, A->root_pos, 1.0),
                           "pos round trip: B.orient*(A rel B) + B.pos == A.pos");
                glm::dvec3 vback = B->root_orient * A->GetVelocityRelTo(B) + B->root_vel;
                CHECK_TRUE(dvec_close(vback, A->root_vel, 1e-9),
                           "vel round trip: B.orient*(A rel B) + B.vel == A.vel");
            }
        }
    }

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
        CHECK_TRUE(mat_close(prod, glm::dmat3(1.0), 1e-9), "O(rot,sun)*O(sun,rot) == I");
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
    // For a frame rotating about Y at rate w (prograde spin, issue #101),
    // stasis vel at (1,0,0) is (0,0,-w).
    {
        double w = eerbon_rot->rot_ang_speed;
        glm::dvec3 sv = eerbon_rot->GetStasisVelocity(glm::dvec3(1, 0, 0));
        CHECK_NEAR(sv.x, 0.0, E, "stasis(1,0,0).x == 0");
        CHECK_NEAR(sv.y, 0.0, E, "stasis(1,0,0).y == 0");
        CHECK_NEAR(sv.z, -w, E, "stasis(1,0,0).z == -w");
    }
    {
        double w = eerbon_rot->rot_ang_speed;
        glm::dvec3 sv = eerbon_rot->GetStasisVelocity(glm::dvec3(0, 0, 1));
        CHECK_NEAR(sv.x, w, E, "stasis(0,0,1).x == w");
        CHECK_NEAR(sv.y, 0.0, E, "stasis(0,0,1).y == 0");
        CHECK_NEAR(sv.z, 0.0, E, "stasis(0,0,1).z == 0");
    }
    // A non-rotating frame has zero stasis velocity.
    CHECK_TRUE(dvec_close(sun->GetStasisVelocity(glm::dvec3(1, 2, 3)), glm::dvec3(0), E),
               "stasis vel of inertial frame == 0");

    printf("== tilted spin axis (axial tilt) ==\n");
    // A frame whose spin axis is tilted (here to +X) uses that axis for the
    // stasis velocity: stasis = cross((w,0,0), p).
    {
        Frame *N = new Frame;
        N->rotating = true;
        N->rot_ang_speed = 0.002;
        N->spin_axis = glm::dvec3(1, 0, 0);   // spin about X (90-deg axial tilt)
        N->UpdateRootRelative();   // stasis reads omega_total, the tree clocks it
        const double w = N->rot_ang_speed;
        glm::dvec3 sv = N->GetStasisVelocity(glm::dvec3(0, 1, 0));
        CHECK_TRUE(dvec_close(sv, glm::dvec3(0, 0, w), 1e-9),
                   "tilted-spin: stasis(0,1,0) about X == (0,0,w)");
        sv = N->GetStasisVelocity(glm::dvec3(0, 0, 1));
        CHECK_TRUE(dvec_close(sv, glm::dvec3(0, -w, 0), 1e-9),
                   "tilted-spin: stasis(0,0,1) about X == (0,-w,0)");
        delete N;
    }
    // The fictitious (centrifugal) term uses the same tilted axis: with
    // omega = (w,0,0) and p on +Y, -omega x (omega x p) == (0, +w^2, 0).
    {
        Frame *N = new Frame;
        N->rotating = true;
        N->rot_ang_speed = 0.002;
        N->spin_axis = glm::dvec3(1, 0, 0);
        N->UpdateRootRelative();   // fictitious reads omega_total
        const double w = N->rot_ang_speed;
        glm::dvec3 a = N->GetFictitiousAccel(glm::dvec3(0, 1, 0), glm::dvec3(0));
        CHECK_TRUE(dvec_close(a, glm::dvec3(0, w * w, 0), 1e-9),
                   "tilted-spin: centrifugal at (0,1,0) about X == (0,+w^2,0)");
        delete N;
    }
    // The default spin axis is the historical pure-Y convention.
    {
        Frame *N = new Frame;
        N->rotating = true;
        N->rot_ang_speed = 0.001;
        // spin_axis left at its default (0,1,0)
        CHECK_TRUE(dvec_close(N->spin_axis, glm::dvec3(0, 1, 0), 1e-12),
                   "default spin_axis == (0,1,0)");
        delete N;
    }

    printf("== orbital inclination ==\n");
    // An inertial frame whose orbital plane is tilted about the parent's X
    // carries pos (in the local plane) into the parent via orient:
    // root_pos = orient * (0,0,-r) == (0, r sin i, -r cos i) for a root parent.
    {
        Frame *parent = new Frame;
        parent->parent = NULL;
        parent->rotating = false;
        parent->pos = glm::dvec3(0);
        parent->orient = glm::dmat3(1.0);
        parent->root_pos = glm::dvec3(0);
        parent->root_orient = glm::dmat3(1.0);

        Frame *child = new Frame;
        child->parent = parent;
        child->rotating = false;
        child->body = NULL;
        child->pos = glm::dvec3(0, 0, -1.0e8);
        const double i = 30.0 * M_PI / 180.0;
        const double c = std::cos(i), s = std::sin(i);
        child->orient = glm::dmat3(glm::dvec3(1.0, 0.0, 0.0),
                                   glm::dvec3(0.0,  c,  s),
                                   glm::dvec3(0.0, -s,  c));
        child->root_pos = glm::dvec3(0);
        parent->children.push_back(child);

        parent->UpdateOrbitRails(0.0);
        glm::dvec3 expected(0.0, 1.0e8 * s, -1.0e8 * c);
        CHECK_TRUE(dvec_close(child->root_pos, expected, 1.0),
                   "orb-incl: tilted root_pos == (0, r sin i, -r cos i)");
        delete child;
        delete parent;
    }

    printf("== Keplerian orbit rail ==\n");
    // UpdateOrbitRails propagates the epoch state on the true Kepler conic
    // under the parent's mu (propagateKepler) instead of rotating it, and
    // the body's orbital velocity composes into root_vel. Pins the circular
    // quarter-period motion, the full-period round trip, and the elliptic
    // periapsis -> apoapsis half-period swap.
    {
        const double mu = 1.0e12;
        Frame *root = new Frame;
        root->parent = NULL;
        root->rotating = false;
        root->body = NULL;
        root->pos = glm::dvec3(0);
        root->orient = glm::dmat3(1.0);
        root->root_pos = glm::dvec3(0);
        root->root_vel = glm::dvec3(0);
        root->root_orient = glm::dmat3(1.0);

        const double r = 1.0e8;
        const double vc = sqrt(mu / r);
        Frame *c = new Frame;
        c->parent = root;
        c->rotating = false;
        c->body = NULL;
        c->orb_ang_speed = 1.0;   // nonzero => the rail is active
        c->parent_mu = mu;
        // epoch at longitude pi/2, i.e. on -Z: prograde (+Y x r_hat) there is -X
        railStateFromElements(r, 0.0, 0.0, M_PI / 2.0, mu,
                              c->orbit_pos0, c->orbit_vel0);
        c->pos = c->orbit_pos0;
        c->vel = c->orbit_vel0;
        c->orient = glm::dmat3(1.0);
        c->root_pos = glm::dvec3(0);
        c->root_vel = glm::dvec3(0);
        c->root_orient = glm::dmat3(1.0);
        root->children.push_back(c);

        CHECK_TRUE(dvec_close(c->orbit_vel0, glm::dvec3(-vc, 0, 0), 1e-9 * vc),
                   "rail: circular epoch vel at -Z == -X * sqrt(mu/r)");

        const double T = 2.0 * M_PI * sqrt(r * r * r / mu);
        root->UpdateOrbitRails(T / 4.0);
        // a quarter period sweeps 90 deg of INCREASING longitude (-Z -> -X)
        CHECK_TRUE(dvec_close(c->pos, glm::dvec3(-r, 0, 0), 1e-3),
                   "rail: circular quarter-period position");
        CHECK_TRUE(dvec_close(c->vel, glm::dvec3(0, 0, vc), 1e-9 * vc),
                   "rail: circular quarter-period velocity (prograde)");
        // orbital velocity composes into root_vel (root parent)
        CHECK_TRUE(dvec_close(c->root_vel, c->vel, 1e-9 * vc),
                   "rail: root_vel == orbital vel (root parent)");
        CHECK_TRUE(dvec_close(c->GetVelocityRelTo(root), c->vel, 1e-9 * vc),
                   "rail: vel rel root == orbital vel");

        root->UpdateOrbitRails(T);
        CHECK_TRUE(dvec_close(c->pos, c->orbit_pos0, 1e-3),
                   "rail: full period returns to epoch position");
        CHECK_TRUE(dvec_close(c->vel, c->orbit_vel0, 1e-9 * vc),
                   "rail: full period returns to epoch velocity");
        delete c;
        delete root;
    }
    {
        const double mu = 1.0e12;
        const double a = 1.0e8, e = 0.5;
        const double rp = a * (1.0 - e), ra = a * (1.0 + e);
        const double vp = sqrt(mu * (2.0 / rp - 1.0 / a));
        const double va = sqrt(mu * (2.0 / ra - 1.0 / a));
        const double T = 2.0 * M_PI * sqrt(a * a * a / mu);
        const double E0 = vp * vp / 2.0 - mu / rp;   // = -mu/(2a)

        Frame *root = new Frame;
        root->parent = NULL;
        root->rotating = false;
        root->body = NULL;
        root->pos = glm::dvec3(0);
        root->orient = glm::dmat3(1.0);
        root->root_pos = glm::dvec3(0);
        root->root_vel = glm::dvec3(0);
        root->root_orient = glm::dmat3(1.0);

        Frame *c = new Frame;
        c->parent = root;
        c->rotating = false;
        c->body = NULL;
        c->orb_ang_speed = 1.0;
        c->parent_mu = mu;
        // epoch at periapsis on -Z (arg_peri = pi/2, nu = 0): prograde -X
        railStateFromElements(a, e, M_PI / 2.0, 0.0, mu,
                              c->orbit_pos0, c->orbit_vel0);
        c->pos = c->orbit_pos0;
        c->vel = c->orbit_vel0;
        c->orient = glm::dmat3(1.0);
        c->root_pos = glm::dvec3(0);
        c->root_vel = glm::dvec3(0);
        c->root_orient = glm::dmat3(1.0);
        root->children.push_back(c);

        CHECK_TRUE(dvec_close(c->orbit_pos0, glm::dvec3(0, 0, -rp), 1e-3),
                   "rail: elliptic epoch at periapsis radius on -Z");
        CHECK_TRUE(dvec_close(c->orbit_vel0, glm::dvec3(-vp, 0, 0), 1e-9 * vp),
                   "rail: elliptic epoch vel == -X * vis-viva at periapsis");

        root->UpdateOrbitRails(T / 2.0);
        // half a period later: longitude pi/2 -> 3pi/2, apoapsis on +Z,
        // prograde there is +X
        CHECK_TRUE(dvec_close(c->pos, glm::dvec3(0, 0, ra), 1e-3),
                   "rail: elliptic T/2 position == apoapsis on +Z");
        CHECK_TRUE(dvec_close(c->vel, glm::dvec3(va, 0, 0), 1e-9 * va),
                   "rail: elliptic T/2 velocity (prograde at apoapsis)");
        const double r1 = glm::length(c->pos), v1 = glm::length(c->vel);
        CHECK_NEAR(v1 * v1 / 2.0 - mu / r1, E0, 1e-6 * fabs(E0),
                   "rail: elliptic T/2 energy conserved");
        CHECK_TRUE(dvec_close(c->GetVelocityRelTo(root), c->vel, 1e-9 * va),
                   "rail: elliptic vel rel root == orbital vel");
        delete c;
        delete root;
    }

    printf("== rel-to an INCLINED inertial frame ==\n");
    // The regression this pins: expressing another frame's origin state
    // relative to an inertial frame whose root_orient != I (inclined
    // ancestors) must rotate the universe-axis difference into that
    // frame's own axes -- the old code returned the raw universe delta,
    // which teleports ships on SOI switches to inclined bodies (Minmus,
    // Gilly, Bop, ...).
    {
        Frame *parent = new Frame;
        parent->parent = NULL;
        parent->rotating = false;
        parent->pos = glm::dvec3(0);
        parent->vel = glm::dvec3(0);
        parent->orient = glm::dmat3(1.0);
        parent->root_pos = glm::dvec3(0);
        parent->root_vel = glm::dvec3(0);
        parent->root_orient = glm::dmat3(1.0);

        Frame *child = new Frame;      // inclined orbit -> root_orient != I
        child->parent = parent;
        child->rotating = false;
        child->body = NULL;
        child->pos = glm::dvec3(0, 0, -1.0e8);
        child->vel = glm::dvec3(30, 0, 0);
        const double i = 30.0 * M_PI / 180.0;
        const double c = std::cos(i), s = std::sin(i);
        child->orient = glm::dmat3(glm::dvec3(1.0, 0.0, 0.0),
                                   glm::dvec3(0.0,  c,  s),
                                   glm::dvec3(0.0, -s,  c));
        child->root_pos = glm::dvec3(0);
        child->root_vel = glm::dvec3(0);
        child->root_orient = glm::dmat3(1.0);
        parent->children.push_back(child);

        Frame *gc = new Frame;         // moon of the inclined body
        child->children.push_back(gc);
        gc->parent = child;
        gc->rotating = false;
        gc->body = NULL;
        // off the tilt axis (X): the universe delta then differs from the
        // child-local coordinates, which is what the check below pins
        gc->pos = glm::dvec3(0, 5.0e6, 0);
        gc->vel = glm::dvec3(0, 0, 120);
        gc->orient = glm::dmat3(1.0);
        gc->root_pos = glm::dvec3(0);
        gc->root_vel = glm::dvec3(0);
        gc->root_orient = glm::dmat3(1.0);

        parent->UpdateOrbitRails(0.0);

        // child's root_orient is the tilt itself (parent is the root)
        CHECK_TRUE(mat_close(child->root_orient, child->orient, 1e-12),
                   "incl-relTo: child.root_orient == the tilt");
        // rel child: gc's own local pos/vel (its relative orient is I)
        CHECK_TRUE(dvec_close(gc->GetPositionRelTo(child), gc->pos, 1.0),
                   "incl-relTo: gc pos rel child == gc.pos (child-local axes)");
        CHECK_TRUE(dvec_close(gc->GetVelocityRelTo(child), gc->vel, 1e-9),
                   "incl-relTo: gc vel rel child == gc.vel (child-local axes)");
        CHECK_TRUE(dvec_close(child->GetPositionRelTo(gc), -gc->pos, 1.0),
                   "incl-relTo: child pos rel gc == -gc.pos");
        // NOT the raw universe delta (what the old code returned)
        CHECK_TRUE(!dvec_close(gc->GetPositionRelTo(child),
                               gc->root_pos - child->root_pos, 1.0),
                   "incl-relTo: result != universe-axis delta");
        // universe round trip
        glm::dvec3 back = child->root_orient * gc->GetPositionRelTo(child) + child->root_pos;
        CHECK_TRUE(dvec_close(back, gc->root_pos, 1.0), "incl-relTo: pos round trip");
        glm::dvec3 vback = child->root_orient * gc->GetVelocityRelTo(child) + child->root_vel;
        CHECK_TRUE(dvec_close(vback, gc->root_vel, 1e-9), "incl-relTo: vel round trip");

        delete gc;
        delete child;
        delete parent;
    }

    printf("== stasis identity vs frame rotation ==\n");
    // A point FIXED in the rotating frame (v = 0) drifts through root space
    // at  R * stasis(pos)  (plus the frame origin's velocity, which is
    // constant here). This links the rotate(ang, Y) convention in
    // UpdateOrbitRails to the sign of GetStasisVelocity; the velocity
    // transforms (Vehicle::moveToFrame, render.cpp, tick.cpp, docktest.cpp)
    // rely on  v_root = R*(v + stasis(p)) + V,
    // so this identity is what makes frame switching state-preserving.
    {
        Frame *N = eerbon_rot;
        const glm::dvec3 p(0.3, 0.4, 1.1); // a point fixed in N
        const double t0 = 37.0, h = 1e-4;

        // The parent (eerbon) is static in this tree, so N's parent-relative
        // orient is the rotation we want, without the 1e8 root_pos offset
        // (which would swamp a central difference of O(1) quantities in
        // double precision).
        sun->UpdateOrbitRails(t0 - h);
        glm::dvec3 X1 = N->orient * p;
        sun->UpdateOrbitRails(t0 + h);
        glm::dvec3 X2 = N->orient * p;
        sun->UpdateOrbitRails(t0);

        glm::dvec3 drift = (X2 - X1) / (2.0 * h);
        glm::dvec3 expect = N->orient * N->GetStasisVelocity(p);
        CHECK_TRUE(dvec_close(drift, expect, 1e-6),
                   "fixed-point drift == orient * stasis(pos)");
    }

    printf("== getRotFrame / getNonRotFrame ==\n");
    CHECK_TRUE(eerbon->getRotFrame() == eerbon_rot, "eerbon.getRotFrame() == eerbon_rot");
    CHECK_TRUE(eerbon->getNonRotFrame() == eerbon, "eerbon.getNonRotFrame() == eerbon");
    CHECK_TRUE(eerbon_rot->getRotFrame() == eerbon_rot, "eerbon_rot.getRotFrame() == self");
    CHECK_TRUE(eerbon_rot->getNonRotFrame() == eerbon, "eerbon_rot.getNonRotFrame() == parent");

    // ---- body draw transform (issue #27) ----
    // render.cpp draws each body with GetBodyDrawTransform(rf). The rotation
    // half must be getRotFrame()->GetOrientRelTo(rf), never bare
    // getRotFrame()->orient (parent-relative only). Standing on a spinning
    // body (rf = eerbon_rot) and drawing the moon is the repro: the old
    // formula left the moon un-rotated by the local spin, so its surface
    // drifted once per local day while the sky position stayed correct.
    printf("== body draw transform (issue #27) ==\n");
    {
        // Landed: rf is the local body's spinning rot frame.
        Frame *rf = eerbon_rot;
        const glm::dmat4 T = moon->GetBodyDrawTransform(rf);
        const glm::dmat3 drawn(T);
        const glm::dmat3 want = moon->getRotFrame()->GetOrientRelTo(rf);
        CHECK_TRUE(mat_close(drawn, want, 1e-9),
                   "drawn rot == getRotFrame()->GetOrientRelTo(rf)");
        CHECK_TRUE(dvec_close(glm::dvec3(T[3]), moon->GetPositionRelTo(rf), 1e-9),
                   "drawn origin == GetPositionRelTo(rf)");
        // The bug: bare orient is identity here and ignores rf's spin.
        CHECK_TRUE(!mat_close(drawn, moon->getRotFrame()->orient, 1e-6),
                   "drawn rot differs from bare getRotFrame()->orient when rf spins");
        // Explicit expected value: moon is unspun and un-tilted in this tree,
        // so its axes in eerbon_rot are the inverse of eerbon_rot's 20-degree
        // spin (and of any spin at a later time).
        sun->UpdateOrbitRails(37.0);
        const glm::dmat4 T2 = moon->GetBodyDrawTransform(eerbon_rot);
        const glm::dmat3 want2 = glm::transpose(eerbon_rot->root_orient)
                               * moon->getRotFrame()->root_orient;
        CHECK_TRUE(mat_close(glm::dmat3(T2), want2, 1e-9),
                   "at t=37: drawn rot == transpose(rf.root_orient)*rot.root_orient");
        sun->UpdateOrbitRails(0.0);
    }
    {
        // Local body, rf is its rot frame: identity, origin 0.
        const glm::dmat4 T = eerbon->GetBodyDrawTransform(eerbon_rot);
        CHECK_TRUE(mat_close(glm::dmat3(T), glm::dmat3(1.0), 1e-9),
                   "local body in its rot frame: rot == I");
        CHECK_TRUE(dvec_close(glm::dvec3(T[3]), glm::dvec3(0), 1e-9),
                   "local body in its rot frame: origin == 0");
    }
    {
        // Local body, rf is its inertial parent: spin relative to that parent
        // (the old special case used bare orient, which is correct only here).
        const glm::dmat4 T = eerbon->GetBodyDrawTransform(eerbon);
        CHECK_TRUE(mat_close(glm::dmat3(T), eerbon->getRotFrame()->orient, 1e-9),
                   "local body in its inertial parent: rot == getRotFrame()->orient");
        CHECK_TRUE(dvec_close(glm::dvec3(T[3]), glm::dvec3(0), 1e-9),
                   "local body in its inertial parent: origin == 0");
    }
    {
        // Inclined-orbit case: a non-identity ancestor tilt must be carried
        // into the draw rotation even with an inertial rf (sun).
        moon->orient = glm::dmat3(glm::rotate(11.0 * M_PI / 180.0, glm::dvec3(0, 0, 1)));
        sun->UpdateOrbitRails(0.0);
        const glm::dmat3 drawn = glm::dmat3(moon->GetBodyDrawTransform(sun));
        CHECK_TRUE(mat_close(drawn, moon->orient, 1e-9),
                   "inclined moon drawn under sun: rot == ancestor tilt");
        CHECK_TRUE(!mat_close(drawn, moon->getRotFrame()->orient, 1e-6),
                   "inclined moon: bare orient misses the ancestor tilt");
        moon->orient = glm::dmat3(1.0);
        sun->UpdateOrbitRails(0.0);
    }

    // ---- #489: the frame-switch tripwires scale with what they measure ----
    {
        // Near home the flat epsilon is still the demand -- that is where these
        // identities actually measure 0.0 (measured at a same-body switch: comErr,
        // hullErr and velErr all exactly zero), so scaling must not loosen it.
        CHECK_NEAR(continuityTol(0.0), 1e-6, 0.0, "floor is the old 1e-6");
        CHECK_NEAR(continuityTol(1e5), 1e-6, 0.0, "1e5 m of scale buys no slack");
        CHECK_NEAR(continuityTol(kContinuityAbs / kContinuityRel), 1e-6, 1e-12,
                   "crossover at 1e6 m of scale");
        CHECK_TRUE(continuityTol(-5.0) == kContinuityAbs,
                   "never tighter than the floor");
        CHECK_TRUE(continuityTol(1e12) > continuityTol(1e10),
                   "looser as the scale grows");
        CHECK_NEAR(continuityTol(1.36571076e10), 0.0136571076, 1e-12,
                   "a Kerbol-distance switch buys 1.4 cm, not metres");

        // THE BUG, pinned. At a Kerbin -> Kerbol switch the hull identity came in
        // one ulp off and the old flat 1e-6 aborted on it (tmp/p489_r2.log). The
        // scale is |p0|, the root distance moveToFrame compares against.
        const double ref = 1.36571076e10;         // |p0| at that switch
        const double hullErr = 1.90736318e-06;    // measured
        const double ulp = std::nextafter(ref, HUGE_VAL) - ref;
        CHECK_NEAR(ulp, 1.9073486328125e-06, 1e-18, "one ulp at 1.37e10 m");
        CHECK_TRUE(!(hullErr < kContinuityAbs),
                   "the old flat 1e-6 still fails a one-ulp round-off");
        CHECK_TRUE(hullErr < continuityTol(ref), "the scaled tolerance clears it");
        CHECK_NEAR(continuityTol(ref) / ulp, 7160.0, 1.0,
                   "7160 ulp of margin at the Kerbol distance");
        CHECK_TRUE(continuityTol(ref) < 1.0,
                   "a metre-scale break still trips it at Kerbol distance");
        // HEAD's own hullErr at a same-body Kerbin switch (1.16415322e-10, printed
        // by the [mvf] line) stays four orders inside the flat floor: scaling has
        // not made anything near home loose.
        CHECK_TRUE(1.16415322e-10 < kContinuityAbs,
                   "HEAD's same-body hullErr stays inside 1e-6");
    }

    // ---- #850: driven (building) frames -------------------------------------
    // The tree gains a third level: a frame whose origin MOVES and whose axes
    // turn with a building, under a body frame. These pins guard the three
    // generalizations that makes necessary: total omega, the parent-spin
    // transport in root_vel/root_accel, and the offset gravity centre.
    {
        printf("== #850 body-frame identities ==\n");
        // The existing tree must not move: for every body frame the new
        // quantities reduce to the old ones.
        Frame *all[] = { sun, eerbon, eerbon_rot, moon, moon_rot };
        for(Frame *f : all) {
            CHECK_TRUE(dvec_close(f->omega_total, f->omega(), 1e-15),
                       "body frame: omega_total == own omega");
            CHECK_TRUE(glm::length(f->root_accel) == 0.0,
                       "body frame: root_accel is zero (patched conics)");
            CHECK_TRUE(glm::length(f->gravity_center) == 0.0,
                       "body frame: gravity centre is the origin");
            CHECK_TRUE(!f->isNonInertial() || f->rotating,
                       "body frame: non-inertial iff rotating");
        }

        printf("== #850 two-spin composition ==\n");
        // A driven node under the SPINNING eerbon_rot: the case the old
        // two-level tree never had. The law supplies parent-axis state.
        StaticBuilding stub;   // never dereferenced; marks the node driven
        Frame *bub = new Frame;
        bub->name = "platform bubble";
        bub->parent = eerbon_rot;
        bub->body = eerbon->body;
        bub->rot_frame = eerbon_rot;
        bub->building = &stub;
        eerbon_rot->children.push_back(bub);

        const glm::dvec3 p(6.0e6, 1.0e6, 2.0e6);      // parent-axis position
        const glm::dvec3 v(30.0, -40.0, 50.0);        // parent-axis velocity
        const glm::dvec3 a(-0.001, 0.002, -0.0005);   // parent-axis accel
        const glm::dmat3 O = glm::dmat3(glm::rotate(0.7, glm::dvec3(0, 0, 1)));
        const glm::dvec3 w_own(0.01, -0.02, 0.03);    // local-axis own spin
        sun->UpdateOrbitRails(123.45);
        bub->SetDrivenState(p, v, a, O, w_own);
        bub->rail_time = 123.45;

        const glm::dvec3 wp = eerbon_rot->omega_total;   // parent spin, local
        // root_vel carries the parent-spin transport of the origin.
        const glm::dvec3 want_vel = eerbon_rot->root_vel
            + eerbon_rot->root_orient * (v + glm::cross(wp, p));
        CHECK_TRUE(dvec_close(bub->root_vel, want_vel, 1e-8),
                   "driven root_vel includes parent-spin transport");
        // Without the transport term the answer is off by ~omega_p * |p| --
        // kilometres per second here, not round-off.
        const glm::dvec3 no_transport = eerbon_rot->root_vel
            + eerbon_rot->root_orient * v;
        CHECK_TRUE(glm::length(bub->root_vel - no_transport) > 1.0,
                   "the transport term is load-bearing here, not round-off");
        // Total omega = own + parent's, in local axes.
        CHECK_TRUE(dvec_close(bub->omega_total,
                              w_own + glm::transpose(O) * wp, 1e-12),
                   "driven omega_total composes own + ancestor spin");
        // root_accel: law + Coriolis/centrifugal transport of the origin.
        const glm::dvec3 want_acc = eerbon_rot->root_accel
            + eerbon_rot->root_orient * (a + 2.0 * glm::cross(wp, v)
                + glm::cross(wp, glm::cross(wp, p)));
        CHECK_TRUE(dvec_close(bub->root_accel, want_acc, 1e-12),
                   "driven root_accel composes law + parent transport");
        // The gravity centre is the BODY centre, in local axes.
        CHECK_TRUE(dvec_close(bub->root_orient * bub->gravity_center + bub->root_pos,
                              eerbon->root_pos, 1e-6),
                   "gravity centre maps to the body centre in root space");
        CHECK_TRUE(bub->isNonInertial(), "driven node is non-inertial");
        CHECK_TRUE(bub->getNonRotFrame() == eerbon,
                   "getNonRotFrame walks a bubble under rot to the body inertial");

        // Point-velocity identity: a point at local (q, u) has the same root
        // velocity composed through the node as through the parent directly.
        const glm::dvec3 q(12.0, -7.0, 3.0), u(1.5, 2.5, -0.5);
        const glm::dvec3 via_node = bub->root_vel
            + bub->root_orient * (u + bub->GetStasisVelocity(q));
        const glm::dvec3 via_parent = eerbon_rot->root_vel
            + eerbon_rot->root_orient * (v + glm::cross(wp, p)
                + O * (u + glm::cross(w_own, q)) + glm::cross(wp, O * q));
        CHECK_TRUE(dvec_close(via_node, via_parent, 1e-8),
                   "point velocity: node composition == parent composition");

        // Switch algebra (moveToFrame's formulas) across the third level:
        // root pos/vel/spin are invariant, both directions.
        /* The SHIPPED law, not a copy of it. A test-local spelling could not see
           #946: Vehicle::comStateIn had the same algebra with the new frame's
           stasis missing, and these invariants were green either way. */
        const auto switch_state = [](Frame *from, Frame *to,
                                        const glm::dvec3 &pf, const glm::dvec3 &vf,
                                        const glm::dvec3 &wf,
                                        glm::dvec3 &pt, glm::dvec3 &vt, glm::dvec3 &wt) {
            switchFrameState(from, to, pf, vf, pt, vt);
            wt = switchFrameSpin(from, to, wf);
        };
        const glm::dvec3 p0(25.0, 8.0, -4.0), v0(0.2, -0.1, 0.05), w0(0.01, 0.02, -0.01);
        const glm::dvec3 root_p0 = bub->root_orient * p0 + bub->root_pos;
        const glm::dvec3 root_v0 = bub->root_vel
            + bub->root_orient * (v0 + bub->GetStasisVelocity(p0));
        const glm::dvec3 root_w0 = bub->root_orient * (w0 + bub->omega_total);
        glm::dvec3 p1, v1, w1, p2, v2, w2;
        switch_state(bub, eerbon_rot, p0, v0, w0, p1, v1, w1);
        switch_state(eerbon_rot, bub, p1, v1, w1, p2, v2, w2);
        CHECK_TRUE(dvec_close(bub->root_orient * p2 + bub->root_pos, root_p0, 1e-7),
                   "bubble->rot->bubble round trip preserves root position");
        CHECK_TRUE(dvec_close(bub->root_vel
                              + bub->root_orient * (v2 + bub->GetStasisVelocity(p2)),
                              root_v0, 1e-7),
                   "bubble->rot->bubble round trip preserves root velocity");
        CHECK_TRUE(dvec_close(bub->root_orient * (w2 + bub->omega_total), root_w0, 1e-10),
                   "bubble->rot->bubble round trip preserves root spin");
        // And the one-way switch lands on the parent's own composition.
        CHECK_TRUE(dvec_close(eerbon_rot->root_orient * p1 + eerbon_rot->root_pos,
                              root_p0, 1e-7),
                   "bubble->rot preserves root position");
        CHECK_TRUE(dvec_close(eerbon_rot->root_vel
                              + eerbon_rot->root_orient * (v1 + eerbon_rot->GetStasisVelocity(p1)),
                              root_v0, 1e-7),
                   "bubble->rot preserves root velocity");

        eerbon_rot->children.pop_back();
        delete bub;
        sun->UpdateOrbitRails(123.45);
    }

    {
        printf("== #946: state read in the body's SPIN frame ==\n");
        // The surface readout's leg: a hull's trajectory lives in the body's
        // INERTIAL node (on rails, or parked while the pad is modelled there),
        // and the horizon-speed readout asks for it in the body's ROTATING node.
        // Dropping the new frame's stasis left that figure at the INERTIAL speed
        // -- 183 m/s of Kerbin's own spin too high for a hull standing still on
        // the ground, and the readout jumped by exactly that when the ship came
        // off the rails. eerbon's spin is 0.001 rad/s at a 100 km radius, so the
        // co-rotation this leg must remove is 100 m/s.
        Frame *iner = eerbon;
        Frame *spin = eerbon_rot;
        const double w = spin->rot_ang_speed;
        const double R = 1.0e5;
        const glm::dvec3 ps(R, 0.0, 0.0);      // on the equator, spin-frame axes
        glm::dvec3 pi, vi;
        switchFrameState(spin, iner, ps, glm::dvec3(0.0), pi, vi);
        CHECK_NEAR(glm::length(vi), w * R, 1e-9,
                   "a hull parked on the deck moves with the ground (inertial node)");
        // And back the other way -- the readout's direction. Parked must read 0.
        glm::dvec3 ps2, vs2;
        switchFrameState(iner, spin, pi, vi, ps2, vs2);
        CHECK_TRUE(dvec_close(ps2, ps, 1e-6), "inertial -> rot round trip: position");
        CHECK_NEAR(glm::length(vs2), 0.0, 1e-9,
                   "a hull parked on the deck reads 0 in the rotating frame");
        // Rolling along the deck at 20 m/s: the inertial figure is the ground's
        // motion plus the roll, and the rotating figure is the roll ALONE -- which
        // is what the HUD's hs means.
        const glm::dvec3 deck(0.0, 0.0, 20.0);
        glm::dvec3 qi, vqi;
        switchFrameState(spin, iner, ps, deck, qi, vqi);
        CHECK_NEAR(glm::length(vqi - vi), 20.0, 1e-9,
                   "inertial speed = ground motion + deck speed");
        glm::dvec3 qs, vqs;
        switchFrameState(iner, spin, qi, vqi, qs, vqs);
        CHECK_TRUE(dvec_close(vqs, deck, 1e-9),
                   "rotating-frame speed == deck speed, spin not counted twice");
        // The spin law has the same shape (omega_old added, omega_new subtracted).
        // Both are along local +Y here, so the inertial node sees their sum.
        const glm::dvec3 ws(0.0, 0.5, 0.0);
        const glm::dvec3 wi = switchFrameSpin(spin, iner, ws);
        const glm::dvec3 back = switchFrameSpin(iner, spin, wi);
        CHECK_TRUE(dvec_close(back, ws, 1e-12), "spin switch round trip");
        CHECK_NEAR(glm::length(wi), 0.5 + w, 1e-12,
                   "spin into the inertial node carries the frame's own spin");
    }

    {
        printf("== #850 orbital station: weightless at the origin ==\n");
        // A body-fixed station on a circular orbit. A ship at rest at the
        // frame origin feels gravity + fictitious = 0: the -A term cancels
        // the body's pull. This is the pin that catches a missing transport
        // or a wrong sign in GetFictitiousAccel.
        const double mu = 3.2484e14;
        const double r = 6.0518e6 + 400e3;
        glm::dvec3 ep, ev;
        CHECK_TRUE(buildingorbit::circularEpochState(r, 0.3, 0.2, 0.0, mu, ep, ev),
                   "circular epoch state builds");
        Frame *body = new Frame;   // inertial body node at the root
        body->name = "body";
        body->parent = sun;
        body->pos = glm::dvec3(0);   // at the root: parent axes == root axes
        body->orient = glm::dmat3(1.0);
        sun->children.push_back(body);
        StaticBuilding stub;
        Frame *st = new Frame;
        st->name = "station";
        st->parent = body;
        st->building = &stub;
        st->rot_frame = nullptr;
        body->children.push_back(st);
        sun->UpdateOrbitRails(0.0);
        const buildingorbit::State s = buildingorbit::stateAt(
            ep, ev, mu, buildingorbit::Mode::BodyFixed, glm::dvec3(0, 0, 1), 0.0, 0.0);
        st->SetDrivenState(s.pos, s.vel, s.accel, s.orient, s.omega_own);
        CHECK_TRUE(st->getNonRotFrame() == body,
                   "orbital bubble walks to the body inertial node");
        // Ship at rest at the origin: gravity toward gravity_center + fict.
        const glm::dvec3 gc = st->gravity_center;
        const double gr = glm::length(gc);
        const glm::dvec3 g_vec = (mu / (gr * gr)) * glm::normalize(gc);
        const glm::dvec3 fict = st->GetFictitiousAccel(glm::dvec3(0.0), glm::dvec3(0.0));
        CHECK_TRUE(glm::length(g_vec + fict) < 1e-9,
                   "station origin: gravity + fictitious == 0 (weightless)");
        CHECK_TRUE(glm::length(g_vec) > 7.0,
                   "the gravity being cancelled is real (~8 m/s2), not zero");
        // A ship 10 m off the origin feels only the tidal residue.
        const glm::dvec3 q(10.0, 0.0, 0.0);
        const glm::dvec3 gq = (mu / glm::length2(gc - q)) * glm::normalize(gc - q);
        const glm::dvec3 fq = st->GetFictitiousAccel(q, glm::dvec3(0.0));
        CHECK_TRUE(glm::length(gq + fq) < 1e-3,
                   "10 m off the station: tidal residue only");
        // Own spin is the orbit rate about local +Y (the LVLH normal).
        const double n = std::sqrt(mu / (r * r * r));
        CHECK_TRUE(dvec_close(st->omega_total, glm::dvec3(0.0, n, 0.0), 1e-12),
                   "circular LVLH: total omega == n about local +Y");
        body->children.clear();
        sun->children.pop_back();
        delete st; delete body;
    }

    {
        printf("== #850 air platform: rest feels the planet's g ==\n");
        // A wind-riding platform under a SPINNING body: the ship at rest on
        // the deck feels essentially full gravity (the frame terms are the
        // tiny centripetal/transport corrections), and the deck point's root
        // velocity matches the law's own inertial velocity.
        const double mu = 3.2484e14;
        const double R = 6.0518e6;
        const double alt = 56e3;
        const double lat = 10.0 * M_PI / 180.0;
        const double r = R + alt;
        const double east = 86.29;
        const double rate = buildingair::driftRate(east, r, lat);
        Frame *body = new Frame;
        body->name = "venus-like";
        body->parent = sun;
        body->pos = glm::dvec3(0);
        body->orient = glm::dmat3(1.0);
        sun->children.push_back(body);
        Frame *rot = new Frame;
        rot->name = "venus-like rot";
        rot->parent = body;
        rot->rotating = true;
        rot->initial_orient = glm::dmat3(1.0);
        rot->rot_ang_speed = 7.29e-5;   // fast spin: transport is visible
        body->children.push_back(rot);
        body->rot_frame = rot;
        StaticBuilding stub;
        Frame *pf = new Frame;
        pf->name = "platform";
        pf->parent = rot;
        pf->building = &stub;
        pf->rot_frame = rot;
        rot->children.push_back(pf);
        sun->UpdateOrbitRails(5000.0);
        const buildingair::State s = buildingair::stateAt(
            r, lat, 0.0, rate, M_PI, 5000.0);
        pf->SetDrivenState(s.pos, s.vel, s.accel, s.orient, s.omega_own);
        // Rest at the deck origin: net acceleration ~ g, downward (local -Z).
        const glm::dvec3 gc = pf->gravity_center;
        const double gr = glm::length(gc);
        const glm::dvec3 g_vec = (mu / (gr * gr)) * glm::normalize(gc);
        const glm::dvec3 fict = pf->GetFictitiousAccel(glm::dvec3(0.0), glm::dvec3(0.0));
        const glm::dvec3 net = g_vec + fict;
        const double g_expect = mu / (r * r);
        /* The fast test spin lifts the apparent g by ~omega^2 r cos^2(lat)
           (~0.03 m/s2 here) -- a real correction, so the band is 5 cm/s2. */
        CHECK_NEAR(glm::length(net), g_expect, 0.05,
                   "platform rest: |net accel| == local g within 5 cm/s2");
        CHECK_TRUE(glm::dot(net, glm::dvec3(0.0, 0.0, 1.0)) < -8.0,
                   "platform rest: net accel points down (local -Z)");
        // The frame terms are corrections, not the story: |fict| << g.
        CHECK_TRUE(glm::length(fict) < 0.06,
                   "platform frame terms are tiny against g");
        // The deck origin's root velocity equals the law's inertial velocity
        // composed by hand through the spinning parent.
        const glm::dvec3 want = rot->root_vel
            + rot->root_orient * (s.vel + glm::cross(rot->omega_total, s.pos));
        CHECK_TRUE(dvec_close(pf->root_vel, want, 1e-8),
                   "platform root_vel == drift + co-rotation, composed");
        CHECK_TRUE(glm::length(pf->root_vel) > east,
                   "the co-rotation adds to the drift, not cancels it");
        rot->children.clear();
        body->children.clear();
        sun->children.pop_back();
        delete pf; delete rot; delete body;
    }

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
