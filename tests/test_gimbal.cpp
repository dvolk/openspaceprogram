//
// Engine-gimbal (TVC) law (src/gimbal.h, header-only pure math): the 2-DOF
// thrust-direction rotation, the rate-limited actuator step, and the
// authority torque. Pinned without Bullet/GL so the law stands independent
// of the render/physics chain (like test_drag, test_orbit).
//
//   gimbalDir:   zero angles -> the authored axis; closed form
//                (cos p sin y, -sin p, cos p cos y) in the part frame pins
//                the pitch-then-yaw composition; unit length at any angle;
//                the stacked-engine sign convention (pitch>0 -> -up lateral,
//                yaw>0 -> +right lateral, so stick[1]/stick[2] turn the nose
//                the same way as the wheel torques do).
//   gimbalStep:  rate limit, no overshoot, exact arrival, symmetric recenter.
//   gimbalTorque: == rel x (T * dir); a centered axial engine on the stack
//                axis produces NO attitude torque.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gimbal.h"

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

#define CHECK_NEAR(actual, expected, rel_tol, msg)                            \
    do {                                                                      \
        g_checks++;                                                           \
        double _a = (actual), _e = (expected);                                \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > rel_tol * std::fabs(_e)) { \
            g_failures++;                                                     \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);          \
        }                                                                     \
    } while (0)

static void checkVec(const glm::dvec3 &a, const glm::dvec3 &e, double tol,
                     const char *msg) {
    g_checks++;
    if(glm::length(a - e) > tol) {
        g_failures++;
        printf("FAIL: %s (got [%.6f %.6f %.6f], want [%.6f %.6f %.6f])\n",
               msg, a.x, a.y, a.z, e.x, e.y, e.z);
    }
}

// The part frame as a rotation matrix (columns = local X, Y, Z in world).
static glm::dmat3 ident() { return glm::dmat3(1.0); }

static void test_dir_identity() {
    printf("== gimbalDir: zero angles ==\n");
    checkVec(gimbalDir(ident(), 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), 0.0,
             "zero angles -> authored +Z exactly");
    // A rotated part (nose along world +X): zero angles -> the rotated axis.
    const glm::dmat3 rot(glm::dvec3(0.0, 0.0, -1.0),   // local X
                         glm::dvec3(0.0, 1.0, 0.0),    // local Y
                         glm::dvec3(1.0, 0.0, 0.0));   // local Z -> world +X
    checkVec(gimbalDir(rot, 0.0, 0.0), glm::dvec3(1.0, 0.0, 0.0), 1e-12,
             "zero angles follow the part's rotation");
}

static void test_dir_closed_form() {
    printf("== gimbalDir: composition closed form ==\n");
    // Pitch (inner ring, about local X) then yaw (outer ring, about local Y),
    // both applied to the local +Z, has the closed form
    //   (cos p sin y, -sin p, cos p cos y)
    // in the part frame. Pinning it kills any silent axis/order swap.
    const double p = 0.3, y = 0.2;
    const glm::dvec3 want(std::cos(p) * std::sin(y),
                          -std::sin(p),
                          std::cos(p) * std::cos(y));
    checkVec(gimbalDir(ident(), p, y), want, 1e-12,
             "pitch-then-yaw about local X then local Y");
    // Unit length at a large deflection.
    const glm::dvec3 big = gimbalDir(ident(), 0.7853981633974483, 0.7853981633974483);
    CHECK_NEAR(glm::length(big), 1.0, 1e-12, "45 deg/45 deg stays unit");
    // Mirror symmetry in each angle.
    checkVec(gimbalDir(ident(), -p, y),
             glm::dvec3(want.x, -want.y, want.z), 1e-12, "pitch mirrors");
    checkVec(gimbalDir(ident(), p, -y),
             glm::dvec3(-want.x, want.y, want.z), 1e-12, "yaw mirrors");
    // The rotated part: the whole picture rides the part frame.
    const glm::dmat3 rot(glm::dvec3(0.0, 0.0, -1.0),
                         glm::dvec3(0.0, 1.0, 0.0),
                         glm::dvec3(1.0, 0.0, 0.0));
    checkVec(gimbalDir(rot, p, y), rot * want, 1e-12,
             "gimbal follows the part frame");
}

static void test_dir_sign_convention() {
    printf("== gimbalDir: wheel-feel sign convention ==\n");
    /* A stacked engine (part frame == ship frame, thrust along the ship's
       +Z nose, engine BELOW the COM). applyRotationForce drives the nose
       with wheel torques about -right (W/S, stick[1]) and -up (A/D,
       stick[2]). TVC answers the SAME stick signs with the SAME nose
       rotation: lateral thrust -up for pitch>0 and +right for yaw>0,
       because torque = (-a z) x F puts those forces on the same sides. */
    const double eps = 1e-3;
    const glm::dvec3 dPitch = gimbalDir(ident(), eps, 0.0);
    CHECK_TRUE(dPitch.y < 0.0, "pitch>0 (stick[1]>0) -> lateral thrust -up");
    CHECK_TRUE(std::fabs(dPitch.x) < 1e-15 && std::fabs(dPitch.z) > 0.999,
               "pure pitch stays in the Y-Z plane");
    const glm::dvec3 dYaw = gimbalDir(ident(), 0.0, eps);
    CHECK_TRUE(dYaw.x > 0.0, "yaw>0 (stick[2]>0) -> lateral thrust +right");
    CHECK_TRUE(std::fabs(dYaw.y) < 1e-15 && std::fabs(dYaw.z) > 0.999,
               "pure yaw stays in the X-Z plane");
    // And the resulting torque about the COM matches the wheel signs.
    const glm::dvec3 rel(0.0, 0.0, -2.0);   // engine 2 m below the COM
    const glm::dvec3 tauP = gimbalTorque(rel, 1.0, dPitch);
    CHECK_TRUE(tauP.x < 0.0 && std::fabs(tauP.y) < 1e-12,
               "pitch>0 torques about -right, like the W/S wheel axis");
    const glm::dvec3 tauY = gimbalTorque(rel, 1.0, dYaw);
    CHECK_TRUE(tauY.y < 0.0 && std::fabs(tauY.x) < 1e-12,
               "yaw>0 torques about -up, like the A/D wheel axis");
}

static void test_rot() {
    printf("== gimbalRot: the swung frame (plume/nozzle draw) ==\n");
    // Zero angles: EXACTLY the authored frame (every non-TVC part, always).
    const glm::dmat3 rot(glm::dvec3(0.0, 0.0, -1.0),
                         glm::dvec3(0.0, 1.0, 0.0),
                         glm::dvec3(1.0, 0.0, 0.0));
    checkVec(gimbalRot(rot, 0.0, 0.0)[0], rot[0], 0.0, "zero angles keep local X");
    checkVec(gimbalRot(rot, 0.0, 0.0)[1], rot[1], 0.0, "zero angles keep local Y");
    checkVec(gimbalRot(rot, 0.0, 0.0)[2], rot[2], 0.0, "zero angles keep local Z");
    // The draw frame and the force direction are ONE rotation: the frame's
    // +Z column is exactly gimbalDir, and the frame stays orthonormal
    // (a sheared plume mesh would show up here).
    const double p = 0.3, y = 0.2;
    const glm::dmat3 sw = gimbalRot(rot, p, y);
    checkVec(sw[2], gimbalDir(rot, p, y), 1e-12, "frame +Z == gimbalDir exactly");
    CHECK_NEAR(glm::length(sw[0]), 1.0, 1e-12, "swung local X stays unit");
    CHECK_NEAR(glm::length(sw[1]), 1.0, 1e-12, "swung local Y stays unit");
    CHECK_NEAR(glm::dot(sw[0], sw[1]), 0.0, 1e-12, "swung axes stay perpendicular");
    checkVec(glm::cross(sw[0], sw[1]), sw[2], 1e-12, "right-handed: x x y == z");
    // The swing happens ABOUT the part's own axes: at tiny pitch the frame's
    // +Z tips toward -its own +Y (the same first-order fact as gimbalDir).
    const double e = 1e-3;
    const glm::dmat3 swt = gimbalRot(rot, e, 0.0);
    CHECK_TRUE(glm::dot(swt[2] - rot[2], -rot[1]) > 0.9 * e,
               "tiny pitch tips +Z toward -local Y");
}

static void test_targets() {
    printf("== gimbalTargets: one shared lateral for the whole ship ==\n");
    const double g = 0.0873;   // 5 deg

    // An engine aligned with the ship: a -up lateral IS pitch +g, +right IS yaw +g
    // (the closed form of gimbalDir read backwards).
    double out[2];
    gimbalTargets(ident(), glm::dvec3(0.0, -g, 0.0), out);
    CHECK_NEAR(out[0], g, 1e-12, "lateral -up -> pitch +g");
    CHECK_NEAR(out[1], 0.0, 0.0, "pure pitch lateral -> no yaw");
    gimbalTargets(ident(), glm::dvec3(g, 0.0, 0.0), out);
    CHECK_NEAR(out[0], 0.0, 0.0, "pure yaw lateral -> no pitch");
    CHECK_NEAR(out[1], g, 1e-12, "lateral +right -> yaw +g");

    // The point of the shared vector: for an engine in ANY rolled or reversed
    // attitude, the first-order deviation gimbalDir achieves (dir - authored
    // +Z) equals the SAME lateral vector -- every nozzle pushes the same way,
    // so a ring adds authority instead of coupling pitch into roll. Tolerance
    // covers the second-order terms of a 5 deg swing (~4e-3).
    const glm::dmat3 rolls[3] = {
        glm::mat3_cast(glm::angleAxis(1.5707963267948966, glm::dvec3(0.0, 0.0, 1.0))),  // rolled 90
        glm::mat3_cast(glm::angleAxis(3.141592653589793, glm::dvec3(0.0, 0.0, 1.0))),   // rolled 180
        glm::mat3_cast(glm::angleAxis(3.141592653589793, glm::dvec3(1.0, 0.0, 0.0))),   // reversed
    };
    const glm::dvec3 lat = g * glm::dvec3(0.7, -0.7, 0.0);
    for(const glm::dmat3 &rot : rolls) {
        gimbalTargets(rot, lat, out);
        CHECK_TRUE(std::fabs(out[0]) <= g + 1e-12 && std::fabs(out[1]) <= g + 1e-12,
                   "projected angles stay within the lateral's magnitude");
        const glm::dvec3 dev = gimbalDir(rot, out[0], out[1]) - rot[2];
        checkVec(dev, lat, 1e-2, "achieved deviation == the shared lateral, any attitude");
    }
}

static void test_step() {
    printf("== gimbalStep: rate limit ==\n");
    double cur[2] = {0.0, 0.0};
    const double target[2] = {0.0873, -0.0873};   // +/- 5 deg
    const double rate = 1.5708;                   // 90 deg/s
    // One 20 ms step: exactly rate * h toward each target, one side each.
    gimbalStep(cur, target, rate, 0.02);
    CHECK_NEAR(cur[0], 1.5708 * 0.02, 1e-12, "positive side steps +rate*h");
    CHECK_NEAR(cur[1], -1.5708 * 0.02, 1e-12, "negative side steps -rate*h");
    // Monotone toward the target while far.
    double prev0 = cur[0], prev1 = cur[1];
    for(int i = 0; i < 20; i++) {
        gimbalStep(cur, target, rate, 0.02);
        CHECK_TRUE(cur[0] >= prev0, "monotone toward the positive target");
        CHECK_TRUE(cur[1] <= prev1, "monotone toward the negative target");
        prev0 = cur[0];
        prev1 = cur[1];
    }
    // Arrives exactly and never overshoots.
    gimbalStep(cur, target, rate, 10.0);
    CHECK_NEAR(cur[0], target[0], 0.0, "lands on the positive target exactly");
    CHECK_NEAR(cur[1], target[1], 0.0, "lands on the negative target exactly");
    // Recenters with the same bound (the engine-off case).
    const double zero[2] = {0.0, 0.0};
    gimbalStep(cur, zero, rate, 0.02);
    CHECK_NEAR(cur[0], target[0] - rate * 0.02, 1e-12, "recenters at rate*h");
    gimbalStep(cur, zero, rate, 100.0);
    CHECK_NEAR(cur[0], 0.0, 0.0, "recenter lands on zero exactly");
    CHECK_NEAR(cur[1], 0.0, 0.0, "recenter lands on zero exactly (neg side)");
}

static void test_torque() {
    printf("== gimbalTorque ==\n");
    const glm::dvec3 rel(0.0, 0.0, -2.0);
    // Matches the cross product the force path will actually produce.
    const glm::dvec3 dir = gimbalDir(ident(), 0.05, -0.03);
    const glm::dvec3 tau = gimbalTorque(rel, 50000.0, dir);
    checkVec(tau, glm::cross(rel, dir * 50000.0), 1e-9, "tau == rel x T*dir");
    // A centered, ungimballed engine on the stack axis: no torque at all.
    const glm::dvec3 axial = gimbalTorque(rel, 50000.0, glm::dvec3(0.0, 0.0, 1.0));
    CHECK_NEAR(glm::length(axial), 0.0, 1e-9, "axial thrust through the lever gives no torque");
    // Zero thrust, no torque.
    CHECK_NEAR(glm::length(gimbalTorque(rel, 0.0, dir)), 0.0, 0.0, "no thrust -> no torque");
    // Small-angle authority grows linearly with the angle.
    const double t1 = glm::length(gimbalTorque(rel, 50000.0, gimbalDir(ident(), 0.01, 0.0)));
    const double t2 = glm::length(gimbalTorque(rel, 50000.0, gimbalDir(ident(), 0.02, 0.0)));
    CHECK_NEAR(t2 / t1, 2.0, 1e-4, "torque ~ sin(angle): linear at small angles");
}

int main() {
    printf("== test_gimbal: the engine-gimbal law (src/gimbal.h) ==\n");
    test_dir_identity();
    test_dir_closed_form();
    test_dir_sign_convention();
    test_rot();
    test_targets();
    test_step();
    test_torque();
    printf("%s: %d checks, %d failures\n",
           g_failures ? "FAIL" : "PASS", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
