//
// Headless golden-value test for the ship's mass properties:
// Vehicle::get_center_of_mass() and Vehicle::getInertia() (src/vehicle.h).
//
// getInertia() is the quantity a torque actually moves -- the denominator of
// a reaction wheel's authority and of the autopilot slew law -- and it had no
// test: test_attitude and test_slew3d simulate their own hardcoded Ix/Iz, and
// test_thrust/test_rotation pin Bullet's readers on a LONE box, never on a
// multi-part assembly. e2e 22 exercises the real thing but only through
// ratio thresholds, so a 2-3x error in the tensor would still pass.
//
// The reference values here are computed independently in the test: the
// analytic solid-box inertia m/3*(hy^2+hz^2) etc. (NOT read back from
// Bullet), assembled with the parallel-axis theorem about the ship COM,
// including the products of inertia -- so a transposed rotation, a missing
// R*I*R^T term, or a COM taken about the wrong point all fail.
//
// Boxes are deliberately non-cubic and parts deliberately rotated: a cube or
// an axis-aligned part hides exactly the axis-convention mistakes this is
// here to catch.
//
// No physics world and no GL context are needed: the bodies are constructed
// directly and positioned with setWorldTransform, and getInertia() only reads
// mass, the local inertia diagonal, and the centre-of-mass transform.
//
// Runs from the repo root:
//   make test   (or: ./test_inertia)

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "vehicle.h"   // Vehicle + Part (inline); body.h pulls in Bullet

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_NEAR(actual, expected, tol, msg)                                 \
    do {                                                                       \
        g_checks++;                                                            \
        double _a = (actual), _e = (expected);                                 \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > (tol)) {                \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g, tol %.3g)\n",               \
                   msg, _a, _e, (double)(tol));                                \
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

/* --- the independent reference math ------------------------------------- */

/* Solid box of HALF-extents (hx,hy,hz) about its own centre. Written out
   rather than asking Bullet, so this stays an independent reference. */
static glm::dvec3 boxInertia(double m, double hx, double hy, double hz) {
    return glm::dvec3(m / 3.0 * (hy * hy + hz * hz),
                      m / 3.0 * (hx * hx + hz * hz),
                      m / 3.0 * (hx * hx + hy * hy));
}

/* One part's contribution to the ship tensor about `com`: its own inertia
   rotated into world axes, plus the parallel-axis term. */
static glm::dmat3 partInertia(double m, const glm::dvec3 &il,
                              const glm::dmat3 &R, const glm::dvec3 &pos,
                              const glm::dvec3 &com) {
    const glm::dmat3 il_diag(il.x, 0.0, 0.0,
                             0.0, il.y, 0.0,
                             0.0, 0.0, il.z);
    const glm::dvec3 d = pos - com;
    return R * il_diag * glm::transpose(R)
         + m * (glm::dot(d, d) * glm::dmat3(1.0) - glm::outerProduct(d, d));
}

/* --- the hand-built ship ------------------------------------------------ */

/* getInertia() is protected (it serves the Kerbal subclass and the internal
   slew paths). Re-expose it through a subclass so the test can call it
   without widening the production API. ~Vehicle is virtual, so the Vehicle*
   the rest of the code holds still tears down correctly. */
struct TestVehicle : Vehicle {
    using Vehicle::getInertia;
};

struct Ship {
    TestVehicle *v;
    std::deque<PartDef> defs;   // deque: push_back never invalidates Part::def
    /* the reference data, parallel to v->parts */
    std::vector<double> mass;
    std::vector<glm::dvec3> il;
    std::vector<glm::dmat3> rot;
    std::vector<glm::dvec3> pos;
};

/* A box part at an explicit world pose. Its local inertia is set to the
   ANALYTIC box value (not Bullet's calculateLocalInertia), so the assembly
   under test is getInertia()'s, and the box formula is checked too. */
static Part *addBox(Ship &s, const char *name, double m,
                    double hx, double hy, double hz,
                    const glm::dvec3 &pos, const glm::dmat3 &rot) {
    const glm::dvec3 il = boxInertia(m, hx, hy, hz);

    btBoxShape *shape = new btBoxShape(btVector3(hx, hy, hz));
    btRigidBody::btRigidBodyConstructionInfo ci(m, 0, shape,
        btVector3(il.x, il.y, il.z));
    Body *b = new Body;
    b->model  = nullptr;      // no GL model in a headless test
    b->btBody = new btRigidBody(ci);
    b->mass   = m;

    /* Position it without a motion state or a world: getInertia() reads only
       the centre-of-mass transform. The orientation goes through a
       quaternion because glm is column-major and btMatrix3x3::setValue is
       row-major -- the trap physics.cpp's GetOrient comment warns about. */
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos.x, pos.y, pos.z));
    const glm::dquat q = glm::quat_cast(rot);
    t.setRotation(btQuaternion(q.x, q.y, q.z, q.w));
    b->btBody->setWorldTransform(t);

    PartDef d;
    d.name   = name;
    d.mass   = (float)m;
    d.radius = (float)hx;
    d.height = (float)(2.0 * hz);
    s.defs.push_back(d);

    Part *p = new Part;
    p->body   = b;
    p->def    = &s.defs.back();
    p->stage  = 1;
    p->parent = s.v->parts.empty() ? nullptr : s.v->parts[0];
    s.v->parts.push_back(p);

    s.mass.push_back(m);
    s.il.push_back(il);
    s.rot.push_back(rot);
    s.pos.push_back(pos);
    return p;
}

static void destroyShip(Ship &s) {
    /* onRails keeps ~Vehicle off Detach/RemoveBody: nothing here was ever
       added to a physics world. */
    s.v->onRails = true;
    delete s.v;
}

/* --- the assertions ------------------------------------------------------ */

static void checkCom(Ship &s, const char *msg) {
    glm::dvec3 want(0.0);
    double mtot = 0.0;
    for(size_t i = 0; i < s.pos.size(); i++) {
        want += s.mass[i] * s.pos[i];
        mtot += s.mass[i];
    }
    want /= mtot;
    const glm::dvec3 got = s.v->get_center_of_mass();
    const double tol = 1e-9 * std::max(1.0, glm::length(want));
    CHECK_NEAR(got.x, want.x, tol, msg);
    CHECK_NEAR(got.y, want.y, tol, msg);
    CHECK_NEAR(got.z, want.z, tol, msg);
}

/* Element-wise, with a relative tolerance: the tensor spans kg m^2 over
   parts metres apart, so the entries differ by orders of magnitude. */
static void checkInertia(Ship &s, const char *msg) {
    glm::dvec3 want_com(0.0);
    double mtot = 0.0;
    for(size_t i = 0; i < s.pos.size(); i++) {
        want_com += s.mass[i] * s.pos[i];
        mtot += s.mass[i];
    }
    want_com /= mtot;

    glm::dmat3 want(0.0);
    for(size_t i = 0; i < s.pos.size(); i++) {
        want += partInertia(s.mass[i], s.il[i], s.rot[i], s.pos[i], want_com);
    }

    const glm::dmat3 got = s.v->getInertia();
    double scale = 0.0;
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) { scale = std::max(scale, std::fabs(want[c][r])); }
    }
    const double tol = 1e-9 * std::max(1.0, scale);
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
            char buf[128];
            snprintf(buf, sizeof(buf), "%s [col %d row %d]", msg, c, r);
            CHECK_NEAR(got[c][r], want[c][r], tol, buf);
        }
    }

    /* Two properties the assembly must have regardless of the reference:
       the tensor is symmetric (products of inertia come in pairs) and
       positive definite (a real body resists rotation about every axis). */
    for(int c = 0; c < 3; c++) {
        for(int r = c + 1; r < 3; r++) {
            char buf[128];
            snprintf(buf, sizeof(buf), "%s symmetric [%d][%d]", msg, c, r);
            CHECK_NEAR(got[c][r], got[r][c], tol, buf);
        }
    }
    /* Positive definite by Sylvester's criterion: all three leading principal
       minors positive. Exact for a symmetric matrix, and it needs no
       eigen-solver (glm's is a gtx extension). */
    const double minor1 = got[0][0];
    const double minor2 = got[0][0] * got[1][1] - got[1][0] * got[0][1];
    const double minor3 = glm::determinant(got);
    char pdbuf[192];
    snprintf(pdbuf, sizeof(pdbuf), "%s: positive-definite inertia", msg);
    CHECK_TRUE(minor1 > 0.0 && minor2 > 0.0 && minor3 > 0.0, pdbuf);
}

/* --- the cases ----------------------------------------------------------- */

/* One part at the origin, unrotated: the COM is the part and the tensor is
   just its own local inertia. The degenerate baseline. */
static void test_single() {
    Ship s; s.v = new TestVehicle;
    addBox(s, "solo", 1200.0, 1.0, 2.0, 3.0,
           glm::dvec3(0.0), glm::dmat3(1.0));
    checkCom(s, "single part: COM");
    checkInertia(s, "single part: inertia");
    destroyShip(s);
}

/* Two identical boxes symmetric about the origin: the COM stays at the
   origin and the tensor is 2x the local inertia plus the parallel-axis
   terms. Catches a COM that ignores the mass weighting. */
static void test_symmetric_pair() {
    Ship s; s.v = new TestVehicle;
    addBox(s, "a", 800.0, 0.5, 1.5, 2.5,
           glm::dvec3(0.0, 0.0,  4.0), glm::dmat3(1.0));
    addBox(s, "b", 800.0, 0.5, 1.5, 2.5,
           glm::dvec3(0.0, 0.0, -4.0), glm::dmat3(1.0));
    checkCom(s, "symmetric pair: COM");
    checkInertia(s, "symmetric pair: inertia");
    destroyShip(s);
}

/* Unequal masses: the COM must shift toward the heavy one, and by the right
   fraction. A stack-like ship (light capsule, heavy tank). */
static void test_unequal_masses() {
    Ship s; s.v = new TestVehicle;
    addBox(s, "capsule", 300.0, 1.0, 1.0, 1.0,
           glm::dvec3(0.0, 0.0, 6.0), glm::dmat3(1.0));
    addBox(s, "tank", 9000.0, 2.0, 2.0, 2.5,
           glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    addBox(s, "engine", 1500.0, 1.5, 1.5, 1.0,
           glm::dvec3(0.0, 0.0, -4.0), glm::dmat3(1.0));

    /* the COM independently, as a fraction check on top of checkCom */
    const double z = (300.0 * 6.0 + 9000.0 * 0.0 + 1500.0 * -4.0)
                   / (300.0 + 9000.0 + 1500.0);
    CHECK_NEAR(s.v->get_center_of_mass().z, z, 1e-9,
               "unequal masses: COM sits at the mass-weighted z");
    checkCom(s, "unequal masses: COM");
    checkInertia(s, "unequal masses: inertia");
    destroyShip(s);
}

/* A rotated part: exercises the R * Ilocal * R^T term. A 90 deg turn about
   X swaps the y and z principal moments, so getting the rotation term wrong
   (or transposed) shows up immediately -- and the box is non-cubic so the
   swap is visible. */
static void test_rotated_part() {
    Ship s; s.v = new TestVehicle;
    /* 90 deg about +X: columns are the images of X, Y, Z. */
    const glm::dmat3 rotX90(glm::dvec3(1, 0, 0),
                            glm::dvec3(0, 0, 1),
                            glm::dvec3(0, -1, 0));
    addBox(s, "base", 2000.0, 1.0, 1.0, 4.0,
           glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    addBox(s, "turned", 2000.0, 1.0, 1.0, 4.0,
           glm::dvec3(0.0, 0.0, 6.0), rotX90);
    checkCom(s, "rotated part: COM");
    checkInertia(s, "rotated part: inertia (R*I*R^T)");
    destroyShip(s);
}

/* The general case: a radial-attachment-like layout. An arbitrary rotation
   (not axis-aligned) plus offsets on all three axes, so every product of
   inertia is nonzero and a dropped or transposed term cannot hide. */
static void test_general_assembly() {
    Ship s; s.v = new TestVehicle;
    /* an arbitrary proper rotation, built from two axis turns and
       re-orthonormalised by glm so it is exactly a rotation */
    const glm::dmat3 turn = glm::mat3_cast(
        glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, 2.0, -1.5)))
      * glm::angleAxis(-1.1, glm::normalize(glm::dvec3(-2.0, 0.5, 3.0))));

    addBox(s, "core",   11000.0, 2.25, 2.25, 2.5,
           glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    addBox(s, "core2",  11000.0, 2.25, 2.25, 2.5,
           glm::dvec3(0.0, 0.0, -5.0), glm::dmat3(1.0));
    addBox(s, "booster", 5000.0, 1.5, 1.5, 2.5,
           glm::dvec3(4.0, 0.3, -1.7), turn);
    addBox(s, "cap",     1100.0, 1.0, 1.0, 1.0,
           glm::dvec3(-0.4, 0.9, 6.2), glm::transpose(turn));
    checkCom(s, "general assembly: COM");
    checkInertia(s, "general assembly: inertia");

    /* the products of inertia really are nonzero here -- if this ever goes
       vacuous the case stops testing what it is for */
    const glm::dmat3 I = s.v->getInertia();
    bool anyOffDiag = false;
    for(int c = 0; c < 3 && !anyOffDiag; c++) {
        for(int r = c + 1; r < 3; r++) {
            if(std::fabs(I[c][r]) > 1.0) { anyOffDiag = true; break; }
        }
    }
    g_checks++;
    if(!anyOffDiag) {
        g_failures++;
        printf("FAIL: general assembly produced no products of inertia -- "
               "the case is vacuous\n");
    }
    destroyShip(s);
}

int main() {
    printf("== ship mass properties (src/vehicle.h) ==\n");
    test_single();
    test_symmetric_pair();
    test_unequal_masses();
    test_rotated_part();
    test_general_assembly();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_inertia: all checks passed\n");
    return 0;
}
