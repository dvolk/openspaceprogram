//
// Headless golden-value test for the ship's mass properties:
// Vehicle::get_center_of_mass() and Vehicle::getInertia() (src/vehicle.h),
// and the single compound rigid body Vehicle::rebuildCompound() builds from
// them.
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
// checkCompound() holds the same reference against what
// btCompoundShape::calculatePrincipalAxisTransform produces, and against the
// part poses derived back out of the resulting rigid body. A btRigidBody's
// transform is its CENTRE-OF-MASS transform and its inertia is stored
// diagonal, so the compound's children have to be re-based into the principal
// frame -- the subtlety that half exists to pin.
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
    /* frame S -- the ship-local frame the authored poses (Part::localPos /
       localRot) live in, and which the compound is built in. By convention S
       is the ROOT part's frame and the root's own authored pose is the
       identity, so S is just the first part's world pose. */
    bool haveS = false;
    glm::dvec3 sPos = glm::dvec3(0.0);
    glm::dmat3 sRot = glm::dmat3(1.0);
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
    /* the authored ship-local pose: the world pose above seen from S. This is
       the relation build_ship's attachPose gives a real ship, and the one
       partWorldPose inverts -- so checkCompound can ask for the world poses
       back and get exactly these. */
    if(!s.haveS) { s.sPos = pos; s.sRot = rot; s.haveS = true; }
    p->localPos = glm::transpose(s.sRot) * (pos - s.sPos);
    p->localRot = glm::transpose(s.sRot) * rot;
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

/* Element-wise rotation-matrix comparison (a dmat3 is column-major: [c][r]). */
static void checkRot(const glm::dmat3 &got, const glm::dmat3 &want,
                     double tol, const char *msg) {
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
            char buf[192];
            snprintf(buf, sizeof(buf), "%s [col %d row %d]", msg, c, r);
            CHECK_NEAR(got[c][r], want[c][r], tol, buf);
        }
    }
}

/* --- the ship as ONE rigid body (Vehicle::rebuildCompound) ---------------

   rebuildCompound() makes a btCompoundShape of the part hulls at their
   authored ship-local poses and re-bases the children into the principal
   (centre-of-mass) frame, because a btRigidBody's transform IS its COM
   transform and its inertia is stored diagonal. Four things are pinned
   here, in increasing order of how easily they go wrong:

     1. `principal`'s origin is the authored COM in S, and its basis rotates
        the body's diagonal inertia back onto the analytic tensor about that
        COM -- i.e. calculatePrincipalAxisTransform was read the right way
        round. (rebuildCompound asserts the same invariant itself on every
        build, so this also exercises that assert on a ship whose numbers the
        test knows independently.)
     2. put the ship body at an ARBITRARY world pose: every part's derived
        world pose must be that pose applied to its authored local pose, AND
        the compound's re-based child taken into world by the same body
        transform must land on it too. This is the check that matters.
        Deriving the body's pose from the parts' own live poses instead would
        cancel `principal` out of the round trip and prove nothing about it;
        and reading only `principal` would miss a re-base done the wrong way
        round (or skipped) entirely, because `principal` comes from the shape
        BEFORE the re-base and is self-consistent either way -- the children
        are the only thing the re-base actually changes.
     3. syncShipBody() -- mirroring the live parts onto the shadow body --
        must then give back exactly the world poses this test set by hand.
     4. partWorldPose() and its siblings -- the accessors the rest of the
        game reads -- must agree with compoundPartPose(). That agreement is
        the whole basis on which the switch-over later swaps one
        implementation for the other without touching a call site.   */
static void checkCompound(Ship &s, const char *msg) {
    s.v->rebuildCompound();
    char buf[192];
    if(s.v->shipBody == nullptr) {
        g_failures++;
        printf("FAIL: %s: rebuildCompound built no ship body\n", msg);
        return;
    }

    /* the child-index -> Part mapping picking will resolve hits through */
    g_checks++;
    bool mapOk = s.v->compoundParts.size() == s.v->parts.size();
    for(size_t i = 0; mapOk && i < s.v->parts.size(); i++) {
        if(s.v->compoundParts[i] != s.v->parts[i]) { mapOk = false; }
    }
    snprintf(buf, sizeof(buf), "%s: compoundParts is the part order", msg);
    CHECK_TRUE(mapOk, buf);

    /* the independent reference, in frame S, from the authored local poses */
    double mtot = 0.0;
    glm::dvec3 comS(0.0);
    for(size_t i = 0; i < s.pos.size(); i++) {
        mtot += s.mass[i];
        comS += s.mass[i] * s.v->parts[i]->localPos;
    }
    comS /= mtot;
    glm::dmat3 wantS(0.0);
    for(size_t i = 0; i < s.pos.size(); i++) {
        const Part *p = s.v->parts[i];
        wantS += partInertia(s.mass[i], s.il[i], p->localRot, p->localPos, comS);
    }

    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    Vehicle::fromBt(s.v->principal, pOrigin, pBasis);
    const double comTol = 1e-9 * std::max(1.0, glm::length(comS));
    for(int c = 0; c < 3; c++) {
        snprintf(buf, sizeof(buf), "%s: principal origin = the COM in S [%d]", msg, c);
        CHECK_NEAR(pOrigin[c], comS[c], comTol, buf);
    }

    const btVector3 &bi = s.v->shipBody->getLocalInertia();
    const glm::dmat3 diag(bi.getX(), 0.0, 0.0,
                          0.0, bi.getY(), 0.0,
                          0.0, 0.0, bi.getZ());
    const glm::dmat3 gotS = pBasis * diag * glm::transpose(pBasis);
    /* btMatrix3x3::diagonalize is a Jacobi iteration that stops once every
       off-diagonal is under 1e-5 x the diagonal trace, so the eigenvalues it
       hands back carry that much of the tensor's residue -- the tolerance
       cannot be tighter than that. */
    double trace = 0.0;
    for(int c = 0; c < 3; c++) { trace += std::fabs(wantS[c][c]); }
    const double iTol = 1e-5 * std::max(1.0, trace);
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
            snprintf(buf, sizeof(buf), "%s: principal-basis inertia [col %d row %d]",
                     msg, c, r);
            CHECK_NEAR(gotS[c][r], wantS[c][r], iTol, buf);
        }
    }
    /* the same tensor in WORLD axes must be getInertia()'s -- tying the
       compound to the reference the rest of this file already pins */
    const glm::dmat3 gotW = s.sRot * gotS * glm::transpose(s.sRot);
    checkRot(gotW, s.v->getInertia(), iTol, (std::string(msg) +
             ": compound inertia == getInertia() (world axes)").c_str());

    /* 2) an arbitrary world pose for the ship body (not the one the parts
       happen to have -- see the block comment) */
    const glm::dmat3 sprime = glm::mat3_cast(
        glm::angleAxis(-0.9, glm::normalize(glm::dvec3(0.4, -1.0, 2.0))));
    const glm::dvec3 tprime(1234.5, -678.9, 42.0);
    s.v->shipBody->setWorldTransform(
        Vehicle::toBt(tprime + sprime * pOrigin, sprime * pBasis));
    for(size_t i = 0; i < s.pos.size(); i++) {
        const Part *p = s.v->parts[i];
        glm::dvec3 gp; glm::dmat3 gr;
        s.v->compoundPartPose(p, gp, gr);
        const glm::dvec3 wp = tprime + sprime * p->localPos;
        const double ptol = 1e-9 * std::max(1.0, glm::length(wp));
        for(int c = 0; c < 3; c++) {
            snprintf(buf, sizeof(buf),
                     "%s: part %zu world pos at an arbitrary ship pose [%d]",
                     msg, i, c);
            CHECK_NEAR(gp[c], wp[c], ptol, buf);
        }
        snprintf(buf, sizeof(buf),
                 "%s: part %zu world rot at an arbitrary ship pose", msg, i);
        checkRot(gr, sprime * p->localRot, 1e-12, buf);

        /* the collision geometry sits where the derived pose says the part
           is: child i of the re-based compound, taken into world by the body
           transform, IS part i's world pose (see item 2 above) */
        glm::dvec3 cw; glm::dmat3 cwr;
        Vehicle::fromBt(s.v->shipBody->getCenterOfMassTransform()
                        * s.v->compound->getChildTransform((int)i), cw, cwr);
        for(int c = 0; c < 3; c++) {
            snprintf(buf, sizeof(buf),
                     "%s: part %zu compound child in world [%d]", msg, i, c);
            CHECK_NEAR(cw[c], wp[c], ptol, buf);
        }
        snprintf(buf, sizeof(buf),
                 "%s: part %zu compound child rotation", msg, i);
        checkRot(cwr, sprime * p->localRot, 1e-12, buf);
    }

    /* 3) syncShipBody mirrors the live parts: the derived poses must come
       back as the world poses this test set by hand */
    s.v->syncShipBody();
    for(size_t i = 0; i < s.pos.size(); i++) {
        glm::dvec3 gp; glm::dmat3 gr;
        s.v->compoundPartPose(s.v->parts[i], gp, gr);
        const double ptol = 1e-9 * std::max(1.0, glm::length(s.pos[i]));
        for(int c = 0; c < 3; c++) {
            snprintf(buf, sizeof(buf),
                     "%s: part %zu world pos after syncShipBody [%d]", msg, i, c);
            CHECK_NEAR(gp[c], s.pos[i][c], ptol, buf);
        }
        snprintf(buf, sizeof(buf),
                 "%s: part %zu world rot after syncShipBody", msg, i);
        checkRot(gr, s.rot[i], 1e-12, buf);
    }

    /* 4) the accessor the rest of the game reads: partWorldPose is the live
       per-part body today and becomes compoundPartPose when the ship does,
       so pin that it currently agrees with the pose this test set by hand --
       and that the two routes give the SAME answer, which is the property
       the switch-over relies on. */
    for(size_t i = 0; i < s.pos.size(); i++) {
        const Part *p = s.v->parts[i];
        glm::dvec3 lp, cp; glm::dmat3 lr, cr;
        s.v->partWorldPose(p, lp, lr);
        s.v->compoundPartPose(p, cp, cr);
        const double ptol = 1e-9 * std::max(1.0, glm::length(s.pos[i]));
        for(int c = 0; c < 3; c++) {
            snprintf(buf, sizeof(buf), "%s: part %zu partWorldPose pos [%d]", msg, i, c);
            CHECK_NEAR(lp[c], s.pos[i][c], ptol, buf);
            snprintf(buf, sizeof(buf),
                     "%s: part %zu partWorldPose == compoundPartPose pos [%d]", msg, i, c);
            CHECK_NEAR(lp[c], cp[c], ptol, buf);
        }
        snprintf(buf, sizeof(buf), "%s: part %zu partWorldPose rot", msg, i);
        checkRot(lr, s.rot[i], 1e-12, buf);
        snprintf(buf, sizeof(buf),
                 "%s: part %zu partWorldPose == compoundPartPose rot", msg, i);
        checkRot(lr, cr, 1e-12, buf);

        /* the rest of the accessor set, against the same reference */
        const glm::dvec3 v = s.v->partPos(p);
        for(int c = 0; c < 3; c++) {
            snprintf(buf, sizeof(buf), "%s: part %zu partPos [%d]", msg, i, c);
            CHECK_NEAR(v[c], s.pos[i][c], ptol, buf);
        }
        snprintf(buf, sizeof(buf), "%s: part %zu partRot", msg, i);
        checkRot(s.v->partRot(p), s.rot[i], 1e-12, buf);
        for(int n = 0; n < 3; n++) {
            const glm::dvec3 ax = s.v->partAxis(p, n);
            for(int c = 0; c < 3; c++) {
                snprintf(buf, sizeof(buf), "%s: part %zu partAxis(%d) [%d]", msg, i, n, c);
                CHECK_NEAR(ax[c], s.rot[i][n][c], 1e-12, buf);
            }
        }
    }
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
    checkCompound(s, "single part: compound");
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
    checkCompound(s, "symmetric pair: compound");
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
    checkCompound(s, "unequal masses: compound");
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
    checkCompound(s, "rotated part: compound");
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
    checkCompound(s, "general assembly: compound");

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

/* --- refreshCompound: the COM-drift threshold ----------------------------

   A burn moves the centre of mass, and the compound's children are re-based
   through `principal`, so a stale COM displaces every hull that picking and
   collision read. Rebuilding two compound shapes per tank draw per tick would
   be waste, so refreshCompound() rebuilds only once the mass distribution has
   shifted the COM past kComRebuildTol. Both halves are pinned: under the
   tolerance the compound is left exactly as it was, over it the rebuild lands
   on the NEW centre of mass.

   What is perturbed here is a part's AUTHORED POSE rather than its mass,
   because that keeps the inertia invariant intact -- checkCompoundInvariants
   compares Bullet's child inertias, recomputed from the current masses, with
   the bodies' stored diagonals, so changing a mass without also calling
   SetMass would trip that assert. The quantity under test, compoundCom()
   against principal.getOrigin(), is identical either way.

   Layout: 300 kg at z=+6, 9000 kg at z=0, 1500 kg at z=-4, total 10800 kg,
   so moving the capsule by dz shifts the COM by dz/36.                    */
static void test_refresh() {
    Ship s; s.v = new TestVehicle;
    addBox(s, "capsule",  300.0, 1.0, 1.0, 1.0,
           glm::dvec3(0.0, 0.0,  6.0), glm::dmat3(1.0));
    addBox(s, "tank",    9000.0, 2.0, 2.0, 2.5,
           glm::dvec3(0.0, 0.0,  0.0), glm::dmat3(1.0));
    addBox(s, "engine",  1500.0, 1.5, 1.5, 1.0,
           glm::dvec3(0.0, 0.0, -4.0), glm::dmat3(1.0));
    s.v->rebuildCompound();

    glm::dvec3 origin; glm::dmat3 basis;
    Vehicle::fromBt(s.v->principal, origin, basis);
    CHECK_NEAR(origin.z, s.v->compoundCom().z, 1e-12,
               "refresh: the compound starts on the authored COM");

    /* under the tolerance: dz = 0.05 m moves the COM by 0.05/36 = 1.4 mm */
    Part *cap = s.v->parts[0];
    cap->localPos += glm::dvec3(0.0, 0.0, 0.05);
    const double driftSmall = glm::length(s.v->compoundCom() - origin);
    CHECK_TRUE(driftSmall < Vehicle::kComRebuildTol,
               "refresh: the small nudge really is under the tolerance "
               "(otherwise this case proves nothing)");
    s.v->refreshCompound();
    glm::dvec3 after; glm::dmat3 b2;
    Vehicle::fromBt(s.v->principal, after, b2);
    g_checks++;
    if(after != origin) {
        g_failures++;
        printf("FAIL: refresh: under the tolerance the compound must be left "
               "untouched (COM z %.12g -> %.12g)\n", origin.z, after.z);
    }

    /* over it: dz = 1.0 m moves the COM by 1/36 = 28 mm */
    cap->localPos += glm::dvec3(0.0, 0.0, 0.95);
    const double driftBig = glm::length(s.v->compoundCom() - origin);
    CHECK_TRUE(driftBig > Vehicle::kComRebuildTol,
               "refresh: the big nudge really is over the tolerance");
    s.v->refreshCompound();
    glm::dvec3 rebuilt; glm::dmat3 b3;
    Vehicle::fromBt(s.v->principal, rebuilt, b3);
    const glm::dvec3 want = s.v->compoundCom();
    CHECK_NEAR(rebuilt.z, want.z, 1e-12,
               "refresh: over the tolerance the rebuild lands on the new COM");
    CHECK_TRUE(glm::length(rebuilt - origin) > Vehicle::kComRebuildTol,
               "refresh: the rebuild really did move the principal origin");

    /* and the rebuilt compound still reproduces its assembly -- the invariant
       assert ran inside rebuildCompound, so reaching here is the pass */
    checkCom(s, "refresh: COM after the rebuild");
    destroyShip(s);
}

int main() {
    printf("== ship mass properties + the compound body (src/vehicle.h) ==\n");
    test_single();
    test_symmetric_pair();
    test_unequal_masses();
    test_rotated_part();
    test_general_assembly();
    test_refresh();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_inertia: all checks passed\n");
    return 0;
}
