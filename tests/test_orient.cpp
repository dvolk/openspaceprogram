// Pin down the GLM <-> Bullet 3x3 orientation convention and validate the
// quaternion-based conversion we plan to use for setPosRot / GetOrient.
//
// Background: the old code did element-wise conversion:
//   write: btMatrix3x3::setValue(rot[0][0], rot[0][1], ...)   // GLM [col][row]
//   read : glm::make_mat3x3(&basis[0][0])                      // raw memcpy
// Bullet stores matrices ROW-major (m_el[i] = row i); GLM stores them
// COLUMN-major (value[i] = column i). So both paths silently transpose the
// orientation. This test (a) demonstrates that, and (b) validates a
// quaternion round-trip that is immune to the row/col-major ambiguity.
//
// Build & run (from repo root):
//   B=middleware/bullet3/build/src
//   g++ -O2 -std=c++11 -DBT_USE_DOUBLE_PRECISION \
//       -I./src -I./middleware/glm/ -I./middleware/bullet3/ -I./middleware/bullet3/bullet \
//       tests/test_orient.cpp -o test_orient \
//       $B/BulletDynamics/libBulletDynamics.a $B/BulletCollision/libBulletCollision.a \
//       $B/Bullet3Collision/libBullet3Collision.a $B/BulletInverseDynamics/libBulletInverseDynamics.a \
//       $B/Bullet3Geometry/libBullet3Geometry.a $B/LinearMath/libLinearMath.a $B/Bullet3Common/libBullet3Common.a
//   ./test_orient

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include <cstdio>
#include <cmath>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

// ---- OLD (buggy) element-wise conversion from src/physics.cpp ----
static glm::dmat3 old_getOrient(const btMatrix3x3& b) {
    return glm::make_mat3x3(&b[0][0]);
}
static void old_setBasis(btMatrix3x3& r, const glm::dmat3& rot) {
    r.setValue(rot[0][0], rot[0][1], rot[0][2],
               rot[1][0], rot[1][1], rot[1][2],
               rot[2][0], rot[2][1], rot[2][2]);
}

// ---- NEW (proposed) quaternion conversion ----
static glm::dmat3 new_getOrient(const btMatrix3x3& b) {
    btQuaternion q;
    b.getRotation(q);
    // GLM 4-scalar quat constructor is (w, x, y, z) -- w first!
    glm::dquat gq(q.w(), q.x(), q.y(), q.z());
    return glm::mat3_cast(gq);
}
static void new_setBasis(btMatrix3x3& r, const glm::dmat3& rot) {
    glm::dquat gq = glm::quat_cast(rot);
    r.setRotation(btQuaternion(gq.x, gq.y, gq.z, gq.w));
}

static double maxdiff_mat(const glm::dmat3& a, const glm::dmat3& b) {
    double d = 0;
    for (int c = 0; c < 3; c++) for (int r = 0; r < 3; r++)
        d = std::max(d, std::fabs(a[c][r] - b[c][r]));
    return d;
}

// Treat a GLM dmat3 as a math rotation matrix M (v' = M*v).
static void print_math(const char* label, const glm::dmat3& m) {
    printf("%s (math rows):\n", label);
    for (int r = 0; r < 3; r++)
        printf("  [% .4f % .4f % .4f]\n", m[0][r], m[1][r], m[2][r]);
}

int main() {
    int failures = 0;

    // Intended orientation: 30 deg about Z. math matrix:
    //   [cos -sin 0; sin cos 0; 0 0 1]
    const double a = 30.0 * M_PI / 180.0;
    glm::dmat3 intended = glm::dmat3(glm::rotate(a, glm::dvec3(0, 0, 1)));
    print_math("Intended 30deg-Z", intended);

    // A probe vector to check the PHYSICAL rotation (not just the matrix text).
    glm::dvec3 probe(1, 2, 3);
    glm::dvec3 expected_probe = intended * probe;   // intended rotation applied

    printf("\n=== OLD element-wise path (the bug) ===\n");
    {
        // write via old_setBasis, then read back via old_getOrient
        btMatrix3x3 stored; stored.setIdentity();
        old_setBasis(stored, intended);
        glm::dmat3 readback = old_getOrient(stored);
        double d = maxdiff_mat(readback, intended);
        printf("old write->read round-trip maxdiff vs intended: %.4f  %s\n",
               d, d < 1e-9 ? "OK" : "NOT round-trip (transposed/buggy) -- expected");

        // physical check: apply stored matrix to probe and compare to intended
        btVector3 bp(probe.x, probe.y, probe.z);
        btVector3 br = stored * bp;
        double pd = std::max(std::max(std::fabs(br.x() - expected_probe.x),
                                      std::fabs(br.y() - expected_probe.y)),
                             std::fabs(br.z() - expected_probe.z));
        printf("old path physical rotation error vs intended: %.4f  %s\n",
               pd, pd < 1e-9 ? "OK" : "WRONG physical rotation -- expected");
    }

    printf("\n=== NEW quaternion path (the fix) ===\n");
    {
        // write via new_setBasis, then read back via new_getOrient
        btMatrix3x3 stored; stored.setIdentity();
        new_setBasis(stored, intended);
        glm::dmat3 readback = new_getOrient(stored);
        double d = maxdiff_mat(readback, intended);
        printf("new write->read round-trip maxdiff vs intended: %.4f  %s\n",
               d, d < 1e-9 ? "OK" : "BUG: not round-trip");
        if (d >= 1e-9) failures++;

        // physical check: apply stored matrix to probe and compare to intended
        btVector3 bp(probe.x, probe.y, probe.z);
        btVector3 br = stored * bp;
        double pd = std::max(std::max(std::fabs(br.x() - expected_probe.x),
                                      std::fabs(br.y() - expected_probe.y)),
                             std::fabs(br.z() - expected_probe.z));
        printf("new path physical rotation error vs intended: %.4f  %s\n",
               pd, pd < 1e-9 ? "OK" : "BUG: wrong physical rotation");
        if (pd >= 1e-9) failures++;
    }

    printf("\n=== Summary (only the NEW path gates the result) ===\n");
    if (failures == 0) {
        printf("PASS: quaternion path round-trips AND matches the intended physical rotation.\n");
    } else {
        printf("FAIL: %d check(s) on the NEW path failed.\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
