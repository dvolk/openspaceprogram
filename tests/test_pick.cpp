// test_pick.cpp -- the picking math (src/pick.cpp), headless:
//   pickRay   pixel -> render-frame ray: a point on the ray must
//             round-trip back through the camera's own view + projection
//             to the pixel it was picked from, and the center pixel must
//             point at the focus.
//   pickBody  a ray vs known convex hulls: hit point / normal / distance,
//             a miss off to the side, and translated + rotated bodies
//             (the world-transform handling that rayTestSingle does for
//             us).
//
// Links pick.cpp + camera.cpp + the real Bullet libs (the convex cast),
// same as test_thrust. No GL needed (the model pointers stay null).
// pickShipPart / pickAt need a Game; those are covered by the e2e case.

#include <cmath>
#include <cstdio>

#include "body.h"    // Body (+ Bullet types; double precision)
#include "camera.h"  // Camera
#include "pick.h"

static int failures = 0;
#define CHECK(cond)                                                    \
    do {                                                               \
        if(!(cond)) {                                                  \
            printf("FAIL: %s (line %d)\n", #cond, __LINE__);           \
            failures++;                                                \
        }                                                              \
    } while(0)

static bool near(double a, double b, double eps) {
    return std::fabs(a - b) <= eps;
}

// A hull body at xform. The test owns `shape` (the Body deletes its model
// + btBody but not the shape -- physics.cpp manages hulls the same way);
// delete both at the end of each case.
static Body *makeBody(btCollisionShape *shape, const glm::dmat4 &xf) {
    Body *b = new Body;
    b->model = nullptr;
    b->mass = 1.0;
    // (this Bullet's btQuaternion has no matrix ctor; the project's own
    // idiom is basis -> getRotation -> btTransform(q, origin))
    btMatrix3x3 m3(xf[0][0], xf[1][0], xf[2][0],
                   xf[0][1], xf[1][1], xf[2][1],
                   xf[0][2], xf[1][2], xf[2][2]);
    btQuaternion q;
    m3.getRotation(q);
    b->btBody = new btRigidBody(1.0, 0, shape, btVector3(1.0, 1.0, 1.0));
    b->btBody->setWorldTransform(btTransform(
        q, btVector3(xf[3][0], xf[3][1], xf[3][2])));
    return b;
}

// A convex hull box with the given half-extents (zero margin).
static btConvexHullShape *boxHull(double hx, double hy, double hz) {
    btConvexHullShape *h = new btConvexHullShape();
    for(int sx = -1; sx <= 1; sx += 2)
        for(int sy = -1; sy <= 1; sy += 2)
            for(int sz = -1; sz <= 1; sz += 2)
                h->addPoint(btVector3(sx * hx, sy * hy, sz * hz));
    h->setMargin(0.0f);
    return h;
}

static void testPickRay() {
    // Identity view looking down -Z from (0,0,10), 16:9, fov 0.5 rad.
    // renderOrigin is a big planet-frame offset (the game sets it to the
    // active ship's COM): it must cancel between buildView (cam = pos -
    // renderOrigin) and the Draw sites' translate(-renderOrigin), leaving
    // p_view = R * (p - pos). A wrong unprojection that keeps the
    // renderOrigin in would land the ray ~1e7 m off.
    Camera cam(glm::dvec3(0.0), 0.5f, 16.0f / 9.0f, 1.0f, 100.0f);
    cam.pos = glm::dvec3(0.0, 0.0, 10.0);
    cam.forward = glm::dvec3(0.0, 0.0, -1.0);
    cam.up = glm::dvec3(0.0, 1.0, 0.0);
    cam.renderOrigin = glm::dvec3(1.0e7, 2.0e6, -3.0e5);
    // The view buildView() would build from this state (R = I,
    // translation = -(pos - renderOrigin)) -- the round-trip below must
    // invert that exact pipeline.
    cam.view = glm::translate(cam.renderOrigin - cam.pos);
    const int W = 1600, H = 900;

    // The center pixel: origin at the camera, direction at the focus.
    PickRay r = pickRay(cam, W, H, W / 2, H / 2);
    CHECK(near(r.origin.x, 0.0, 1e-9) && near(r.origin.y, 0.0, 1e-9)
          && near(r.origin.z, 10.0, 1e-9));
    CHECK(near(r.dir.x, 0.0, 1e-9) && near(r.dir.y, 0.0, 1e-9)
          && near(r.dir.z, -1.0, 1e-9));

    // Round-trip: a point far along the ray must project back to the
    // pixel it was picked from (1 px tolerance; the projection is float).
    auto roundTrip = [&](int px, int py) {
        PickRay rr = pickRay(cam, W, H, px, py);
        const glm::dvec3 p = rr.origin + rr.dir * 100.0;  // render frame
        const glm::dvec4 v = cam.view * glm::dvec4(p - cam.renderOrigin, 1.0);
        const glm::dvec4 c = glm::dmat4(cam.projection) * v;
        const double ndcX = (c.x / c.w) * 0.5 + 0.5;
        const double ndcY = (c.y / c.w) * 0.5 + 0.5;
        CHECK(near(ndcX * W, px, 1.0));
        CHECK(near((1.0 - ndcY) * H, py, 1.0));
    };
    roundTrip(W / 2, H / 2);
    roundTrip(1200, 450);
    roundTrip(100, 800);
    roundTrip(W - 1, H - 1);

    // Sanity on the side directions: right of center -> +x, below center
    // -> -y, always still pointing forward (-z).
    PickRay r2 = pickRay(cam, W, H, 1200, H / 2);
    CHECK(r2.dir.x > 0.0 && r2.dir.z < 0.0);
    PickRay r3 = pickRay(cam, W, H, W / 2, 800);
    CHECK(r3.dir.y < 0.0 && r3.dir.z < 0.0);
}

static void testPickBody() {
    // Unit cube at the origin.
    {
        Body *b = makeBody(boxHull(0.5, 0.5, 0.5), glm::dmat4(1.0));
        PickBodyHit h;
        CHECK(pickBody({glm::dvec3(0, 0, 5), glm::dvec3(0, 0, -1)}, b, h));
        CHECK(near(h.point.x, 0.0, 0.05) && near(h.point.y, 0.0, 0.05)
              && near(h.point.z, 0.5, 0.05));
        CHECK(near(h.dist, 4.5, 0.05));
        CHECK(near(std::fabs(h.normal.z), 1.0, 0.05));

        // A parallel ray off to the side misses.
        CHECK(!pickBody({glm::dvec3(10, 0, 5), glm::dvec3(0, 0, -1)}, b, h));
        delete b;
    }
    // The same cube moved to (10,0,0): hit point and distance follow.
    {
        const glm::dmat4 xf = glm::translate(glm::dvec3(10.0, 0.0, 0.0));
        Body *b = makeBody(boxHull(0.5, 0.5, 0.5), xf);
        PickBodyHit h;
        CHECK(pickBody({glm::dvec3(10, 0, 5), glm::dvec3(0, 0, -1)}, b, h));
        CHECK(near(h.point.x, 10.0, 0.05) && near(h.point.z, 0.5, 0.05));
        CHECK(near(h.dist, 4.5, 0.05));
        delete b;
    }
    // A z-rod (half-length 2) rotated 90 deg about Y now lies along X:
    // the +Z end sits at +X, so a ray from +X hits it at x = +2.
    {
        const double halfPi = 3.14159265358979323846 / 2.0;
        const glm::dmat4 xf = glm::rotate(glm::dmat4(1.0), halfPi,
                                          glm::dvec3(0.0, 1.0, 0.0));
        Body *b = makeBody(boxHull(0.5, 0.5, 2.0), xf);
        PickBodyHit h;
        CHECK(pickBody({glm::dvec3(5, 0, 0), glm::dvec3(-1, 0, 0)}, b, h));
        CHECK(near(h.point.x, 2.0, 0.05) && near(h.point.y, 0.0, 0.05)
              && near(h.point.z, 0.0, 0.05));
        CHECK(near(h.dist, 3.0, 0.05));
        delete b;
    }
}

int main() {
    testPickRay();
    testPickBody();
    if(failures) {
        printf("%d FAILURES\n", failures);
        return 1;
    }
    printf("test_pick: all passed\n");
    return 0;
}
