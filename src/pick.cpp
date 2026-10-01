// pick.cpp -- the picking math (see pick.h). Uses btCollisionWorld::rayTestSingle
// (Bullet's own cast) so a pick hits exactly what collides. rayTestSingle
// takes object + shape + transform directly (no collision world needed --
// each ship's bodies live in that ship's own frame).

#include "pick.h"

#include "body.h"      // Body (pointer members only, no Bullet include)
#include "camera.h"    // Camera
#include "frame.h"     // Frame (the per-ship frame transform)
#include "game.h"      // Game (the fleet, the camera)
#include "ships.h"     // Ships
#include "vehicle.h"   // Vehicle (parts)

#include "btcommon.h"  // complete Bullet types (double precision)
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

PickRay pickRay(const Camera &cam, int W, int H, int px, int py) {
    // NDC of the pixel (top-left origin -> y flipped).
    const double nx = 2.0 * (double)px / (double)W - 1.0;
    const double ny = 1.0 - 2.0 * (double)py / (double)H;

    // View-space ray direction from the projection. This works for the
    // zFar=infinity projection where unprojecting far-clip NaNs.
    const double fx = cam.projection[0][0];
    const double fy = cam.projection[1][1];
    const double C  = cam.projection[2][3];   // the w row's Z coefficient
    const glm::dvec3 dirView = glm::dvec3(-nx * C / fx, -ny * C / fy, -1.0);

    // The -renderOrigin shifts in the view and Draw sites cancel, so
    // p_render = R^T * p_view + pos. Caveat: in Orbit mode the pick ray
    // origin diverges from the rendered eye by <= ULP(pos)/2 (~6 cm at 1e15,
    // only at interstellar ranges).
    const glm::dmat3 R(cam.view);
    return PickRay{
        cam.pos,
        glm::normalize(glm::transpose(R) * dirView)
    };
}

/* One ray against one collision shape at one transform. Also the
   physics-free seam for the VAB build tree (no rigid body). */
bool castRay(const PickRay &ray, btCollisionObject *obj,
             const btCollisionShape *shape, const btTransform &xform,
             PickBodyHit &hit) {
    // One long segment along the ray (double precision, scene-sized length is exact).
    const double L = 1e7;   // m
    btVector3 from(ray.origin.x, ray.origin.y, ray.origin.z);
    btVector3 to((ray.origin + ray.dir * L).x,
                 (ray.origin + ray.dir * L).y,
                 (ray.origin + ray.dir * L).z);

    btTransform rayFrom, rayTo;
    rayFrom.setIdentity();
    rayTo.setIdentity();
    rayFrom.setOrigin(from);
    rayTo.setOrigin(to);

    btCollisionWorld::ClosestRayResultCallback cb(from, to);
    btCollisionWorld::rayTestSingle(rayFrom, rayTo, obj, shape, xform, cb);
    if(!cb.hasHit()) { return false; }

    hit.point  = glm::dvec3(cb.m_hitPointWorld.getX(),
                            cb.m_hitPointWorld.getY(),
                            cb.m_hitPointWorld.getZ());
    hit.normal = glm::dvec3(cb.m_hitNormalWorld.getX(),
                            cb.m_hitNormalWorld.getY(),
                            cb.m_hitNormalWorld.getZ());
    hit.dist   = cb.m_closestHitFraction * L;
    return true;
}

bool pickBody(const PickRay &ray, const Body *body, PickBodyHit &hit) {
    return castRay(ray, body->btBody, body->shape,
                   body->btBody->getWorldTransform(), hit);
}

/* One child of a ship's compound. The child index IS the part index
   (rebuildCompound fills compoundParts in parts order). */
static bool pickShipChild(const PickRay &ray, Vehicle *ship, size_t child,
                          PickBodyHit &hit) {
    btCompoundShape *cs = ship->compoundShape();
    return castRay(ray, ship->hull->btBody,
                   cs->getChildShape((int)child),
                   ship->hull->btBody->getCenterOfMassTransform()
                       * cs->getChildTransform((int)child),
                   hit);
}

bool pickShipPart(Game &g, int px, int py,
                  Vehicle *&ship, size_t &part, PickBodyHit &hit) {
    if(g.camera == nullptr || g.ship == nullptr) { return false; }

    // The render frame the view is built in (the active ship's frame).
    Frame *renderFrame = g.ship->frame;
    PickRay ray = pickRay(*g.camera,
                          g.camera->viewport_w, g.camera->viewport_h,
                          px, py);

    double bestDist = 1e300;
    Vehicle *bestShip = nullptr;
    size_t bestPart = 0;
    PickBodyHit bestHit;

    for(auto *b : g.sys.bodies) {
    for(auto *s : b->ships) {
        if(s->compoundShape() == nullptr) { continue; }
        // Ray must live in the ship's frame, where its bodies' transforms live.
        const glm::dmat4 invXf = glm::inverse(s->renderXform(renderFrame));
        const glm::dvec4 po = invXf * glm::dvec4(ray.origin, 1.0);
        PickRay sray{ glm::dvec3(po.x, po.y, po.z),
                      glm::normalize(glm::dmat3(invXf) * ray.dir) };

        for(size_t i = 0; i < s->parts.size(); i++) {
            PickBodyHit h;
            if(!pickShipChild(sray, s, i, h)) { continue; }
            if(h.dist < bestDist) {
                bestDist = h.dist;
                bestShip = s;
                bestPart = i;
                bestHit = h;
            }
        }
    }
    }
    if(bestShip == nullptr) { return false; }

    ship = bestShip;
    part = bestPart;
    hit = bestHit;
    return true;
}
