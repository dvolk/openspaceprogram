// pick.cpp -- the picking math (see pick.h).
//
// The ray test uses btCollisionWorld::rayTestSingle (Bullet's own convex
// cast / triangle raycast) rather than a hand-rolled hull test: it hits
// exactly what collides (same margin, same shape) and already supports
// the triangle meshes terrain/pads use. rayTestSingle takes the object +
// shape + world transform directly, so it needs no collision world --
// which matters because every ship's bodies live in that SHIP's own
// frame, and there is one global Bullet world with no frame of its own.

#include "pick.h"

#include "body.h"      // Body (+ the Bullet types, double precision)
#include "camera.h"    // Camera
#include "frame.h"     // Frame (the per-ship frame transform)
#include "game.h"      // Game (the fleet, the camera)
#include "ships.h"     // Ships
#include "vehicle.h"   // Vehicle (parts)

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

PickRay pickRay(const Camera &cam, int W, int H, int px, int py) {
    // NDC of the pixel (top-left origin -> y flipped).
    const double nx = 2.0 * (double)px / (double)W - 1.0;
    const double ny = 1.0 - 2.0 * (double)py / (double)H;

    // View-space ray direction for that pixel, straight from the
    // projection. Reading off the matrix (x/y/w rows: fx*X, fy*Y, C*Z) a
    // point at infinity in direction d projects to NDC
    //   (fx*dx, fy*dy) / (C*dz)   =>   dx/dz = nx*C/fx,  dy/dz = ny*C/fy.
    // The ray must point FORWARD (the camera looks down view -Z, so
    // dz = -1), which flips the signs:
    //   dx = -nx*C/fx,   dy = -ny*C/fy.
    // Doing it this way (instead of unprojecting the near + far clip
    // points) also works for the game's zFar = infinity projection, where
    // the far clip point has w = 0 and that other approach NaNs.
    const double fx = cam.projection[0][0];
    const double fy = cam.projection[1][1];
    const double C  = cam.projection[2][3];   // the w row's Z coefficient
    const glm::dvec3 dirView = glm::dvec3(-nx * C / fx, -ny * C / fy, -1.0);

    // buildView() (camera.cpp) builds the view with cam = pos - renderOrigin,
    // and the Draw sites shift geometry by -renderOrigin (body.h,
    // terrain.cpp). The two shifts cancel, so a render-frame point p maps
    // as p_view = R * (p - pos) -- renderOrigin only buys float precision
    // in the MVP cast -- and the inverse (this unprojection) is
    // p_render = R^T * p_view + pos.
    const glm::dmat3 R(cam.view);
    return PickRay{
        cam.pos,
        glm::normalize(glm::transpose(R) * dirView)
    };
}

bool pickBody(const PickRay &ray, const Body *body, PickBodyHit &hit) {
    // One long segment along the ray, in the body's frame (double
    // precision, so a scene-sized length is exact enough).
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
    btCollisionWorld::rayTestSingle(rayFrom, rayTo,
                                    body->btBody,
                                    body->btBody->getCollisionShape(),
                                    body->btBody->getWorldTransform(),
                                    cb);
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
        // The ship's part frame -> render frame (the same transform
        // Vehicle::Draw uses); the ray must live in the ship's frame,
        // where its bodies' transforms live.
        const glm::dmat4 invXf = glm::inverse(s->renderXform(renderFrame));
        // (glm has no mat4 * vec3; the w=1 point form does the job)
        const glm::dvec4 po = invXf * glm::dvec4(ray.origin, 1.0);
        PickRay sray{ glm::dvec3(po.x, po.y, po.z),
                      glm::normalize(glm::dmat3(invXf) * ray.dir) };

        for(size_t i = 0; i < s->parts.size(); i++) {
            PickBodyHit h;
            if(!pickBody(sray, s->parts[i], h)) { continue; }
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
