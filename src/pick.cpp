// pick.cpp -- the picking math (see pick.h). Uses btCollisionWorld::rayTestSingle
// (Bullet's own cast) so a pick hits exactly what collides; it takes object +
// shape + transform directly, no collision world needed -- each ship's bodies
// live in that ship's own frame.

#include "pick.h"

#include "body.h"      // Body (pointer members only, no Bullet include)
#include "buildings.h" // buildingOrient (a pad's local axes)
#include "camera.h"    // Camera
#include "frame.h"     // Frame (the per-ship frame transform)
#include "game.h"      // Game (the fleet, the camera)
#include "ships.h"     // Ships
#include "terrain.h"   // TerrainBody (its ships) + StaticBuilding
#include "vehicle.h"   // Vehicle (parts)

#include "btcommon.h"  // complete Bullet types (double precision)
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <numbers>

PickRay pickRay(const Camera &cam, int W, int H, int px, int py) {
    // NDC of the pixel (top-left origin -> y flipped).
    const double nx = 2.0 * (double)px / (double)W - 1.0;
    const double ny = 1.0 - 2.0 * (double)py / (double)H;

    // View-space ray dir from the projection: works for the zFar=infinity
    // projection, where unprojecting far-clip NaNs.
    const double fx = cam.projection[0][0];
    const double fy = cam.projection[1][1];
    const double C  = cam.projection[2][3];   // the w row's Z coefficient
    const glm::dvec3 dirView = glm::dvec3(-nx * C / fx, -ny * C / fy, -1.0);

    // The -renderOrigin shifts in the view and Draw sites cancel, so
    // p_render = R^T * p_view + pos.
    const glm::dmat3 R(cam.view);
    return PickRay{
        cam.pos,
        glm::normalize(glm::transpose(R) * dirView)
    };
}

/* Physics-free seam for the VAB build tree too (no rigid body). */
bool castRay(const PickRay &ray, btCollisionObject *obj,
             const btCollisionShape *shape, const btTransform &xform,
             PickBodyHit &hit) {
    // One long segment along the ray (exact at scene scale in double).
    const double L = 1e7;   // m
    btVector3 from(ray.origin.x, ray.origin.y, ray.origin.z);
    btVector3 to((ray.origin + ray.dir * L).x,
                 (ray.origin + ray.dir * L).y,
                 (ray.origin + ray.dir * L).z);

    btTransform rayFrom, rayTo;
    rayFrom.setIdentity();   // btTransform() {} leaves BOTH members uninitialised
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

/* The child index IS the part index: rebuildCompound fills compoundParts in
   parts order. */
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

    for(auto *s : g.vehicles) {
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
    if(bestShip == nullptr) { return false; }

    ship = bestShip;
    part = bestPart;
    hit = bestHit;
    return true;
}

/* One deck holds one ship: two grounded ships wake and torque each other, so an
   occupied pad is a collision, not a cosmetic stack (#231). Rays go up the pad's
   axis on a grid across the hull's deck box; the 0 samples are load-bearing (the
   VAB parks a ship dead centre). Not exhaustive by design -- parts between sample
   lines and ships clear of or hovering over the deck are missed. Only ship parts
   are cast, so the rays' trip through pad and ground is free. */
Vehicle *findPadOccupant(const StaticBuilding *site, const Fleet &fleet) {
    if(site == nullptr || site->parent == nullptr) { return nullptr; }
    TerrainBody *hb = site->parent;
    const glm::dmat3 deck = buildingOrient(
        site->dir, site->heading_deg * std::numbers::pi / 180.0);
    const glm::dvec3 org = site->origin();   // pad centre, in the rotating frame
    static const double lat[] = { -9.0, 0.0, 9.0 };
    static const double along[] = { -30.0, -15.0, 0.0, 15.0, 30.0 };
    const double sink = 5.0;    // m below the deck rays start, so a ship settled
                                // into the pad's contact slop still registers
    const double reach = 120.0;  // m above the deck a hit counts: taller than
                                 // anything we build, far below orbit

    Frame *rot = hb->frame->getRotFrame();
    for(Vehicle *s : fleet) {
        btCompoundShape *cs = s->compoundShape();
        if(cs == nullptr) { continue; }
        // The same-SoI filter a body-side list used to give for free.
        if(s->m_parent != hb) { continue; }
        /* A seated kerbal is never in the fleet -- it is a seat on its capsule's
           Part (part.h), with no Body in the world. A walking kerbal can be, and
           is a genuine obstruction. */
        // Cheap reject before the frame hop: only one ship is near a given pad.
        const glm::dvec3 comRot = s->frame->GetOrientRelTo(rot)
                                * s->get_center_of_mass()
                                + s->frame->GetPositionRelTo(rot);
        if(glm::length(comRot - org) > reach + sink + 60.0) { continue; }

        /* The pad's pose is in the body's ROTATING frame; the ship's Bullet
           transforms live in the ship's OWN frame. No collision world needed, so
           a ship on rails still answers. */
        const glm::dmat3 O = rot->GetOrientRelTo(s->frame);
        const glm::dvec3 P = rot->GetPositionRelTo(s->frame);
        const btTransform hull = s->hull->btBody->getCenterOfMassTransform();
        const glm::dvec3 up = O * site->dir;
        for(size_t i = 0; i < sizeof(lat) / sizeof(lat[0]); i++) {
            for(size_t k = 0; k < sizeof(along) / sizeof(along[0]); k++) {
                PickRay ray{ O * (org + deck * glm::dvec3(lat[i], along[k], -sink)) + P,
                             up };
                PickBodyHit hit;
                for(int c = 0; c < cs->getNumChildShapes(); c++) {
                    if(!castRay(ray, s->hull->btBody, cs->getChildShape(c),
                                hull * cs->getChildTransform(c), hit)) {
                        continue;
                    }
                    if(hit.dist <= sink + reach) { return s; }
                }
            }
        }
    }
    return nullptr;
}
