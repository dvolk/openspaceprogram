// pick.cpp -- the picking math (see pick.h). Uses btCollisionWorld::rayTestSingle
// (Bullet's own cast) so a pick hits exactly what collides. rayTestSingle
// takes object + shape + transform directly (no collision world needed --
// each ship's bodies live in that ship's own frame).

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

/* Is a launch pad's deck clear? (#231: a second ship placed on an occupied
   pad used to just stand inside the first.)

   One deck holds one ship. space_port.obj's z=0 face is a hexagon out to
   x = 8.66, and two grounded ships sit inside each other's 10 m grounded
   proximity radius, so they wake on the first tick and torque each other --
   an occupied pad is a collision, not a cosmetic stack.

   The deck is the hull's box: local x +-13, y -41..15 at z = 0, the same box
   test_buildings measures when it checks that a pad's deck reaches the ground
   under its footprint. Rays go up the pad's own axis sampled on a grid across
   that box. The centre sample is load-bearing: the VAB places a ship dead on
   the pad and a part is only ~2.3 m across, so a grid that samples +-10 m and
   never 0 misses it outright (verified -- take the 0 back out and the launch
   goes through). The off-centre samples catch a ship that LANDED there. A
   part between two sample lines 15 m apart is missed: this answers "is
   something standing on this deck", which is all a launch decision needs, not
   "where exactly is it".

   Rays start 5 m UNDER the deck plane so that a ship which settles into the
   pad's contact slop is still above the start. They pass through the pad hull
   and the ground on the way up, which costs nothing because only ship parts
   are cast against at all -- and terrain cannot fake a hit even if it were,
   since the rays run radially outward and the heightmap is single-valued.

   Deliberately not exhaustive: a ship parked clear of the footprint, or
   hovering above it, is not what a launch decision needs to know. */
Vehicle *findPadOccupant(const StaticBuilding *site) {
    if(site == nullptr || site->parent == nullptr) { return nullptr; }
    TerrainBody *hb = site->parent;
    const glm::dmat3 deck = buildingOrient(
        site->dir, site->heading_deg * std::numbers::pi / 180.0);
    const glm::dvec3 org = site->origin();   // pad centre, in the rotating frame
    static const double lat[] = { -9.0, 0.0, 9.0 };
    static const double along[] = { -30.0, -15.0, 0.0, 15.0, 30.0 };
    const double sink = 5.0;    // m below the deck the rays start from
    const double reach = 120.0;  // m above the deck a hit still counts for
    // 120 m is taller than anything we build and far below the lowest thing
    // in orbit, which keeps a ship over the pad from reading as standing on it.

    Frame *rot = hb->frame->getRotFrame();
    for(Vehicle *s : hb->ships) {
        btCompoundShape *cs = s->compoundShape();
        if(cs == nullptr) { continue; }
        /* Aboard crew are owned by their ship (Vehicle::crew) and never in
           this list, so they cannot turn up here. A free EVA kerbal CAN, and
           a kerbal standing on the deck is a genuine obstruction -- the
           caller names whatever it finds. */
        // Cheap reject before the per-ship frame hop: a body can carry a
        // wide fleet and only one ship is ever near a given pad.
        const glm::dvec3 comRot = s->frame->GetOrientRelTo(rot)
                                * s->get_center_of_mass()
                                + s->frame->GetPositionRelTo(rot);
        if(glm::length(comRot - org) > reach + sink + 60.0) { continue; }

        /* The pad's pose is in the body's ROTATING frame; a ship's Bullet
           transforms live in the ship's OWN frame. rayTestSingle needs no
           collision world, so a ship on rails -- one the proximity system has
           parked out of it -- still answers. */
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
