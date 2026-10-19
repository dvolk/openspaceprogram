// pick.cpp -- the picking math (see pick.h). Uses btCollisionWorld::rayTestSingle
// (Bullet's own cast) so a pick hits exactly what collides; it takes object +
// shape + transform directly, no collision world needed -- each ship's bodies
// live in that ship's own frame.

#include "pick.h"

#include "body.h"      // Body (pointer members only, no Bullet include)
#include "buildings.h" // buildingOrient (a pad's local axes)
#include "camera.h"    // Camera
#include "frame.h"     // Frame (the per-ship frame transform)
#include "game.h"      // Game (the vehicles, the camera)
#include "part.h"      // Part (id, to name a part from a test)
#include "terrain.h"   // TerrainBody + StaticBuilding
#include "vehicle.h"   // Vehicle (parts)

#include "btcommon.h"  // complete Bullet types (double precision)
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <cmath>
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

// Render-frame point -> window pixel, the inverse of pickRay's unprojection.
// The same -renderOrigin shifts cancel, so p_render maps straight as
// v = R * (pS - pos). false if behind the camera.
// pickRay reads the projection's w-row Z coefficient (cam.projection[2][3])
// rather than assuming it; this inverts the specific value that coefficient
// has for every camera here (camera.cpp builds it as -1), which is what lets
// the -v.z divide stand in for it.
bool projectPixel(const Game &g, const glm::dvec3 &pS, double &px, double &py) {
    const Camera &cam = *g.camera;
    const glm::dmat3 R(cam.view);
    const glm::dvec3 v = R * (pS - cam.pos);
    if(v.z >= -1e-6) { return false; }
    const double fx = cam.projection[0][0];
    const double fy = cam.projection[1][1];
    const double nx = fx * v.x / (-v.z);
    const double ny = fy * v.y / (-v.z);
    px = (nx + 1.0) * 0.5 * (double)cam.viewport_w;
    py = (1.0 - ny) * 0.5 * (double)cam.viewport_h;
    return true;
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

std::string aimAtPoint(Game &g, const glm::dvec3 &pS,
                       const std::function<bool(int, int)> &verify,
                       double &sx, double &sy) {
    if(g.camera == nullptr) { return "no camera"; }
    double ox = 0, oy = 0;
    if(!projectPixel(g, pS, ox, oy)) { return "behind the camera"; }
    const int vw = g.camera->viewport_w, vh = g.camera->viewport_h;
    auto hits = [&](int px, int py) {
        if(px < 0 || py < 0 || px >= vw || py >= vh) { return false; }
        return verify(px, py);
    };
    if(hits((int)ox, (int)oy)) { sx = ox; sy = oy; return std::string(); }
    // Rings around the projected centre. A part's screen footprint is a few
    // tens of pixels at the editor's framing, so this reaches past any single
    // part while staying cheap (each probe is one pick over the ship).
    for(int r = 6; r <= 160; r += 6) {
        for(int a = 0; a < 360; a += 15) {
            const double rad = (double)a * std::numbers::pi / 180.0;
            const int px = (int)(ox + std::cos(rad) * r);
            const int py = (int)(oy + std::sin(rad) * r);
            if(hits(px, py)) { sx = px; sy = py; return std::string(); }
        }
    }
    return "not clickable from this camera angle (orbit and aim again)";
}

std::string flightAimAtPart(Game &g, const std::string &shipName,
                            const std::string &partId, double &sx, double &sy) {
    if(g.ship == nullptr) { return "no active ship to pick against"; }
    Vehicle *want = nullptr;
    for(Vehicle *s : g.vehicles) {
        if(s->name == shipName) { want = s; break; }
    }
    if(want == nullptr) { return "no ship named '" + shipName + "' in the world"; }
    size_t wantPart = 0;
    bool found = false;
    for(size_t i = 0; i < want->parts.size(); i++) {
        if(want->parts[i]->id == partId) { wantPart = i; found = true; break; }
    }
    if(!found) {
        return "no part '" + partId + "' on ship '" + shipName + "'";
    }
    btCompoundShape *cs = want->compoundShape();
    if(cs == nullptr || (int)wantPart >= cs->getNumChildShapes()) {
        return "ship '" + shipName + "' has no collision shape";
    }

    /* The part's own transform, exactly as pickShipPart casts against it, then
       hopped into the frame the camera's view is built in. */
    const btTransform child = want->hull->btBody->getCenterOfMassTransform()
                            * cs->getChildTransform((int)wantPart);
    const btVector3 c = child.getOrigin();
    const glm::dvec4 pS = want->renderXform(g.ship->frame)
                        * glm::dvec4(c.getX(), c.getY(), c.getZ(), 1.0);

    return aimAtPoint(g, glm::dvec3(pS.x, pS.y, pS.z),
                      [&](int px, int py) {
        Vehicle *gotShip = nullptr;
        size_t gotPart = 0;
        PickBodyHit hit;
        return pickShipPart(g, px, py, gotShip, gotPart, hit)
            && gotShip == want && gotPart == wantPart;
    }, sx, sy);
}

/* One deck holds one ship: two grounded ships wake and torque each other, so an
   occupied pad is a collision, not a cosmetic stack (#231). Rays go up the pad's
   axis on a grid across the hull's deck box; the 0 samples are load-bearing (the
   VAB parks a ship dead centre). Not exhaustive by design -- parts between sample
   lines and ships clear of or hovering over the deck are missed. Only ship parts
   are cast, so the rays' trip through pad and ground is free. */
Vehicle *findPadOccupant(const StaticBuilding *site,
                         const std::vector<Vehicle *> &vehicles) {
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
    for(Vehicle *s : vehicles) {
        btCompoundShape *cs = s->compoundShape();
        if(cs == nullptr) { continue; }
        // The same-SoI filter a body-side list used to give for free.
        if(s->m_parent != hb) { continue; }
        /* A seated kerbal is never in this list -- it is a seat on its capsule's
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
