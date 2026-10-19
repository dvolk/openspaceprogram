// pick.h -- screen-space picking: which part did the player point at?
// castRay is the physics-free seam (no rigid body) the VAB build tree picks
// against.

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

struct Camera;
struct Body;
struct Game;
class StaticBuilding;
class Vehicle;
class btCollisionObject;
class btCollisionShape;
class btTransform;

// A ray in some frame (the frame the test bodies live in).
struct PickRay {
    glm::dvec3 origin;
    glm::dvec3 dir;     // unit
};

// A hit, in the ray's frame (== the body's frame, by contract).
struct PickBodyHit {
    glm::dvec3 point;
    glm::dvec3 normal;
    double dist;        // from the ray origin
};

// Window pixel (top-left origin) -> ray in the render frame.
PickRay pickRay(const Camera &cam, int W, int H, int px, int py);

// The inverse: a render-frame point -> window pixel. false if behind the camera.
bool projectPixel(const Game &g, const glm::dvec3 &pS, double &px, double &py);

// Ray vs one body's collision shape. ray must be in the body's frame.
bool pickBody(const PickRay &ray, const Body *body, PickBodyHit &hit);

// Ray vs one collision shape at one transform (no rigid body required).
bool castRay(const PickRay &ray, btCollisionObject *obj,
             const btCollisionShape *shape, const btTransform &xform,
             PickBodyHit &hit);

// Nearest ship part under a pixel, across every ship. false = miss.
bool pickShipPart(Game &g, int px, int py,
                  Vehicle *&ship, size_t &part, PickBodyHit &hit);

/* The shared shape of "aim at a part": project pS to a pixel, then spiral
   outward until a real pick confirms the part is actually reachable there.
   Empty string = aimed and (sx,sy) is such a pixel; otherwise the reason, so a
   case naming the wrong part fails loudly instead of clicking empty space.
   Projecting the centre is not enough -- the pick keeps the nearest hit, so an
   interior part usually sits behind whatever is in front of it along the view
   ray. Nothing is faked: every candidate pixel is confirmed by verify(), which
   must run the same pick a click would. If no pixel in the spiral confirms it,
   the part really is hidden from this angle and the case has to orbit, exactly
   as a player would. */
std::string aimAtPoint(Game &g, const glm::dvec3 &pS,
                       const std::function<bool(int, int)> &verify,
                       double &sx, double &sy);

/* Where <partId> on the ship named <shipName> can be CLICKED, for --ui-focus.
   Mirrors the VAB's vabAimAtPart. */
std::string flightAimAtPart(Game &g, const std::string &shipName,
                            const std::string &partId, double &sx, double &sy);

/* The first vessel standing on a launch pad's deck, null if the deck is
   clear. Rays go up the pad's own axis from below the deck, so a ship that
   LANDED there counts like one placed there. Deliberately narrow: "is
   something standing on this deck", not "is the pad safe". The vehicle list is
   filtered to the pad's own SoI body before the rays are cast. */
Vehicle *findPadOccupant(const StaticBuilding *site,
                         const std::vector<Vehicle *> &vehicles);
