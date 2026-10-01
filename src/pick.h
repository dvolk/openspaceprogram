// pick.h -- screen-space picking: which part did the player point at?
// Three layers: pickRay (pixel -> ray in the render frame), pickBody
// (ray vs one body's collision shape via Bullet), pickShipPart (nearest
// ship part under a pixel). castRay is the physics-free seam (no rigid
// body) the VAB build tree picks against.

#pragma once

#include <cstddef>

#include <glm/glm.hpp>

struct Camera;
struct Body;
struct Game;
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

// Ray vs one body's collision shape. ray must be in the body's frame.
bool pickBody(const PickRay &ray, const Body *body, PickBodyHit &hit);

// Ray vs one collision shape at one transform (no rigid body required).
bool castRay(const PickRay &ray, btCollisionObject *obj,
             const btCollisionShape *shape, const btTransform &xform,
             PickBodyHit &hit);

// Nearest ship part under a pixel, across every ship. false = miss.
bool pickShipPart(Game &g, int px, int py,
                  Vehicle *&ship, size_t &part, PickBodyHit &hit);
