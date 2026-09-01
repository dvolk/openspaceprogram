// pick.h -- screen-space picking: which part did the player point at?
//
// Three layers, each usable on its own:
//   pickRay      pixel -> ray in the RENDER frame (pure camera math,
//                unit-testable headless -- the same convention as the
//                render pass: view built in the render frame, whose
//                -renderOrigin shift cancels the Draw sites' shift,
//                leaving p_view = R * (p - cam.pos)).
//   pickBody     a ray vs ONE rigid body's collision shape, via Bullet's
//                own convex/concave cast (so a pick hits exactly what
//                collides; works for the part hulls, and for triangle
//                meshes -- terrain/pads -- later).
//   pickShipPart the first concrete use of both: the nearest ship part
//                under a pixel, across every ship (each in its own frame).
//
// New pickable kinds are "add candidates": anything with a btRigidBody +
// collision shape can be ray-tested by pickBody.

#pragma once

#include <cstddef>

#include <glm/glm.hpp>

struct Camera;
struct Body;
struct Game;
class Vehicle;

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

// Window pixel (top-left origin, SDL convention) -> ray in the render
// frame. W/H = the viewport size (the window), px/py the pixel.
PickRay pickRay(const Camera &cam, int W, int H, int px, int py);

// Ray vs one body's collision shape. ray must be in the SAME frame as
// the body's world transform (callers transform it per ship). false = miss.
bool pickBody(const PickRay &ray, const Body *body, PickBodyHit &hit);

// The nearest ship part under window pixel (px,py): every part of every
// ship, nearest hit wins. false = nothing hit (ship/part/hit out-params
// are untouched on a miss).
bool pickShipPart(Game &g, int px, int py,
                  Vehicle *&ship, size_t &part, PickBodyHit &hit);
