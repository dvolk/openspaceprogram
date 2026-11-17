#pragma once

#include <glm/glm.hpp>
#include <vector>

class Mesh;
class Body;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btCollisionShape;
class btRigidBody;

class GLDebugDrawer;
class Camera;

class PhysicsEngine {
public:
    PhysicsEngine();
    ~PhysicsEngine();

    void tick(float timeStep);
    /* collisionMesh: hull this mesh instead of body->mesh (see body.h). */
    void RegisterObject(Body *body, glm::vec3 pos, glm::vec3 rot,
                        Mesh *collisionMesh = nullptr);
    void BuildHull(Body *body, Mesh *collisionMesh = nullptr);
    /* Terrain patch collision: mesh verts are anchor-relative (terragen.h
       GridGeom), so the rigid body is placed at `anchor`. */
    btRigidBody *AddTerrainCollision(Mesh *mesh, const glm::dvec3 &anchor);
    void RemoveTerrainCollision(btRigidBody *b);
    /* Remove a body's rigid body from the dynamics world (call BEFORE
       deleting the Body). */
    void RemoveBody(Body *body);
    /* Re-add a parked body's EXISTING rigid body to the world (inverse of
       RemoveBody). */
    void AddBody(Body *body);
    bool BodyInContact(Body *body);
    bool groundRay(const glm::dvec3 &from, const glm::dvec3 &to,
                   glm::dvec3 &hit, glm::dvec3 &normal, const Body *skip,
                   Body **hitBody = nullptr);
    void Draw(const Camera * camera);

private:
    btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btBroadphaseInterface *overlappingPairCache;
    btSequentialImpulseConstraintSolver *solver;
    btDiscreteDynamicsWorld *dynamicsWorld;
    GLDebugDrawer *debugDrawer;
};

btRigidBody *addTerrainCollision(Mesh *m, const glm::dvec3 &anchor);
void removeTerrainCollision(btRigidBody *b);

/* Default convex-hull collision margin (m): OSP_HULL_MARGIN, else 0.1. Ship
   parts AND static buildings get it when they set none of their own. */
double hullMargin();
/* Terrain triangle-mesh margin (m): OSP_TERRAIN_MARGIN, else 0.5. Raw terrain
   only; a ship on a pad meets the pad's hull margin instead. */
double terrainMargin();
/* The margin (m) a Body's hull actually got: its own hull_margin when set
   (>= 0), else hullMargin(). */
double effectiveHullMargin(const Body *body);

/* Coulomb friction of a hull that sets none of its own. Bullet's own default
   (0.5) is NOT this -- such a hull would be slipperier than its terrain. */
inline constexpr double kDefaultHullFriction = 4.0;

void NeverSleep(Body *body);
void RemoveBody(Body *body);
void AddPhysicsBody(Body *body);

/* Force applied at `rel`, an offset from the body's centre of mass -- each
   push contributes net force plus the torque from its own offset. */
void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force);
void ApplyCentralForce(Body *body, glm::dvec3 force);
void ApplyTorque(Body *body, glm::dvec3 torque);
/* the body's local moment-of-inertia diagonal (kg m^2), from its shape at its
   current mass */
glm::dvec3 getInertiaDiag(Body *body);

/* The mass the body presents to an impulse along the unit `dir` applied at the
   COM offset `rel`: 1 / (1/m + (rel x dir)^T Iw^-1 (rel x dir)). A force that
   must not reverse the motion it is fighting -- a tire braking a slip -- has to
   know this; the linear mass alone ignores the pitch it also drives (#770).
   `dir` is asserted unit, not renormalised. Zero for a body that cannot move
   (infinite mass) -- note that wheel::force reads a zero stopRate as "clamp off",
   the opposite meaning, so a new caller must not pass this answer straight in. */
double impulseMass(Body *body, glm::dvec3 rel, glm::dvec3 dir);

void SetVelocity(Body *body, glm::dvec3 vel);
void SetAngVelocity(Body *body, glm::dvec3 vel);
/* Teleports pose ONLY: proceedToTransform is just setCenterOfMassTransform,
   so linear and angular velocity survive the teleport untouched. Clear them
   explicitly if that is what you want. */
void setPosRot(Body *body, glm::dvec3 pos, glm::dmat3 rot);
/* Mark a mass-0 body kinematic (driven pose, immovable). Unused in v1 (orbital
   buildings are static); reserved for the docking follow-up #848. */
void MakeKinematic(Body *body);

glm::dvec3 GetPosition(Body *body);
glm::dvec3 GetVelocity(Body *body);
glm::dvec3 GetAngVelocity(Body *b);
glm::dmat3 GetOrient(Body *body);

/* True when the body touches anything in the current world state (the
   EVA grounded check). */
bool BodyInContact(Body *body);

/* Override a body's Coulomb friction (the solver multiplies the pair's). */
void SetFriction(Body *b, double mu);

/* Cast a ray from `from` to `to` through the collision world (terrain mesh,
   buildings, hulls). True on a hit, writing the hit point and surface normal.
   `skip` is excluded -- a wheel does not ray onto its own hull. The hit is the
   margin-inflated surface (the same one collision rests on). `hitBody` (optional)
   receives the Body hit -- nullptr for terrain (no Body). */
bool groundRay(const glm::dvec3 &from, const glm::dvec3 &to,
               glm::dvec3 &hit, glm::dvec3 &normal, const Body *skip,
               Body **hitBody = nullptr);

void debug_draw(const Camera * camera);

/* Step the global physics world one substep. The caller re-applies the forces
   before EVERY call -- stepSimulation clears accumulated forces on exit. */
void physics_tick(float timeStep);

/* Create the global physics world. Must run before any Body is built with
   collision. */
void create_physics(void);
