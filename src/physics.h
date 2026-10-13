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
    void RegisterObject(Body *body, glm::vec3 pos, glm::vec3 rot);
    void BuildHull(Body *body);
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

void SetVelocity(Body *body, glm::dvec3 vel);
void SetAngVelocity(Body *body, glm::dvec3 vel);
/* Teleports pose ONLY: proceedToTransform is just setCenterOfMassTransform,
   so linear and angular velocity survive the teleport untouched. Clear them
   explicitly if that is what you want. */
void setPosRot(Body *body, glm::dvec3 pos, glm::dmat3 rot);

glm::dvec3 GetPosition(Body *body);
glm::dvec3 GetVelocity(Body *body);
glm::dvec3 GetAngVelocity(Body *b);
glm::dmat3 GetOrient(Body *body);

/* True when the body touches anything in the current world state (the
   EVA grounded check). */
bool BodyInContact(Body *body);

void debug_draw(const Camera * camera);

/* Step the global physics world one substep. The caller re-applies the forces
   before EVERY call -- stepSimulation clears accumulated forces on exit. */
void physics_tick(float timeStep);

/* Create the global physics world. Must run before any Body is built with
   collision. */
void create_physics(void);
