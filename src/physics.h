#pragma once

#include <glm/glm.hpp>
#include <vector>

class Mesh;
class Body;
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
    btRigidBody *AddTerrainCollision(Mesh *mesh);
    void RemoveTerrainCollision(btRigidBody *b);
    /* Remove a body's rigid body from the dynamics world (call BEFORE
       deleting the Body). The collision shape / model are the Body's to
       free; this only unregisters it so the world holds no dangling ptr. */
    void RemoveBody(Body *body);
    /* Re-add a parked body's EXISTING rigid body to the world (the inverse
       of RemoveBody; the rails handoff parks and restores ship parts). */
    void AddBody(Body *body);
    /* True when the body has any contact point in the current world state
       (terrain, pads, ships -- whatever it touches). */
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

btRigidBody *addTerrainCollision(Mesh *m);
void removeTerrainCollision(btRigidBody *b);
void NeverSleep(Body *body);
/* Unregister a body's rigid body from the world (call before `delete body`). */
void RemoveBody(Body *body);
/* Re-add a parked body's rigid body to the world (inverse of RemoveBody). */
void AddPhysicsBody(Body *body);

/* Force applied at `rel`, an offset from the body's centre of mass -- the
   primitive that lets several parts push a SINGLE rigid body correctly, each
   contributing its share of the net force plus the torque from its own
   offset. This is how a ship's thrust and its per-part gravity are now
   delivered, so an off-axis engine and the tide both turn the ship. */
void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force);
void ApplyCentralForce(Body *body, glm::dvec3 force);
void ApplyTorque(Body *body, glm::dvec3 torque);
/* the body's local moment-of-inertia diagonal (kg m^2), from its shape at its
   current mass */
glm::dvec3 getInertiaDiag(Body *body);

void SetVelocity(Body *body, glm::dvec3 vel);
void SetAngVelocity(Body *body, glm::dvec3 vel);
void SetFriction(Body *body, double f);
/* Teleports pose AND zeroes both velocities (proceedToTransform) --
   right for rails handoffs, a trap for live bodies. */
void setPosRot(Body *body, glm::dvec3 pos, glm::dmat3 rot);

glm::dvec3 GetPosition(Body *body);
glm::dvec3 GetVelocity(Body *body);
glm::dvec3 GetAngVelocity(Body *b);
glm::dmat3 GetOrient(Body *body);

/* True when the body touches anything in the current world state (the
   EVA grounded check; see PhysicsEngine::BodyInContact). */
bool BodyInContact(Body *body);

void debug_draw(const Camera * camera);

/* Step the global physics world one substep. The caller (the logic tick,
   tick.cpp) re-applies the forces before EVERY call -- stepSimulation
   clears accumulated forces on exit (see PhysicsEngine::tick). */
void physics_tick(float timeStep);
