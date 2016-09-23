#pragma once

#include <glm/glm.hpp>

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
    void RegisterObject(Body *body, glm::vec3 pos,
                        glm::vec3 rot, bool planet);
    btRigidBody *AddTerrainCollision(Mesh *mesh);
    void RemoveTerrainCollision(btRigidBody *b);
    void * GlueTogether(Body *parent, Body *child);
    void collisions(void);
    void Draw(const Camera * camera);
    void Detach(void * constraint);

 private:
    btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btBroadphaseInterface *overlappingPairCache;
    btSequentialImpulseConstraintSolver *solver;
    btDiscreteDynamicsWorld *dynamicsWorld;
    btCollisionShape *debugShape;
    btCollisionShape *planetShape;
    GLDebugDrawer *debugDrawer;
};

btRigidBody *addTerrainCollision(Mesh *m);
void removeTerrainCollision(btRigidBody *b);
void NeverSleep(Body *body);

double GetMass(Body *body);
void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force);
void ApplyCentralForce(Body *body, glm::dvec3 dir, double mag);
void ApplyCentralForce(Body *body, glm::dvec3 force);
void ApplyCentralForceForward(Body *body, double mag);
void ApplyTorque(Body *body, glm::dvec3 dir, double mag);
void ApplyTorqueRelX(Body *body, double mag);
void ApplyTorqueRelY(Body *body, double mag);
void ApplyTorqueRelZ(Body *body, double mag);

void SetVelocity(Body *body, glm::dvec3 vel);
void setGravity(Body *body, double acc);

glm::dvec3 GetPosition(Body *body);
glm::dvec3 GetVelocity(Body *body);
glm::dvec3 GetAngVelocity(Body *b);
glm::dvec3 getCOM(Body *body);

void * GlueTogether(Body *parent, Body *child);

void debug_draw(const Camera * camera);
