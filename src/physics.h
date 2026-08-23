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
    /* Weld two parts at the given local anchor points (the anchor points
       must coincide in world space, i.e. they define the relative offset). */
    void * GlueTogether(Body *parent, Body *child,
                        glm::dvec3 parentAnchor, glm::dvec3 childAnchor);
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
void SetMass(Body *body, double newMass);
void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force);
void ApplyCentralForce(Body *body, glm::dvec3 dir, double mag);
void ApplyCentralForce(Body *body, glm::dvec3 force);
void ApplyCentralForceForward(Body *body, double mag);
void ApplyTorque(Body *body, glm::dvec3 dir, double mag);
void ApplyTorque(Body *body, glm::dvec3 torque);
void ApplyTorqueRelX(Body *body, double mag);
void ApplyTorqueRelY(Body *body, double mag);
void ApplyTorqueRelZ(Body *body, double mag);
/* local axis n (0/1/2) of the body, in world coordinates */
glm::dvec3 getRelAxis_(Body *body, int n);
/* the body's local moment-of-inertia diagonal (kg m^2), as Bullet has it */
glm::dvec3 getInertiaDiag(Body *body);

void SetVelocity(Body *body, glm::dvec3 vel);
void setPosRot(Body *body, glm::dvec3 pos, glm::dmat3 rot);
void setGravity(Body *body, double acc);

glm::dvec3 GetPosition(Body *body);
glm::dvec3 GetVelocity(Body *body);
glm::dvec3 GetAngVelocity(Body *b);
glm::dvec3 getCOM(Body *body);
glm::dmat3 GetOrient(Body *body);

/* Weld two parts; anchors are local points that must coincide in world
   space (they define the relative offset, e.g. faces at +-h/2). */
void * GlueTogether(Body *parent, Body *child,
                    glm::dvec3 parentAnchor, glm::dvec3 childAnchor);

void debug_draw(const Camera * camera);
