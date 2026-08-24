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

/* One contact point from the last solver pass (spin diagnostics). */
struct ContactPointInfo {
    glm::dvec3 pos;      // world
    glm::dvec3 normal;   // world (Bullet's normalOnB convention)
    double pen;          // m, > 0 = overlapping
    glm::dvec3 impulse;  // N s applied at this point in the last solve
};

/* The contact state of a two-body pair after the last solve.
   netTorque is the internal torque the pair as a whole receives from
   these contacts: (comB - comA) x sum(impulses) -- nonzero only when the
   impulses are not all parallel to the COM offset (friction / off-axis
   normals). That is the torque that spins a welded ship. The sign
   follows Bullet's body ordering; the magnitude is the quantity of
   interest. */
struct ContactPairInfo {
    int manifolds = 0;
    int otherManifolds = 0;   /* manifolds in the world NOT involving this
                                 pair -- sanity check that the matcher is
                                 seeing the dispatcher's manifold list */
    std::vector<ContactPointInfo> points;
    glm::dvec3 netForce = glm::dvec3(0, 0, 0);   // N s
    glm::dvec3 netTorque = glm::dvec3(0, 0, 0);  // N m s
    double maxImpulse = 0.0;                      // N s
};

class PhysicsEngine {
public:
    PhysicsEngine();
    ~PhysicsEngine();

    void tick(float timeStep);
    void RegisterObject(Body *body, glm::vec3 pos,
                        glm::vec3 rot, bool planet);
    btRigidBody *AddTerrainCollision(Mesh *mesh);
    void RemoveTerrainCollision(btRigidBody *b);
    /* Remove a body's rigid body from the dynamics world (call BEFORE
       deleting the Body). The collision shape / model are the Body's to
       free; this only unregisters it so the world holds no dangling ptr. */
    void RemoveBody(Body *body);
    /* Weld two parts at the given local anchor points (the anchor points
       must coincide in world space, i.e. they define the relative offset). */
    void * GlueTogether(Body *parent, Body *child,
                        glm::dvec3 parentAnchor, glm::dvec3 childAnchor);
    void collisions(void);
    void Draw(const Camera * camera);
    /* Remove a constraint from the world AND delete it (no dangling ref). */
    void Detach(void * constraint);
    /* Spin diagnostics: the contact state between two ship parts after
       the last solve (per point: world position, normal, penetration,
       applied impulse), plus the pair's net internal torque -- the only
       way the welded pair can spin itself. */
    ContactPairInfo reportContactPair(Body *a, Body *b);

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
/* Unregister a body's rigid body from the world (call before `delete body`). */
void RemoveBody(Body *body);

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
/* Remove a weld (constraint) from the world and delete it. */
void Detach(void *constraint);
/* Spin diagnostics for a two-part ship (see ContactPairInfo). */
ContactPairInfo contact_report(Body *a, Body *b);

void debug_draw(const Camera * camera);
