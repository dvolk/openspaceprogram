#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <glm/gtx/transform.hpp>

#include <iostream>

#include "physics.h"
#include "body.h"

PhysicsEngine *physics;

class GLDebugDrawer : public btIDebugDraw
{
    int m_debugMode;
public:
    GLDebugDrawer();
    virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& color);
    virtual void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
    virtual void reportErrorWarning(const char* warningString);
    virtual void draw3dText(const btVector3& location,const char* textString);
    virtual void setDebugMode(int debugMode);
    virtual int getDebugMode() const { return m_debugMode;}

};

GLDebugDrawer::GLDebugDrawer() :m_debugMode(0) {
}

void GLDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
    glLineWidth(1);
    glBegin(GL_LINES);
    glColor3f(color[0],color[1],color[2]);
    glVertex3d(from[0],from[1],from[2]);
    glVertex3d(to[0],to[1],to[2]);
    glEnd();
}

void    GLDebugDrawer::setDebugMode(int debugMode)
{
    m_debugMode = debugMode;
}

void    GLDebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
    //glRasterPos3f(location.x(),  location.y(),  location.z());
    //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
}

void    GLDebugDrawer::reportErrorWarning(const char* warningString)
{
    printf(warningString);
}

void    GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
    {
        //btVector3 to=pointOnB+normalOnB*distance;
        //const btVector3&from = pointOnB;
        //glColor4f(color.getX(), color.getY(), color.getZ(), 1.0f);

        //GLDebugDrawer::drawLine(from, to, color);

        //glRasterPos3f(from.x(),  from.y(),  from.z());
        //char buf[12];
        //sprintf(buf," %d",lifeTime);
        //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
    }
}

void debug_draw(void) {
    physics->Draw();
}

void create_physics(void) {
    physics = new PhysicsEngine;
}

void PhysicsEngine::Draw() {
    dynamicsWorld->debugDrawWorld();
}

PhysicsEngine::PhysicsEngine() {
    printf("sizeof(btScalar): %d\n", sizeof(btScalar));
    assert(sizeof(btScalar) == 8);

    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, 0, 0));
    dynamicsWorld->setApplySpeculativeContactRestitution(true);

    debugShape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
    planetShape = new btBoxShape(btVector3(10, 10, 10));

    GLDebugDrawer *debugDrawer = new GLDebugDrawer;
    dynamicsWorld->setDebugDrawer(debugDrawer);
    debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
}

PhysicsEngine::~PhysicsEngine() {
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
    delete debugShape;
}

void PhysicsEngine::tick(float timeStep) {
    dynamicsWorld->stepSimulation(timeStep, 3, timeStep/3);
}

void physics_tick(float timeStep) {
    physics->tick(timeStep);
}

void setRigidBody(Body *b, btRigidBody *rb);

btRigidBody *addTerrainCollision(Mesh *m) {
    return physics->AddTerrainCollision(m);
}

void removeTerrainCollision(btRigidBody *b) {
    physics->RemoveTerrainCollision(b);
}

void PhysicsEngine::RemoveTerrainCollision(btRigidBody *b) {
    dynamicsWorld->removeRigidBody(b);
}

btRigidBody *PhysicsEngine::AddTerrainCollision(Mesh *m) {
    btTransform startTransform;
    startTransform.setIdentity();

    btTriangleIndexVertexArray *mesh_interface
        = new btTriangleIndexVertexArray(m->num_indices / 3,
                                         m->is,
                                         3*sizeof(int), // grr bytes!
                                         m->num_vertices,
                                         m->vs,
                                         3*sizeof(double));

    btBvhTriangleMeshShape *terrain
        = new btBvhTriangleMeshShape(mesh_interface, true, true);
    terrain->setMargin(0.5);

    btDefaultMotionState* myMotionState =
        new btDefaultMotionState(startTransform);

    btVector3 localInertia(0.0f, 0.0f, 0.0f);

    btRigidBody::btRigidBodyConstructionInfo
        rbInfo(0, myMotionState, terrain, localInertia);

    btRigidBody *b = new btRigidBody(rbInfo);

    dynamicsWorld->addRigidBody(b);

    return b;
}

void PhysicsEngine::RegisterObject(Body *body, glm::vec3 pos,
                                   glm::vec3 rot, bool debug_mesh)
{
    btTransform startTransform;
    startTransform.setIdentity();

    btCollisionShape *shape;
    if(debug_mesh == false) {
        Mesh *m = body->model->mesh;

	printf("PhysicsEngine::RegisterObject(): m->num_indices: %d\n", m->num_indices);
	printf("PhysicsEngine::RegisterObject(): m->num_vertices: %d\n", m->num_vertices);
	assert(m->num_indices > 3);
	assert(m->num_vertices > 3);

        btTriangleIndexVertexArray *mesh_interface
            = new btTriangleIndexVertexArray(m->num_indices / 3,
                                             m->is,
                                             3*sizeof(int), // grr bytes!
                                             m->num_vertices,
                                             m->vs,
                                             3*sizeof(double));

        btBvhTriangleMeshShape *mesh_shape
            = new btBvhTriangleMeshShape(mesh_interface, true, true);

	mesh_shape->setMargin(0.5);

        shape = mesh_shape;
    }
    else {
        shape = debugShape;
    }

    startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));
    btQuaternion euler_rot(rot.x, rot.y, rot.z);
    startTransform.setRotation(euler_rot);

    btDefaultMotionState* myMotionState =
        new btDefaultMotionState(startTransform);

    btVector3 localInertia(1.0f, 1.0f, 1.0f);

    if(body->mass != 0.0f) {
        shape->calculateLocalInertia(body->mass, localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo
        rbInfo(body->mass, myMotionState, shape, localInertia);

    rbInfo.m_friction = 4.0;

    btRigidBody *b = new btRigidBody(rbInfo);

    setRigidBody(body, b);
    dynamicsWorld->addRigidBody(b);
}

btRigidBody* getRigidBody(Body *b);



void *PhysicsEngine::GlueTogether(Body *parent, Body *child) {
    btRigidBody *btParent = getRigidBody(parent);
    btRigidBody *btChild = getRigidBody(child);

    // btPoint2PointConstraint *constraint =
    //     new btPoint2PointConstraint(*btParent, *btChild,
    //                                 btVector3(0,-1,0), btVector3(0,1,0));

    btTransform t1 = btTransform(btQuaternion(1,1,1), // what's this?
                                 btVector3(0,0,-1));
    btTransform t2 = btTransform(btQuaternion(1,1,1),
                                 btVector3( 0,0,1));

    btGeneric6DofConstraint *constraint =
        new btGeneric6DofConstraint(*btParent, *btChild, t1, t2, false);

    constraint->setAngularLowerLimit(btVector3(0, 0, 0));
    constraint->setAngularUpperLimit(btVector3(0, 0, 0));

    dynamicsWorld->addConstraint(constraint, true);

    return (void *)constraint;
}

void Detach(void *constraint) {
  physics->Detach(constraint);
}

void PhysicsEngine::Detach(void *constraint) {
  dynamicsWorld->removeConstraint((btTypedConstraint *)constraint);
}

void RegisterPhysicsBody(Body *body,
                         glm::vec3 pos, glm::vec3 rot, bool planet) {
    physics->RegisterObject(body, pos, rot, planet);
}

void ApplyCentralForce(Body *body, glm::dvec3 dir, double mag) {
    btVector3 ndir = btVector3(dir.x, dir.y, dir.z).normalized();
    getRigidBody(body)->applyCentralForce(mag * ndir);
}

void ApplyCentralForce(Body *body, glm::dvec3 force) {
    getRigidBody(body)->applyCentralForce(btVector3(force.x, force.y, force.z));
}

void SetMass(Body *body, double newMass) {
  getRigidBody(body)->setMassProps(newMass, btVector3(1, 1, 1) /* fixme */);
}

void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force) {
    getRigidBody(body)->applyForce(btVector3(rel.x, rel.y, rel.z),
                                   btVector3(force.x, force.y, force.z));
}

void ApplyTorque(Body *body, glm::dvec3 torque) {
  getRigidBody(body)->applyTorque(btVector3(torque.x, torque.y, torque.z));
}

glm::dvec3 GetPosition(Body *b) {
    const btVector3& pos = getRigidBody(b)->getCenterOfMassPosition();
    return glm::dvec3(pos.getX(), pos.getY(), pos.getZ());
}

glm::dvec3 GetVelocity(Body *b) {
    const btVector3& vel = getRigidBody(b)->getLinearVelocity();
    return glm::dvec3(vel.getX(), vel.getY(), vel.getZ());
}

glm::dvec3 GetAngVelocity(Body *b) {
    const btVector3& vel = getRigidBody(b)->getAngularVelocity();
    return glm::dvec3(vel.getX(), vel.getY(), vel.getZ());
}

void SetVelocity(Body *b, glm::dvec3 vel) {
    btVector3 btvel = btVector3(vel.x, vel.y, vel.z);
    getRigidBody(b)->setLinearVelocity(btvel);
}

glm::dvec3 getCOM(Body *body) {
    btVector3 COM = getRigidBody(body)->getCenterOfMassTransform().getOrigin();
    return glm::dvec3(COM.getX(), COM.getY(), COM.getX());
}

double GetMass(Body *body) {
    return body->mass;
}

btVector3 getRelAxis(Body *body, int n) {
    return getRigidBody(body)->getCenterOfMassTransform().getBasis().getColumn(n);
}

glm::dvec3 getRelAxis_(Body *body, int n) {
    btVector3 v = getRigidBody(body)->getCenterOfMassTransform().getBasis().getColumn(n);
    return glm::dvec3(v.getX(), v.getY(), v.getZ());
}

double angleFacing(Body *body, glm::dvec3 dir) {
  return getRelAxis(body, 2).angle(btVector3(dir.x, dir.y, dir.z));
}

void ApplyCentralForceForward(Body *body, double mag) {
    btVector3 forward = getRelAxis(body, 2);
    getRigidBody(body)->applyCentralForce(mag * forward.normalized());
}

void setGravity(Body *body, double acc) {
    const btVector3 dir = getRigidBody(body)->getCenterOfMassPosition();
    getRigidBody(body)->setGravity(acc * dir.normalized());
}

void ApplyTorqueRelX(Body *body, double mag) {
    const btVector3& axis = getRelAxis(body, 0);
    getRigidBody(body)->applyTorque(mag * axis.normalized());
}
void ApplyTorqueRelY(Body *body, double mag) {
    const btVector3& axis = getRelAxis(body, 1);
    getRigidBody(body)->applyTorque(mag * axis.normalized());
}
void ApplyTorqueRelZ(Body *body, double mag) {
    const btVector3& axis = getRelAxis(body, 2);
    getRigidBody(body)->applyTorque(mag * axis.normalized());
}

void ApplyTorque(Body *body, glm::dvec3 dir, double mag) {
    btVector3 ndir = btVector3(dir.x, dir.y, dir.z).normalized();
    getRigidBody(body)->applyTorque(mag * ndir);
}

void *GlueTogether(Body *parent, Body *child) {
    return physics->GlueTogether(parent, child);
}

void PhysicsEngine::collisions() {
    //Perform collision detection
    dynamicsWorld->performDiscreteCollisionDetection();

    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    printf("manifolds: %d\n", numManifolds);
    //For each contact manifold
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
        const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
        int numContacts = contactManifold->getNumContacts();
        //For each contact point in that manifold
        for (int j = 0; j < numContacts; j++) {
            //Get the contact information
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            btVector3 ptA = pt.getPositionWorldOnA();
            btVector3 ptB = pt.getPositionWorldOnB();
            double ptdist = pt.getDistance();
            printf("cdist: %f\n", ptdist);
        }
    }
}

void collisions() {
    physics->collisions();
}

void NeverSleep(Body *body) {
    getRigidBody(body)->setSleepingThresholds(0.0, 0.0);
}
