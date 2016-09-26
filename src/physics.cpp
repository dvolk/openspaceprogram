#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

#include "physics.h"
#include "body.h"
#include "mesh.h"
#include "camera.h"
#include "shader.h"
#include "gldebug.h"

PhysicsEngine *physics;

class GLDebugDrawer : public btIDebugDraw {
  int m_debugMode;
  Shader *lineshader;
public:
  std::vector<float> lineBuffer;

  void init();
  void Draw(const Camera * camera);

  void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
  void reportErrorWarning(const char* warningString);

  /* TODO */
  void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {}
  /* TODO */
  void draw3dText(const btVector3& location, const char* textString) {}

  void setDebugMode(int debugMode) { m_debugMode = debugMode; }
  int getDebugMode() const { return m_debugMode; }
};

void GLDebugDrawer::reportErrorWarning(const char* warningString) {
  printf("!!! BULLET: %s\n", warningString);
}

void GLDebugDrawer::Draw(const Camera * camera) {

  const glm::mat4 view = camera->GetView();
  const glm::mat4 projection = camera->GetProjection();

  lineshader->Bind();
  check_gl_error();

  lineshader->setUniform_mat4(0, projection * view);
  check_gl_error();

  int attribute_pos = glGetAttribLocation(lineshader->m_program, "pos");
  check_gl_error();
  assert(attribute_pos != -1);

  glEnableVertexAttribArray(attribute_pos);
  check_gl_error();
  glVertexAttribPointer(
			attribute_pos, // attribute
			3,                 // number of elements per vertex, here (x,y)
			GL_FLOAT,          // the type of each element
			GL_FALSE,          // take our values as-is
			0,                 // no extra data between each position
			lineBuffer.data()  // pointer to the C array
			);
  check_gl_error();
  glDrawArrays(GL_LINES, 0, lineBuffer.size() / 3);
  check_gl_error();

  glDisableVertexAttribArray(attribute_pos);
  check_gl_error();

  // printf("%d\n", lineBuffer.size());
}

void GLDebugDrawer::init() {
  lineBuffer.reserve(512 * 1024);

  lineshader = new Shader;
  lineshader->registerAttribs({ "pos" });
  lineshader->registerUniforms({ "VP" });
  lineshader->FromFile("./res/lineShader");

  m_debugMode = DBG_DrawWireframe;
}

void GLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
  // lineBuffer.push_back(PosColVertex(from.getX(), from.getY(), from.getZ(),
  // 				    color.getX(), color.getY(), color.getZ()));
  // lineBuffer.push_back(PosColVertex(to.getX(), to.getY(), to.getZ(),
  // 				    color.getX(), color.getY(), color.getZ()));
  lineBuffer.push_back(from.getX());
  lineBuffer.push_back(from.getY());
  lineBuffer.push_back(from.getZ());
  lineBuffer.push_back(to.getX());
  lineBuffer.push_back(to.getY());
  lineBuffer.push_back(to.getZ());

}

void debug_draw(const Camera * camera) {
    physics->Draw(camera);
}

void create_physics(void) {
    physics = new PhysicsEngine;
}

void PhysicsEngine::Draw(const Camera * camera) {
    debugDrawer->lineBuffer.clear();
    dynamicsWorld->debugDrawWorld();
    debugDrawer->Draw(camera);
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

    debugDrawer = new GLDebugDrawer;
    debugDrawer->init();
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
  // delete b->getCollisionShape();
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

glm::dmat3 GetOrient(Body *b) {
  return glm::make_mat3x3(&getRigidBody(b)->getCenterOfMassTransform().getBasis()[0][0]);
}

void SetVelocity(Body *b, glm::dvec3 vel) {
    btVector3 btvel = btVector3(vel.x, vel.y, vel.z);
    getRigidBody(b)->setLinearVelocity(btvel);
}

// void setRotation(Body *b, glm::dmat3 rot) {

// }

// void setPosition(Body *b, glm:;dvec3 pos) {
// }

void setPosRot(Body *b, glm::dvec3 pos, glm::dmat3 rot)
{
  btTransform t;
  btMatrix3x3 r;
  t.setIdentity();

  t.setOrigin(btVector3(pos.x, pos.y, pos.z));

  r.setFromOpenGLSubMatrix((btScalar*)&rot[0][0]);
  t.setBasis(r);

  getRigidBody(b)->proceedToTransform(t);
}

// void setModelMatrix(Body *b, glm::dmat4 model) {
//   btTransform t;
//   t.setFromOpenGLMatrix(&model[0][0]);
//   getRigidBody(b)->setCenterOfMassTransform(t);
// }

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

// double angleFacing(Body *body, glm::dvec3 dir) {
//   return getRelAxis(body, 2).angle(btVector3(dir.x, dir.y, dir.z));
// }

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
