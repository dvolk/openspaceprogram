#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <iostream>
#include <cstdlib>

#include "physics.h"
#include "body.h"
#include "mesh.h"
#include "camera.h"
#include "shader.h"
#include "gldebug.h"

PhysicsEngine *physics;

/* Convex-hull margin (m), tunable via OSP_HULL_MARGIN.
   Each part's collision hull is its mesh expanded by this much on every
   face. Because the ship's parts are welded face-to-face (the visible meshes
   touch exactly), two adjacent hulls overlap by 2*margin -- the contact
   solver then fires an impulse to resolve that overlap on EVERY substep.
   A large margin (the old 0.5 m -> 1.0 m overlap) is fine for chunky parts
   but destabilizes a thin one (the 0.25 m reaction-wheel disc): the impulse
   the solver applies to its low moment of inertia tumbles the whole ship.
   0.1 m (0.2 m overlap) keeps every current part stable, verified headless. */
static double hull_margin() {
    const char *e = getenv("OSP_HULL_MARGIN");
    if(e && e[0]) { return strtod(e, NULL); }
    return 0.1;
}

class GLDebugDrawer : public btIDebugDraw {
    int m_debugMode;
    Shader *lineshader;
    GLuint m_vao;
    GLuint m_bufs[1];

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

void GLDebugDrawer::Draw(const Camera * camera)
{
    const glm::mat4 view = camera->GetView();
    const glm::mat4 projection = camera->GetProjection();

    int attribute_pos = glGetAttribLocation(lineshader->m_program, "pos");
    check_gl_error();
    lineshader->Bind();
    check_gl_error();
    lineshader->setUniform_mat4(0, projection * view);
    check_gl_error();
    glBindVertexArray(m_vao);
    check_gl_error();
    glBindBuffer(GL_ARRAY_BUFFER, m_bufs[0]);
    check_gl_error();
    glBufferData(GL_ARRAY_BUFFER, sizeof(lineBuffer.data()[0]) * lineBuffer.size(), lineBuffer.data(), GL_STATIC_DRAW);
    check_gl_error();
    glEnableVertexAttribArray(0);
    check_gl_error();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    check_gl_error();
    glDrawArrays(GL_LINES, 0, lineBuffer.size() / 3);
    check_gl_error();
    glBindVertexArray(0);
    check_gl_error();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    check_gl_error();
}

void GLDebugDrawer::init() {
    lineBuffer.reserve(512 * 1024);

    lineshader = new Shader;
    lineshader->registerAttribs({ "pos" });
    lineshader->registerUniforms({ "VP" });
    lineshader->FromFile("./res/lineShader");

    m_debugMode = DBG_DrawWireframe;

    glGenVertexArrays(1, &m_vao);
    check_gl_error();
    glGenBuffers(1, &m_bufs[0]);
    check_gl_error();
}

void GLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
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
    printf("sizeof(btScalar): %lu\n", sizeof(btScalar));
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
    // Integrate a single substep. The caller (the main logic loop) is
    // responsible for re-applying the forces (gravity + rotating-frame
    // fictitious terms) before EVERY call, because stepSimulation clears
    // accumulated forces on exit. Splitting the step into multiple
    // stepSimulation calls here WITHOUT re-applying the forces in between
    // would leave the ship force-free for all but the first substep.
    dynamicsWorld->stepSimulation(timeStep, 1, timeStep);
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

        printf("PhysicsEngine::RegisterObject(): m->num_vertices: %d\n", m->num_vertices);
        assert(m->vs != NULL);
        assert(m->num_vertices >= 3);

        /* Convex hull of the part mesh. Bullet has no collision
           algorithm for concave-vs-concave pairs (the dispatcher
           falls through to btEmptyAlgorithm), so dynamic bodies
           must stay convex. The hull keeps the part's real
           silhouette (vs the 2 m debug box) and pairs correctly
           with the triangle-mesh world (terrain / space port). */
        btConvexHullShape *hull = new btConvexHullShape(m->vs, (int)m->num_vertices,
                                                        3 * sizeof(double));

        hull->setMargin(hull_margin());

        shape = hull;
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



void *PhysicsEngine::GlueTogether(Body *parent, Body *child,
                                  glm::dvec3 parentAnchor, glm::dvec3 childAnchor) {
    btRigidBody *btParent = getRigidBody(parent);
    btRigidBody *btChild = getRigidBody(child);

    // btPoint2PointConstraint *constraint =
    //     new btPoint2PointConstraint(*btParent, *btChild,
    //                                 btVector3(0,-1,0), btVector3(0,1,0));

    /* The 6DOF constraint locks the CURRENT relative Euler angle to the
       angular limits (0,0,0 here), i.e. it drives the relative ORIENTATION
       toward the identity. That preserves the parts' actual relative pose
       only if the two constraint frames already share a world rotation --
       which is true iff frameA_world and frameB_world use the SAME
       rotation. So build both frames as (identity rotation, anchor):
       the anchors coincide in world space by construction (the callers
       choose local anchors that land on the same world point), and the
       identity-vs-identity relative rotation is satisfied from the first
       solve for ANY relative part orientation (stacked or radial).

       The old code used one arbitrary quaternion (1,1,1) for both frames;
       the relative rotation it implied was the parts' actual relative
       rotation CONJUGATED by that quaternion, which equals the identity
       only when the parts share a world orientation (the stacked case).
       A radially attached part (perpendicular axes) started ~90 deg off,
       and the solver's first pass kicked it toward parallel at tens of
       rad/s -- the radial spin bug. */
    btQuaternion qA, qB;
    btParent->getCenterOfMassTransform().getBasis().getRotation(qA);
    btChild->getCenterOfMassTransform().getBasis().getRotation(qB);
    qA = qA.inverse(); /* local rotation so the world frame rotation is I */
    qB = qB.inverse();
    btTransform t1 = btTransform(qA,
                                 btVector3(parentAnchor.x, parentAnchor.y, parentAnchor.z));
    btTransform t2 = btTransform(qB,
                                 btVector3(childAnchor.x, childAnchor.y, childAnchor.z));

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
    btTypedConstraint *c = (btTypedConstraint *)constraint;
    dynamicsWorld->removeConstraint(c);
    /* btTypedConstraint has a virtual dtor, so deleting through the base
       pointer frees the concrete (6DOF) constraint. */
    delete c;
}

void PhysicsEngine::RemoveBody(Body *body) {
    dynamicsWorld->removeRigidBody(body->btBody);
}

void RemoveBody(Body *body) {
    physics->RemoveBody(body);
}

void PhysicsEngine::AddBody(Body *body) {
    dynamicsWorld->addRigidBody(body->btBody);
}

void AddPhysicsBody(Body *body) {
    physics->AddBody(body);
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
    // setMassProps takes the inertia tensor AS-IS, so it must be recomputed
    // from the collision shape (as RegisterObject does) -- a fixed
    // btVector3(1,1,1) would silently reset the body's moment of inertia
    // to the identity on every call.
    btRigidBody *rb = getRigidBody(body);
    btVector3 inertia(0, 0, 0);
    rb->getCollisionShape()->calculateLocalInertia(newMass, inertia);
    rb->setMassProps(newMass, inertia);
}

void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force) {
    getRigidBody(body)->applyForce(btVector3(rel.x, rel.y, rel.z),
                                   btVector3(force.x, force.y, force.z));
}

void ApplyTorque(Body *body, glm::dvec3 torque) {
    getRigidBody(body)->applyTorque(btVector3(torque.x, torque.y, torque.z));
}

glm::dvec3 getInertiaDiag(Body *body) {
    const btVector3& i = getRigidBody(body)->getLocalInertia();
    return glm::dvec3(i.getX(), i.getY(), i.getZ());
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
    // Read the orientation through a quaternion. Bullet's basis matrix is
    // stored row-major (m_el[i] = row i) while a glm::dmat3 is column-major,
    // so a direct element copy (the old make_mat3x3) silently transposed the
    // orientation. A quaternion is four scalars with an unambiguous order,
    // sidestepping the row/col-major trap entirely.
    btQuaternion q;
    getRigidBody(b)->getCenterOfMassTransform().getBasis().getRotation(q);
    // GLM's 4-scalar quaternion constructor is (w, x, y, z) -- w FIRST.
    glm::dquat gq(q.w(), q.x(), q.y(), q.z());
    return glm::mat3_cast(gq);
}

void SetVelocity(Body *b, glm::dvec3 vel) {
    btVector3 btvel = btVector3(vel.x, vel.y, vel.z);
    getRigidBody(b)->setLinearVelocity(btvel);
}

void SetAngVelocity(Body *b, glm::dvec3 vel) {
    btVector3 btvel = btVector3(vel.x, vel.y, vel.z);
    getRigidBody(b)->setAngularVelocity(btvel);
}

void setPosRot(Body *b, glm::dvec3 pos, glm::dmat3 rot)
{
    btTransform t;
    t.setIdentity();

    t.setOrigin(btVector3(pos.x, pos.y, pos.z));

    // Write the orientation through a quaternion.
    glm::dquat gq = glm::quat_cast(rot);
    // Bullet's quaternion constructor is (x, y, z, w); GLM components are by name.
    t.setRotation(btQuaternion(gq.x, gq.y, gq.z, gq.w));

    getRigidBody(b)->proceedToTransform(t);
}

glm::dvec3 getCOM(Body *body) {
    btVector3 COM = getRigidBody(body)->getCenterOfMassTransform().getOrigin();
    return glm::dvec3(COM.getX(), COM.getY(), COM.getZ());
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

// TODO this seems like it would be useful
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

void *GlueTogether(Body *parent, Body *child,
                   glm::dvec3 parentAnchor, glm::dvec3 childAnchor) {
    return physics->GlueTogether(parent, child, parentAnchor, childAnchor);
}

ContactPairInfo PhysicsEngine::reportContactPair(Body *a, Body *b) {
    btRigidBody *btA = getRigidBody(a);
    btRigidBody *btB = getRigidBody(b);
    ContactPairInfo out;

    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for(int i = 0; i < numManifolds; i++) {
        btPersistentManifold *cm =
            dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        /* pointer identity vs the two ship parts (no downcast needed) */
        const btCollisionObject *ob0 = cm->getBody0();
        const btCollisionObject *ob1 = cm->getBody1();
        const btCollisionObject *ca = (const btCollisionObject *)getRigidBody(a);
        const btCollisionObject *cb = (const btCollisionObject *)getRigidBody(b);
        if((ob0 != ca && ob0 != cb) || (ob1 != ca && ob1 != cb)) {
            out.otherManifolds++;
            continue;
        }
        if(ob0 == ob1) { continue; }

        out.manifolds++;
        cm->refreshContactPoints(ob0->getWorldTransform(), ob1->getWorldTransform());
        for(int j = 0; j < cm->getNumContacts(); j++) {
            btManifoldPoint &pt = cm->getContactPoint(j);
            ContactPointInfo ci;
            const btVector3 p = pt.getPositionWorldOnA();
            ci.pos = glm::dvec3(p.getX(), p.getY(), p.getZ());
            /* Bullet 2.x manifold point: scalar normal impulse + two
               lateral (friction) impulses along stored world directions. */
            const glm::dvec3 n(pt.m_normalWorldOnB.getX(),
                               pt.m_normalWorldOnB.getY(),
                               pt.m_normalWorldOnB.getZ());
            ci.normal = n;
            ci.pen = -pt.getDistance(); /* sign to be read off the data */
            const glm::dvec3 d1(pt.m_lateralFrictionDir1.getX(),
                                pt.m_lateralFrictionDir1.getY(),
                                pt.m_lateralFrictionDir1.getZ());
            const glm::dvec3 d2(pt.m_lateralFrictionDir2.getX(),
                                pt.m_lateralFrictionDir2.getY(),
                                pt.m_lateralFrictionDir2.getZ());
            ci.impulse = pt.m_appliedImpulse * n
                       + pt.m_appliedImpulseLateral1 * d1
                       + pt.m_appliedImpulseLateral2 * d2;
            out.points.push_back(ci);
            out.maxImpulse = std::max(out.maxImpulse, glm::length(ci.impulse));
        }
    }

    out.netForce = glm::dvec3(0, 0, 0);
    for(size_t j = 0; j < out.points.size(); j++) {
        out.netForce += out.points[j].impulse;
    }
    /* internal torque on the pair: the impulses are equal-and-opposite at
       the same world point, so only (comB - comA) x F_net survives. The
       sign follows Bullet's body ordering (which body is "a"); the
       magnitude is what spins the ship. */
    const glm::dvec3 comA = GetPosition(a);
    const glm::dvec3 comB = GetPosition(b);
    out.netTorque = glm::cross(comB - comA, out.netForce);
    return out;
}

ContactPairInfo contact_report(Body *a, Body *b) {
    return physics->reportContactPair(a, b);
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
