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

/* The default convex-hull collision margin (m), overridable per part
   (res/parts.json), per ship (ShipDef::hull_margin, which wins) and wholesale
   by OSP_HULL_MARGIN -- the last is what makes the value measurable, since
   build_ship's pad lift assumes it:

       shift = -lowest + 0.6     // terrain margin 0.5 + hull margin 0.1

   so a ship that overrides the margin is placed by a formula that no longer
   describes it. It also reaches the mass properties: a convex hull inherits
   btPolyhedralConvexShape::calculateLocalInertia, which is a BOX inertia over
   the shape's AABB *inflated by the margin* -- so 0 -> 0.1 measured +1.2% /
   +1.1% / +6.5% on heavy_two's principal moments (most on the roll axis,
   where the parts' radii are small and 0.2 m of inflation is proportionally
   largest). Keep that in mind before tuning it: it is not a free parameter. */
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

    /* Line vertices are stored RELATIVE to this, subtracted in double in
       drawLine(). Bullet hands drawLine absolute frame coordinates, and the
       buffer is float32, so the subtraction has to happen BEFORE the
       narrowing: at Kerbin's radius a float32 quantum is ~7 cm and at 1 AU
       ~18 km, so absolute vertices are already quantized by the time any
       shader-side origin shift could cancel them. That is why the debug
       wireframe jittered while the ship meshes -- which shift in double,
       see body.h -- stayed solid. Same convention as the other Draw sites;
       see Camera::renderOrigin. */
    glm::dvec3 renderOrigin = glm::dvec3(0.0);

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
    // the vertices are already render-frame relative (drawLine subtracts
    // renderOrigin in double) and the view is built in the render frame, so
    // no origin shift belongs in this matrix
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
    // Subtract in double, THEN narrow to float32 (see renderOrigin): Bullet
    // hands us absolute frame coordinates, and the render-frame-relative
    // difference is the only thing the float32 buffer can hold without
    // quantizing the ship itself away.
    const glm::dvec3 a(from.getX(), from.getY(), from.getZ());
    const glm::dvec3 b(to.getX(), to.getY(), to.getZ());
    const glm::vec3 ra(a - renderOrigin);
    const glm::vec3 rb(b - renderOrigin);
    lineBuffer.push_back(ra.x);
    lineBuffer.push_back(ra.y);
    lineBuffer.push_back(ra.z);
    lineBuffer.push_back(rb.x);
    lineBuffer.push_back(rb.y);
    lineBuffer.push_back(rb.z);
}

void debug_draw(const Camera * camera) {
    physics->Draw(camera);
}

void create_physics(void) {
    physics = new PhysicsEngine;
}

void PhysicsEngine::Draw(const Camera * camera) {
    debugDrawer->lineBuffer.clear();
    // set BEFORE debugDrawWorld: drawLine subtracts it as the buffer fills
    debugDrawer->renderOrigin = camera->GetRenderOrigin();
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
                                   glm::vec3 rot)
{
    btTransform startTransform;
    startTransform.setIdentity();

    BuildHull(body);
    btCollisionShape *hull = body->shape;

    startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));
    btQuaternion euler_rot(rot.x, rot.y, rot.z);
    startTransform.setRotation(euler_rot);

    btDefaultMotionState* myMotionState =
        new btDefaultMotionState(startTransform);

    btVector3 localInertia(1.0f, 1.0f, 1.0f);

    if(body->mass != 0.0f) {
        hull->calculateLocalInertia(body->mass, localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo
        rbInfo(body->mass, myMotionState, hull, localInertia);

    rbInfo.m_friction = 4.0;

    btRigidBody *b = new btRigidBody(rbInfo);

    setRigidBody(body, b);
    dynamicsWorld->addRigidBody(b);
}

/* The convex hull of a model's mesh, stored on the Body (which owns it).
   Shared by RegisterObject (a simulated body: a space pad) and
   create_part_body (a ship part, whose hull becomes a child of the ship's
   compound and so must exist without a rigid body of its own). */
void PhysicsEngine::BuildHull(Body *body) {
    Mesh *m = body->model->mesh;

    printf("PhysicsEngine::BuildHull(): m->num_vertices: %d\n", m->num_vertices);
    assert(m->vs != NULL);
    assert(m->num_vertices >= 3);

    /* Bullet has no collision algorithm for concave-vs-concave pairs (the
       dispatcher falls through to btEmptyAlgorithm), so anything that moves
       must stay convex. The hull keeps the part's real silhouette and pairs
       correctly with the triangle-mesh world (terrain / space port). */
    btConvexHullShape *hull = new btConvexHullShape(m->vs, (int)m->num_vertices,
                                                    3 * sizeof(double));

    /* the model carries the part's resolved margin (ship def > catalog,
       see resolveHullMargin); -1 when neither sets one */
    const double margin = (body->model->hull_margin >= 0.0)
                        ? body->model->hull_margin : hull_margin();
    hull->setMargin(margin);
    body->shape = hull;
}

void BuildPartHull(Body *body) {
    physics->BuildHull(body);
}

btRigidBody* getRigidBody(Body *b);




struct AnyContactCallback : public btCollisionWorld::ContactResultCallback {
    bool any = false;
    btScalar addSingleResult(btManifoldPoint &,
                             const btCollisionObjectWrapper *, int, int,
                             const btCollisionObjectWrapper *, int, int) {
        any = true;
        return btScalar(0);
    }
};

bool PhysicsEngine::BodyInContact(Body *body) {
    AnyContactCallback cb;
    dynamicsWorld->contactTest(body->btBody, cb);
    return cb.any;
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

void RegisterPhysicsBody(Body *body, glm::vec3 pos, glm::vec3 rot)
{
    physics->RegisterObject(body, pos, rot);
}


void ApplyCentralForce(Body *body, glm::dvec3 force) {
    getRigidBody(body)->applyCentralForce(btVector3(force.x, force.y, force.z));
}

void ApplyForce(Body *body, glm::dvec3 rel, glm::dvec3 force) {
    // Bullet's signature is applyForce(force, rel_pos); the old body had
    // them swapped (never called, so it went unnoticed).
    getRigidBody(body)->applyForce(btVector3(force.x, force.y, force.z),
                                   btVector3(rel.x, rel.y, rel.z));
}

void ApplyTorque(Body *body, glm::dvec3 torque) {
    getRigidBody(body)->applyTorque(btVector3(torque.x, torque.y, torque.z));
}

/* The shape's inertia diagonal at the Body's current mass. Read from the
   SHAPE, not from a rigid body's stored props: a ship part has no rigid body
   (see Body::btBody). The per-kilogram figure is cached on the Body -- see
   Body::inertiaPerKg for why that is exact and not an approximation.

   Vehicle::checkCompoundInvariants compares this against the child inertias
   Bullet computes itself inside calculatePrincipalAxisTransform (which calls
   calculateLocalInertia(mass) directly, uncached), so the linearity the cache
   rests on is re-verified on every ship at every build, staging event and
   burn-triggered refresh -- in the unit tests and in the game. */
glm::dvec3 getInertiaDiag(Body *body) {
    if(body->mass == 0.0 || body->shape == nullptr) {
        return glm::dvec3(1.0, 1.0, 1.0);   // RegisterObject's placeholder
    }
    if(!body->inertiaCached) {
        btVector3 i(0, 0, 0);
        body->shape->calculateLocalInertia(1.0, i);
        body->inertiaPerKg = glm::dvec3(i.getX(), i.getY(), i.getZ());
        body->inertiaCached = true;
    }
    return body->inertiaPerKg * body->mass;
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

void SetFriction(Body *b, double f) {
    getRigidBody(b)->setFriction((btScalar)f);
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

    // proceedToTransform zeroes both velocities -- right for rails
    // handoffs, a trap for live bodies (it killed the kerbal's walk when
    // used to overwrite its standing attitude per substep).
    getRigidBody(b)->proceedToTransform(t);
}



// TODO this seems like it would be useful
// double angleFacing(Body *body, glm::dvec3 dir) {
//   return getRelAxis(body, 2).angle(btVector3(dir.x, dir.y, dir.z));
// }





bool BodyInContact(Body *body) {
    return physics->BodyInContact(body);
}





void NeverSleep(Body *body) {
    getRigidBody(body)->setSleepingThresholds(0.0, 0.0);
}
