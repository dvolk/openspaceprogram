#include "btcommon.h"
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <iostream>
#include <cstdlib>
#include <algorithm>

#include "physics.h"
#include "body.h"
#include "mesh.h"
#include "camera.h"
#include "shader.h"
#include "gldebug.h"

PhysicsEngine *physics;

/* Collision margins (m). A hull and what it rests on each carry one, and the two
   SUM to the visible gap -- anything placing a vessel on a floor must read these,
   not bake a constant (build_ship's lift, Kerbal::restAlt). The margin also
   reaches mass properties: a convex hull inherits a BOX inertia over the
   margin-inflated AABB, and calculateLocalInertia adds the margin a SECOND time. */
static double envMargin(const char *name, double fallback) {
    const char *e = getenv(name);
    // A negative margin inverts the shape's AABB; the JSON paths reject it, so
    // the env knobs reject it too.
    if(e && e[0]) { return std::max(0.0, strtod(e, NULL)); }
    return fallback;
}

double hullMargin() { return envMargin("OSP_HULL_MARGIN", 0.1); }

double terrainMargin() { return envMargin("OSP_TERRAIN_MARGIN", 0.5); }

double effectiveHullMargin(const Body *body) {
    return (body->hull_margin >= 0.0) ? body->hull_margin : hullMargin();
}

class GLDebugDrawer : public btIDebugDraw {
    int m_debugMode;
    Shader *lineshader;
    GLuint m_vao;
    GLuint m_bufs[1];

public:
    std::vector<float> lineBuffer;

    /* Line vertices are stored RELATIVE to this, subtracted in double in
       drawLine(): Bullet hands drawLine absolute coordinates and the buffer is
       float32, so the subtraction must happen BEFORE the narrowing (float32 at
       1 AU is an ~18 km quantum). */
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

    lineshader->Bind();
    check_gl_error();
    // Vertices are already render-frame relative and the view is built in the
    // render frame: no origin shift belongs in this matrix.
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

    lineshader = get_shader("res/shaders/lineShader", { "pos" }, { "VP" });

    m_debugMode = DBG_DrawWireframe;

    glGenVertexArrays(1, &m_vao);
    check_gl_error();
    glGenBuffers(1, &m_bufs[0]);
    check_gl_error();
}

void GLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
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
    // %zu: size_t is unsigned long on linux, unsigned long long on windows.
    printf("sizeof(btScalar): %zu\n", sizeof(btScalar));
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
    // One substep. The caller must re-apply the forces before EVERY call:
    // stepSimulation clears accumulated forces on exit.
    dynamicsWorld->stepSimulation(timeStep, 1, timeStep);
}

void physics_tick(float timeStep) {
    physics->tick(timeStep);
}

btRigidBody *addTerrainCollision(Mesh *m, const glm::dvec3 &anchor) {
    return physics->AddTerrainCollision(m, anchor);
}

void removeTerrainCollision(btRigidBody *b) {
    physics->RemoveTerrainCollision(b);
}

void PhysicsEngine::RemoveTerrainCollision(btRigidBody *b) {
    dynamicsWorld->removeRigidBody(b);
    // Bullet frees NONE of these (~btRigidBody is a no-op, and neither
    // btBvhTriangleMeshShape nor its base releases the striding interface).
    // Free interface -> shape -> motion state; ~GeoPatch frees the mesh after
    // this, so the pointers stay valid.
    btTriangleMeshShape *t =
        static_cast<btTriangleMeshShape *>(b->getCollisionShape());
    delete t->getMeshInterface();
    delete t;
    delete b->getMotionState();
}

btRigidBody *PhysicsEngine::AddTerrainCollision(Mesh *m,
                                                const glm::dvec3 &anchor) {
    btTransform startTransform;
    startTransform.setIdentity();
    // The patch mesh is baked relative to its anchor (terragen.h GridGeom):
    // placing the body there puts the triangles back in body-frame coords.
    startTransform.setOrigin(btVector3(anchor.x, anchor.y, anchor.z));

    // Terrain-only triangles: the skirt tail (numInnerIndices) is render-only
    // crack filler UNDER the surface -- in the BVH it would be a hidden
    // two-sided collision slab. 0 means no skirt.
    const unsigned int inner = m->numInnerIndices();
    const unsigned int num_tris = (inner != 0) ? inner / 3
                                               : m->num_indices / 3;
    btTriangleIndexVertexArray *mesh_interface
        = new btTriangleIndexVertexArray(num_tris,
                                         m->is,
                                         3*sizeof(int), // grr bytes!
                                         m->num_vertices,
                                         m->vs,
                                         3*sizeof(double));

    btBvhTriangleMeshShape *terrain
        = new btBvhTriangleMeshShape(mesh_interface, true, true);
    terrain->setMargin((btScalar)terrainMargin());

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

    rbInfo.m_friction = (btScalar)kDefaultHullFriction;

    btRigidBody *b = new btRigidBody(rbInfo);

    setRigidBody(body, b);
    dynamicsWorld->addRigidBody(b);
}

/* The convex hull of the body's mesh, stored on the Body (which owns it). */
void PhysicsEngine::BuildHull(Body *body) {
    Mesh *m = body->mesh;

    assert(m->vs != NULL);
    assert(m->num_vertices >= 3);

    /* Bullet has no collision algorithm for concave-vs-concave pairs, so
       anything that moves must stay convex. */
    btConvexHullShape *hull = new btConvexHullShape(m->vs, (int)m->num_vertices,
                                                    3 * sizeof(double));
    // Reduce to extreme vertices: geometrically identical, and projectedArea
    // sorts these every substep.
    hull->optimizeConvexHull();

    /* The body carries the part's resolved margin (ship def > catalog, see
       resolveHullMargin); -1 when neither sets one. */
    const double margin = effectiveHullMargin(body);
    hull->setMargin(margin);
    body->shape = hull;
}

void BuildPartHull(Body *body) {
    physics->BuildHull(body);
}

/* Body's Bullet-side lifecycle, out of body.h so that header stays Bullet-free. */
Body::~Body() {
    if(btBody != nullptr) {
        // Bullet never frees the body's motion state (~btRigidBody is
        // a no-op) -- ours to free before the body that points at it.
        delete btBody->getMotionState();
    }
    delete btBody;
    delete shape;
    /* mesh/shader/texture are shared (the asset registries own them) --
       never freed here. The hull shape copied the mesh's vertices at build
       time, so it holds no pointer into the mesh. */
}

void Body::UpdateModelMatrix() {
    btBody->getCenterOfMassTransform().getOpenGLMatrix(&model_matrix[0][0]);
}

void captureHullVerts(Body *body) {
    /* The collision hull's vertices (part-local frame) for the projected-area
       drag (drag.h projectedArea). Read from body->shape -- the SAME shape the
       collision uses -- so the drag silhouette matches it by construction. */
    if(const btConvexHullShape *hull =
           static_cast<const btConvexHullShape *>(body->shape)) {
        const int n = hull->getNumVertices();
        body->hullVerts.reserve(n);
        for(int i = 0; i < n; i++) {
            btVector3 v;
            hull->getVertex(i, v);
            body->hullVerts.push_back(
                glm::dvec3(v.getX(), v.getY(), v.getZ()));
        }
    }
}

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
    // Bullet's signature is applyForce(force, rel_pos).
    getRigidBody(body)->applyForce(btVector3(force.x, force.y, force.z),
                                   btVector3(rel.x, rel.y, rel.z));
}

void ApplyTorque(Body *body, glm::dvec3 torque) {
    getRigidBody(body)->applyTorque(btVector3(torque.x, torque.y, torque.z));
}

/* The shape's inertia diagonal at the Body's current mass. Read from the SHAPE,
   not a rigid body's stored props: a ship part has no rigid body. The
   per-kilogram figure is cached on the Body (Body::inertiaPerKg). */
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
    // Through a quaternion: Bullet's basis matrix is row-major (m_el[i] = row i)
    // and glm::dmat3 is column-major, so a direct element copy silently
    // transposed the orientation.
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

    glm::dquat gq = glm::quat_cast(rot);
    // Bullet's quaternion constructor is (x, y, z, w); GLM components are by name.
    t.setRotation(btQuaternion(gq.x, gq.y, gq.z, gq.w));

    // proceedToTransform is ONLY setCenterOfMassTransform: it teleports the pose
    // and leaves BOTH velocities alone. Callers wanting them cleared must do it
    // themselves.
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
