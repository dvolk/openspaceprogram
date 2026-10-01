#pragma once

// The Bullet types below are forward-declared only: this header must stay
// free of Bullet includes. Complete types come from btcommon.h.
class btRigidBody;
class btCollisionShape;

#include "camera.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"

/* Per-draw appearance overrides for the physics-free authoring path (VAB
   ghost / selection highlight / palette preview). */
struct DrawOpts {
    float alpha = 1.0f;                 // 1 = opaque, <1 = translucent ghost
    glm::vec3 tint = glm::vec3(1.0f);   // multiplies the lit color (selection)
    float flat = 0.0f;                  // 1 = uniform studio light (the VAB look)
};

/* Draw one part's render assets at an explicit model matrix -- NO rigid body
   required. xform: extra world transform applied before the camera view (a
   body's rigid-body coordinates live in whatever frame it was integrated in;
   when that is not the frame the view is built in, the caller passes the
   body-frame -> render-frame transform here). */
void DrawModelAt(const Camera *camera, Mesh *mesh, Shader *shader, Texture *texture,
                 const glm::dmat4 &modelMat, glm::vec3 &sunlightVec, float shadow,
                 const glm::dmat4 &xform = glm::dmat4(1.0),
                 const DrawOpts &opts = DrawOpts());

struct Body {
    /* The render assets are SHARED (the get_mesh/get_texture registries own
       them) -- ~Body must not free them. The hull body leaves them null. */
    Mesh *mesh = nullptr;
    Shader *shader = nullptr;
    Texture *texture = nullptr;

    /* collision convex-hull margin (m); -1 = not set -> physics default
       (OSP_HULL_MARGIN / 0.1). */
    double hull_margin = -1.0;

    /* The rigid body, or null. A ship PART has no rigid body of its own --
       the ship is one body (Vehicle::hull) and the part is a child of its
       compound shape. Bodies that ARE simulated (a space pad) have one. */
    btRigidBody *btBody = nullptr;

    /* The collision shape -- the convex hull of the part mesh. Owned HERE,
       not by the rigid body (Bullet's btRigidBody never owned its shape).
       Freed after btBody, which points at it. */
    btCollisionShape *shape = nullptr;

    /* The part's collision hull's VERTICES (part-local frame), captured once
       at build time from body->shape. The drag area facing the flow is
       projectedArea(hullVerts, v̂). Storing the HULL's vertices (not the
       mesh's triangles) keeps the silhouette exact for a non-convex mesh. */
    std::vector<glm::dvec3> hullVerts;

    /* The shape's inertia diagonal per kilogram. A fixed shape's inertia is
       exactly LINEAR in its mass (btPolyhedralConvexShape::calculateLocalInertia
       is (mass/12)*(ly^2+lz^2, ...) over the AABB), so computed once. */
    glm::dvec3 inertiaPerKg = glm::dvec3(0.0);
    bool inertiaCached = false;

    double mass;

    glm::dmat4 model_matrix = glm::dmat4(1.0);

    ~Body();   // frees btBody/shape (physics.cpp: needs the complete
               // Bullet types; ordering constraints live there)

    /* The pose to draw at, read off this body's own rigid body. Right for a
       space pad. A ship part's pose is derived from the hull instead --
       Vehicle::Draw passes the matrix in through DrawAt. */
    void UpdateModelMatrix();

    /* xform: extra world transform applied before the camera view (identity
       by default). See DrawModelAt. */
    void Draw(const Camera* camera, glm::vec3 & sunlightVec, float shadow,
              const glm::dmat4 &xform = glm::dmat4(1.0)) {
        UpdateModelMatrix();
        DrawAt(camera, sunlightVec, shadow, model_matrix, xform);
    }

    /* Draw at an explicit model matrix, leaving model_matrix untouched. */
    void DrawAt(const Camera* camera, glm::vec3 & sunlightVec, float shadow,
                const glm::dmat4 &modelMat,
                const glm::dmat4 &xform = glm::dmat4(1.0)) {
        DrawModelAt(camera, mesh, shader, texture, modelMat, sunlightVec,
                    shadow, xform);
    }
};

void RegisterPhysicsBody(Body *body, glm::vec3 pos, glm::vec3 rot);
/* Build body->shape (the convex hull of its mesh) without a rigid body. */
void BuildPartHull(Body *body);
/* Pointer accessors (no complete Bullet type needed to assign or read). */
void setRigidBody(Body *b, btRigidBody *rb);
btRigidBody *getRigidBody(Body *b);
/* Read body->shape's hull vertices into body->hullVerts (the projected-area
   drag silhouette). Defined in physics.cpp (needs the complete shape type). */
void captureHullVerts(Body *body);

Body *create_body(Mesh *mesh, Shader *shader, Texture *texture,
                  float x, float y, float z, float mass);

/* A ship part's Body: shared render assets + collision hull + mass, and
   NO rigid body -- the part is a child of the ship's compound. */
Body *create_part_body(Mesh *mesh, Shader *shader, Texture *texture,
                       float mass, double hull_margin);
