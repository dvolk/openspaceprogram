#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "camera.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"

/* Per-draw appearance overrides for the physics-free authoring path (VAB
   ghost / selection highlight / palette preview). Defaults reproduce today's
   plain opaque part. */
struct DrawOpts {
    float alpha = 1.0f;                 // 1 = opaque, <1 = translucent ghost
    glm::vec3 tint = glm::vec3(1.0f);   // multiplies the lit color (selection)
    float flat = 0.0f;                  // 1 = uniform studio light (the VAB
                                        // look), 0 = the scene's directional
};

/* Draw one part's render assets (mesh + shader + texture) at an explicit
   model matrix -- NO rigid body required. This is the physics-free draw path
   the VAB authoring preview, ghost and palette use; Body::DrawAt is a thin
   wrapper around it for bodies that do carry one.
   xform: extra world transform applied before the camera view (a body's
   rigid-body coordinates live in whatever frame it was integrated in; when
   that is not the frame the view is built in, the caller passes the
   body-frame -> render-frame transform here).
   When opts.alpha < 1 the draw enables alpha blending and disables depth
   write for the translucent pass, restoring both afterwards. */
void DrawModelAt(const Camera *camera, Mesh *mesh, Shader *shader, Texture *texture,
                 const glm::dmat4 &modelMat, glm::vec3 &sunlightVec, float shadow,
                 const glm::dmat4 &xform = glm::dmat4(1.0),
                 const DrawOpts &opts = DrawOpts());

struct Body {
    /* The render assets: SHARED (the get_mesh/get_texture registries own
       them, see mesh.h/texture.h), so ~Body must not free them -- every
       part of a ship that uses one part type draws the SAME mesh and
       texture. The hull body (Vehicle::hull) leaves them null: the ship
       is drawn part by part. */
    Mesh *mesh = nullptr;
    Shader *shader = nullptr;
    Texture *texture = nullptr;

    /* collision convex-hull margin (m); -1 = not set -> the physics
       engine uses its default (OSP_HULL_MARGIN / 0.1). Set from the
       part catalog entry when a part body is built. */
    double hull_margin = -1.0;

    /* The rigid body, or null. A ship PART has no rigid body of its own --
       the ship is one body (Vehicle::hull) and the part is a child of its
       compound shape -- so this stays null for parts and any leftover
       per-part physics call crashes immediately instead of silently reading
       a transform nothing integrates. Bodies that ARE simulated (a space
       pad) have one. */
    btRigidBody *btBody = nullptr;

    /* The collision shape -- the convex hull of the part mesh. Owned HERE,
       not by the rigid body: Bullet's btRigidBody never owned its shape (it
       leaked one per part), and the hull has to outlive registration anyway,
       because the ship's compound references it as a child and picking casts
       against it. Freed after btBody, which points at it. */
    btCollisionShape *shape = nullptr;

    /* The part's collision hull's VERTICES (part-local frame), captured once
       at build time from body->shape (the same btConvexHullShape the
       collision uses). The hull is reduced to its EXTREME points at build
       (optimizeConvexHull, physics.cpp BuildHull), so this is tens of
       verts, not the mesh's full 192-640 -- same hull, same silhouette.
       The drag area facing the flow is
       projectedArea(hullVerts, v̂) -- the body's silhouette (drag.h), so a
       part drags more as it turns broadside to the velocity (Phase 2 of the
       projected-drag work, reports/projected-drag). Storing the HULL's
       vertices (not the mesh's triangles) is what keeps the silhouette exact
       for a non-convex mesh (the engine's hollow nozzle) and makes the drag
       area match the collision shape by construction. Empty for a body with
       no hull -- projectedArea of < 3 vertices is 0. */
    std::vector<glm::dvec3> hullVerts;

    /* The shape's inertia diagonal per kilogram, and whether it has been
       worked out yet. A fixed shape's inertia is exactly LINEAR in its mass
       -- Bullet's btPolyhedralConvexShape::calculateLocalInertia, which a
       convex hull inherits, is (mass/12)*(ly^2+lz^2, ...) over the shape's
       AABB -- so it is computed once per hull instead of once per
       rebuildCompound(). That matters because a rebuild walks every part and
       a burn triggers one: a 1000-part ship would otherwise pay an AABB walk
       plus a tensor per part, repeatedly, for a number that only changes when
       the shape or its margin does (neither does at runtime). */
    glm::dvec3 inertiaPerKg = glm::dvec3(0.0);
    bool inertiaCached = false;

    double mass;

    glm::dmat4 model_matrix = glm::dmat4(1.0);

    ~Body() {
        if(btBody != nullptr) {
            // Bullet never frees the body's motion state (~btRigidBody is
            // a no-op) and nothing reads it (static bodies), so it is
            // ours: free it before the body that points at it. The hull
            // body has none (constructed with a null).
            delete btBody->getMotionState();
        }
        delete btBody;
        delete shape;
        /* mesh/shader/texture are shared (the asset registries own them,
           and live until process exit) -- never freed here. The hull
           shape copied the mesh's vertices at build time, so it holds no
           pointer into the mesh. */
    }

    /* The pose to draw at, read off this body's own rigid body. Right for
       anything whose rigid body IS the registered, integrated one (a space
       pad). A ship part is not that any more -- the ship is one body and a
       part's pose is derived from it -- so Vehicle::Draw passes the matrix
       in through DrawAt instead. */
    void UpdateModelMatrix() {
        btBody->getCenterOfMassTransform().getOpenGLMatrix(&model_matrix[0][0]);
    }

    /* xform: extra world transform applied before the camera view
       (identity by default). A body's rigid-body coordinates live in
       WHATEVER reference frame it was integrated in; when that is not
       the frame the camera view is built in (an idle ship that switched
       SOI while another ship is being controlled), the caller passes the
       ship-frame -> render-frame transform here. */
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

Body *create_body(Mesh *mesh, Shader *shader, Texture *texture,
                  float x, float y, float z, float mass);

/* A ship part's Body: shared render assets + collision hull + mass, and
   NO rigid body -- the part is a child of the ship's compound, not a
   simulated object of its own (see Body::btBody). Nothing is registered
   in the world. */
Body *create_part_body(Mesh *mesh, Shader *shader, Texture *texture,
                       float mass, double hull_margin);
