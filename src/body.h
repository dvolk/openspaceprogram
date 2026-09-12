#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "camera.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"

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
        glm::dmat4 View = camera->GetView();
        // The view is built in the render frame (origin = renderOrigin),
        // so shift the geometry into that frame before the float32 cast.
        const glm::dmat4 xf = glm::translate(-camera->GetRenderOrigin()) * xform;
        // make sure View * Model happens with double precision
        glm::dmat4 ModelView = View * xf * modelMat;
        glm::mat4 ModelViewFloat = ModelView;
        glm::mat4 Projection = camera->GetProjection();
        glm::mat4 MVP = Projection * ModelViewFloat;
        glm::mat4 ModelFloat = xf * modelMat;

        shader->Bind();
        shader->setUniform_mat4(0, MVP);
        shader->setUniform_mat4(1, ModelFloat);
        shader->setUniform_vec3(2, sunlightVec);
        shader->setUniform_vec1(3, shadow);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture->id);

        mesh->Draw();

        glBindTexture(GL_TEXTURE_2D, 0);
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
