#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "model.h"
#include "camera.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"

struct Body {
    // mesh + shader
    Model *model;

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

    double mass;

    glm::dmat4 model_matrix = glm::dmat4(1.0);

    ~Body() {
        delete model;
        delete btBody;
        delete shape;
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

    /* Draw at an explicit model matrix, leaving model_matrix untouched.
       (modelMat, not model: `model` is the mesh+shader member.) */
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

        model->shader->Bind();
        model->shader->setUniform_mat4(0, MVP);
        model->shader->setUniform_mat4(1, ModelFloat);
        model->shader->setUniform_vec3(2, sunlightVec);
        model->shader->setUniform_vec1(3, shadow);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, model->texture->id);

        model->mesh->Draw();

        glBindTexture(GL_TEXTURE_2D, 0);
    }
};

void RegisterPhysicsBody(Body *body, glm::vec3 pos, glm::vec3 rot);
/* Build body->shape (the convex hull of its model) without a rigid body. */
void BuildPartHull(Body *body);

Body *create_body(Model *model, float x, float y, float z, float mass);

/* A ship part's Body: model + collision hull + mass, and NO rigid body --
   the part is a child of the ship's compound, not a simulated object of its
   own (see Body::btBody). Nothing is registered in the world. */
Body *create_part_body(Model *model, float mass);
