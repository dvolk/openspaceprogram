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

    // bullet object, stores all the physical body information
    btRigidBody *btBody;

    double mass;

    glm::dmat4 model_matrix = glm::dmat4(1.0);

    ~Body() {
        delete model;
        delete btBody;
    }

    // model matrix received from bullet for drawing
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

        glm::dmat4 View = camera->GetView();
        // make sure View * Model happens with double precision
        glm::dmat4 ModelView = View * xform * model_matrix;
        glm::mat4 ModelViewFloat = ModelView;
        glm::mat4 Projection = camera->GetProjection();
        glm::mat4 MVP = Projection * ModelViewFloat;
        glm::mat4 ModelFloat = xform * model_matrix;

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

void RegisterPhysicsBody(Body *body, glm::vec3 pos,
                         glm::vec3 rot, bool planet);

Body *create_body(Model *model, float x, float y, float z,
                  float mass, bool planet);
