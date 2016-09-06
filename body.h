#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "model.h"

struct Body {
    // mesh + shader
    Model *model;

    // color of the body
    glm::vec4 color;

    // bullet object, stores all the physical body information
    btRigidBody *btBody;

    double mass;

    glm::dmat4 model_matrix;

    // model matrix received from bullet for drawing
    void UpdateModelMatrix() {
     btBody->getCenterOfMassTransform().getOpenGLMatrix(&model_matrix[0][0]);
    }

    void Draw(const Camera& camera) {
        UpdateModelMatrix();
        model->shader->Bind();
        model->shader->Update(model_matrix, color, camera);
        model->mesh->Draw();

        /* glUseProgram(0); */
        /* draw_line(glm::vec3(0, 0, 0), */
        /*           glm::vec3(10, 10, 10), */
        /*           glm::vec3(1.0, 1.0, 1)); */
    }
};

void RegisterPhysicsBody(Body *body, glm::vec3 pos,
                         glm::vec3 rot, bool planet);

Body *create_body(Model *model, float x, float y, float z,
                  float mass, bool planet);
