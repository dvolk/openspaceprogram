#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "model.h"
#include "camera.h"

struct Body {
  // mesh + shader
  Model *model;

  // color of the body
  // TODO remove this
  glm::vec4 color;

  // bullet object, stores all the physical body information
  btRigidBody *btBody;

  double mass;

  glm::dmat4 model_matrix;

  ~Body() {
    delete model;
    delete btBody;
  }

  // model matrix received from bullet for drawing
  void UpdateModelMatrix() {
    btBody->getCenterOfMassTransform().getOpenGLMatrix(&model_matrix[0][0]);
  }

  void Draw(const Camera* camera, glm::vec3 & sunlightVec) {
    UpdateModelMatrix();
    model->shader->Bind();

    glm::dmat4 View = camera->GetView();
    // make sure View * Model happens with double precision
    glm::dmat4 ModelView = View * model_matrix;
    glm::mat4 ModelViewFloat = ModelView;
    glm::mat4 Projection = camera->GetProjection();
    glm::mat4 MVP = Projection * ModelViewFloat;
    glm::mat4 ModelFloat = model_matrix;

    model->shader->setUniform_mat4(0, MVP);
    model->shader->setUniform_mat4(1, ModelFloat);
    model->shader->setUniform_vec3(2, sunlightVec);
    model->shader->setUniform_vec4(3, color);

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
