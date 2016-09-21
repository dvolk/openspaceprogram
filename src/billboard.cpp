#include "billboard.h"

#include "camera.h"
#include "frame.h"
#include "shader.h"
#include "mesh.h"

Mesh *make_quad_mesh(float size) {
  PosVertex v[4] = { PosVertex(-size/2, -size/2, 0),
		     PosVertex( size/2, -size/2, 0),
		     PosVertex(-size/2,  size/2, 0),
		     PosVertex( size/2,  size/2, 0)
  };

  int indices[6] = { 0, 1, 2,
		     2, 1, 3
  };

  PosNorIndColInterface interface;

  for(int i = 0; i < 4; i++) {
    interface.positions.push_back(v[i].pos);
    interface.normals.push_back(glm::vec3(0, 0, 1));
    interface.colors.push_back(glm::vec3(1, 1, 0));
  }

  for(int i = 0; i < 6; i++) {
    interface.indices.push_back(indices[i]);
  }

  Mesh *m = new Mesh;
  m->InitMesh(interface, false);

  return m;
}

Billboard *mk_billboard(Frame *frame, Shader *shader, glm::dvec3 pos) {
  Billboard *b = new Billboard;
  b->model = translate(pos);
  b->frame = frame;
  b->shader = shader;
  b->mesh = make_quad_mesh(12000000);
  // b->mesh = make_quad_mesh(261600000);
  return b;
}

void Billboard::Draw(const Camera * camera) {
  glm::dmat4 View = camera->GetView();

  // const glm::dmat4 & View = camera->GetView();
  glm::dmat4 inv_rot = glm::dmat4(transpose(glm::mat3(View)));
  glm::dvec3 pos = glm::dvec3(model[3]);
  // // printf("drawing at %.0f, %.0f, %.0f\n", pos.x, pos.y, pos.z);
  model = glm::translate(pos) * inv_rot;
  glm::dmat4 ModelView = View * model;
  // ModelView = glm::dmat4(glm::dmat3(ModelView));

  const glm::mat4 & Projection = camera->GetProjection();
  glm::mat4 MVP = Projection * glm::mat4(ModelView);

  shader->Bind();
  shader->setUniform_mat4(0, MVP);
  // shader->setUniform_mat4(1, ModelFloat);
  mesh->Draw();
}
