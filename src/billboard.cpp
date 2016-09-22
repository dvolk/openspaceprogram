#include "billboard.h"

#include "camera.h"
#include "frame.h"
#include "shader.h"
#include "mesh.h"
#include "texture.h"

Mesh *make_quad_mesh(float sizex, float sizey) {
  PosVertex v[4] = { PosVertex(-sizex/2, -sizey/2, 0),
		     PosVertex( sizex/2, -sizey/2, 0),
		     PosVertex(-sizex/2,  sizey/2, 0),
		     PosVertex( sizex/2,  sizey/2, 0)
  };

  int indices[6] = { 0, 1, 2,
		     2, 1, 3
  };

  PosTexNorIndColInterface interface;

  for(int i = 0; i < 4; i++) {
    interface.positions.push_back(v[i].pos);
    interface.normals.push_back(glm::vec3(0, 0, 1));
    interface.colors.push_back(glm::vec3(1, 1, 0));
  }

  interface.texcoords.push_back(glm::vec2(0, 0));
  interface.texcoords.push_back(glm::vec2(1, 0));
  interface.texcoords.push_back(glm::vec2(0, 1));
  interface.texcoords.push_back(glm::vec2(1, 1));

  for(int i = 0; i < 6; i++) {
    interface.indices.push_back(indices[i]);
  }

  Mesh *m = new Mesh;
  m->InitMesh(interface, false);

  return m;
}

Billboard *mk_billboard(Shader *shader, Texture *texture, float size) {
  Billboard *b = new Billboard;
  b->shader = shader;
  b->mesh = make_quad_mesh(size, size);
  b->texture = texture;
  return b;
}

void Billboard::Draw(const Camera * camera) {
  glm::dmat4 View = camera->GetView();
  glm::dmat4 _View = glm::dmat4(glm::dmat3(View));

  // const glm::dmat4 & View = camera->GetView();
  glm::dmat4 inv_rot = glm::dmat4(transpose(glm::mat3(View)));
  glm::dvec3 pos = glm::dvec3(model[3]);
  // glm::dvec3 campos = glm::dvec3(View[3]);
  model = glm::translate(10.0 * glm::normalize(pos)) * glm::dmat4(inv_rot);
  glm::dmat4 ModelView = _View * model;
  // pos = glm::dvec3(ModelView[3]);
  // printf("drawing at %.0f, %.0f, %.0f\n", pos.x, pos.y, pos.z);
  // ModelView = glm::dmat4(glm::transpose(glm::dmat3(ModelView)));

  const glm::mat4 & Projection = camera->GetProjection();
  glm::mat4 MVP = Projection * glm::mat4(ModelView);

  shader->Bind();

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture->id);

  shader->setUniform_mat4(0, MVP);
  // shader->setUniform_mat4(1, ModelFloat);
  mesh->Draw();
  glBindTexture(GL_TEXTURE_2D, 0);
}
