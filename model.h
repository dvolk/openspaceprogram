#pragma once

#include "mesh.h"
#include "shader.h"

struct Model {
    Mesh *mesh;
    Shader *shader;

    void FromData(Mesh *mesh, Shader *shader) {
        this->mesh = mesh;
        this->shader = shader;
    }

  ~Model() {
    delete mesh;
  }
};
