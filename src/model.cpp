#include "model.h"

#include "mesh.h"

void Model::FromData(Mesh *mesh, Shader *shader, Texture *texture) {
    this->mesh = mesh;
    this->shader = shader;
    this->texture = texture;
}

Model::~Model() {
    delete mesh;
}
