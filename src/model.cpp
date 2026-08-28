#include "model.h"

#include "mesh.h"
#include "texture.h"

void Model::FromData(Mesh *mesh, Shader *shader, Texture *texture) {
    this->mesh = mesh;
    this->shader = shader;
    this->texture = texture;
}

Model::~Model() {
    delete mesh;
    /* Owns its texture (load_texture hands each model a fresh GL object),
       so deleting a body frees it -- the old code leaked one texture per
       part. The shader stays borrowed (parts share the one partsshader).
       Safe in every delete path: the only bodies that are ever deleted are
       ship parts, and each carries a unique model + texture. */
    delete texture;
}
