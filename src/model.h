#pragma once

class Mesh;
class Shader;
class Texture;

struct Model {
    Mesh *mesh;
    Shader *shader;
    Texture *texture;

    ~Model();

    void FromData(Mesh *mesh, Shader *shader, Texture *texture);
};

