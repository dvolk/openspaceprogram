#pragma once

class Mesh;
class Shader;
class Texture;

struct Model {
    Mesh *mesh;
    Shader *shader;
    Texture *texture;

    /* collision convex-hull margin (m); -1 = not set -> the physics
       engine uses its default (OSP_HULL_MARGIN / 0.1). Set from the
       part catalog entry when a part body is built. */
    double hull_margin = -1.0;

    ~Model();

    void FromData(Mesh *mesh, Shader *shader, Texture *texture);
};

