#include <glm/glm.hpp>

#include "mesh.h"   // complete type: ~Billboard deletes the quad

class Camera;
class Frame;
class Shader;
class Texture;

struct Billboard {
    ~Billboard() { delete mesh; }   // owns its procedural quad (the texture
                                    // is registry-shared)

    Frame *frame;
    Texture *texture;
    Shader *shader;
    Mesh *mesh;
    glm::vec4 color;
    glm::dvec3 pos;

    void Draw(const Camera *camera, double angle);
};

Billboard *mk_billboard(Shader *shader, Texture *texture, float sizex, float sizey, glm::vec4 color);
