#include <glm/glm.hpp>

#include "mesh.h"   // complete type: ~Billboard deletes the quad

class Camera;
class Shader;
class Texture;

struct Billboard {
    ~Billboard() { delete mesh; }   // owns its quad (texture is registry-shared)

    Texture *texture;
    Shader *shader;
    Mesh *mesh;
    glm::vec4 color;
    // A DIRECTION, not a place: Draw stands the quad 10 m from the eye along
    // this vector, which is what makes every marker the same size on screen.
    // (A `Frame *` here used to be read by nothing; a world-placed marker wants
    // a point and its own orientation, not a frame pointer on a billboard.)
    glm::dvec3 pos;

    void Draw(const Camera *camera, double angle);
};

Billboard *mk_billboard(Shader *shader, Texture *texture, float sizex, float sizey, glm::vec4 color);
