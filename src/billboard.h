#include <glm/glm.hpp>

class Camera;
class Frame;
class Shader;
class Mesh;
class Texture;

struct Billboard {
  Frame *frame;
  Texture *texture;
  Shader *shader;
  Mesh *mesh;
  glm::vec4 color;
  glm::dvec3 pos;

  void Draw(const Camera *camera, double angle);
};

Billboard *mk_billboard(Shader *shader, Texture *texture, float sizex, float sizey, glm::vec4 color);
