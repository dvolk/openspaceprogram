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
  glm::dmat4 model;

  void Draw(const Camera *camera);
};

Billboard *mk_billboard(Shader *shader, Texture *texture, float size);
