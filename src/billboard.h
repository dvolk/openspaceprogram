#include <glm/glm.hpp>

class Camera;
class Frame;
class Shader;
class Mesh;

struct Billboard {
  Frame *frame;
  Shader *shader;
  Mesh *mesh;
  glm::dmat4 model;

  void Draw(const Camera *camera);
};

Billboard *mk_billboard(Frame *frame, Shader *shader, glm::dvec3 pos);
