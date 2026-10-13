#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

struct Camera;
struct Shader;

// The star field. The unit-cube geometry is system-independent; the cubemap
// belongs to the running system, so a system switch reloads it.
struct Skybox {
    Skybox() = default;
    // Owns GL handles: a copy would double-delete them.
    Skybox(const Skybox &) = delete;
    Skybox &operator=(const Skybox &) = delete;

    ~Skybox();

    void init(void);   // the unit-cube VAO (needs a GL context)

    // `faces` is six image names in GL cubemap order (+X,-X,+Y,-Y,+Z,-Z). A
    // face that fails to load is reported and leaves the current cubemap in
    // place: the names come from hand-editable JSON.
    void load(const std::vector<std::string> &faces);

    // `gain` fades the star field (1.0 = as authored); render.cpp drives it
    // from the sun's position in view.
    void Draw(const Camera * camera, Shader * shader, const glm::dmat3 &skyRot,
              float gain);

    unsigned int vao = 0, vbo = 0, cubemap = 0;   // really GLuint (texture.h)
};
