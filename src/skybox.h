#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

struct Camera;
struct Shader;

// The star field. The geometry (a unit cube) is system-independent; the
// cubemap belongs to the running system (System::skybox_faces), so a system
// switch reloads it.
struct Skybox {
    Skybox() = default;
    // Owns GL handles: a copy would double-delete them.
    Skybox(const Skybox &) = delete;
    Skybox &operator=(const Skybox &) = delete;

    ~Skybox();

    void init(void);   // the unit-cube VAO (needs a GL context)

    // `faces` is six image names in GL cubemap order (+X,-X,+Y,-Y,+Z,-Z).
    // A face that will not load (missing, undecodable, wrong shape) is
    // reported and leaves the current cubemap in place: the names come from
    // a hand-editable system JSON, so a bad sky is data, not an invariant.
    void load(const std::vector<std::string> &faces);

    void Draw(const Camera * camera, Shader * shader, const glm::dmat3 &skyRot);

    unsigned int vao = 0, vbo = 0, cubemap = 0;   // really GLuint (texture.h)
};
