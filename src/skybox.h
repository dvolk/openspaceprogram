#pragma once

#include <glm/glm.hpp>

struct Camera;
struct Shader;

struct Skybox {
    ~Skybox();

    void init(void);
    void Draw(const Camera * camera, Shader * shader, const glm::dmat3 &skyRot);
};
