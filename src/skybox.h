#pragma once

struct Camera;
struct Shader;

struct Skybox {
  ~Skybox();

  void init(void);
  void Draw(const Camera * camera, Shader * shader);
};
