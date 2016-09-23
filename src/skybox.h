#pragma once

struct Texture;
struct Camera;
struct Shader;

struct Skybox {
  Texture *textures[6];

  void init(void);
  void Draw(const Camera * camera, Shader * shader);
};
