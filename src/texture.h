#pragma once

struct Texture {
  ~Texture();

  unsigned int id; /* really GLuint */
};

Texture * load_texture(const char *filename);
