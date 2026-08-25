#pragma once

struct Texture {
    ~Texture();

    unsigned int id; /* really GLuint */
};

Texture * load_texture(const char *filename);

/* Highest anisotropic filtering ratio the driver supports (0 = unsupported);
   set GL_TEXTURE_MAX_ANISOTROPY to this value on minifying textures. */
float max_anisotropy();
