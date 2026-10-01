#pragma once

#include <string>

struct Texture {
    ~Texture();

    unsigned int id; /* really GLuint */
};

/* Shared file-asset registry: ONE GL texture per (file, mipmap) pair.
   mipmap=false for flat billboard icons (no alpha-edge bleed). A failed
   load yields a hot-pink placeholder instead of NULL. */
Texture *get_texture(const std::string &path, bool mipmap = true);

/* CPU-generated RGBA8 texture (heatmap / surface map). linear=false: NEAREST
   (crisp cells); linear=true: LINEAR (smooth map). CLAMP_TO_EDGE. */
Texture * make_texture_r8(int w, int h, const unsigned char *rgba,
                          bool linear = false);
/* Re-upload new pixels to an existing make_texture_r8 texture (same w/h). */
void upload_texture_r8(Texture *tex, int w, int h, const unsigned char *rgba);

/* CPU-generated R8 texture WITH a mip chain (the cloud deck's coverage map).
   wrap_s = GL_REPEAT when the UV scrolls (the deck drift). */
Texture * make_coverage_texture(int w, int h, const unsigned char *r,
                                bool wrap_s);
/* Re-upload the grid to an existing make_coverage_texture texture. */
void upload_coverage_r8(Texture *tex, int w, int h, const unsigned char *r);

/* Highest anisotropic filtering ratio the driver supports (0 = unsupported). */
float max_anisotropy();
