#pragma once

struct Texture {
    ~Texture();

    unsigned int id; /* really GLuint */
};

/* mipmap=false keeps a single level (no alpha-edge bleed from mip chains);
   use it for the flat billboard icons. */
Texture * load_texture(const char *filename, bool mipmap = true);

/* CPU-generated RGBA8 texture (the porkchop heatmap; the surface map):
   rgba is w*h pixels of [R,G,B,A], row 0 = bottom (GL convention).
   linear=false (default): NEAREST filtering (crisp heatmap cells, no
   wrap bleed); linear=true: LINEAR (a smooth map upscaled over the
   window). CLAMP_TO_EDGE either way. */
Texture * make_texture_r8(int w, int h, const unsigned char *rgba,
                          bool linear = false);
/* Re-upload new pixels to an existing make_texture_r8 texture (same w/h). */
void upload_texture_r8(Texture *tex, int w, int h, const unsigned char *rgba);

/* Highest anisotropic filtering ratio the driver supports (0 = unsupported);
   set GL_TEXTURE_MAX_ANISOTROPY to this value on minifying textures. */
float max_anisotropy();
