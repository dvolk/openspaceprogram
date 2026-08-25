#include "texture.h"

#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <SDL_image.h>

Texture::~Texture() {
    glDeleteTextures(1, &id);
}

float max_anisotropy() {
    float max_aniso = 0.0f;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY, &max_aniso);
    return max_aniso;
}

Texture *load_texture(const char *filename) {
    Texture * ret = new Texture;
  
    SDL_Surface* res_texture = IMG_Load(filename);
    if (res_texture == NULL) {
        return NULL;
    }

    SDL_PixelFormat pf;
    pf.palette = 0;
    pf.BitsPerPixel = 32;
    pf.BytesPerPixel = 4;
    // pf.alpha = 255;
    pf.Rshift = pf.Rloss = pf.Gloss = pf.Bloss = pf.Aloss; // = pf.colorkey = 0;
    pf.Rmask = 0x000000ff;
    pf.Gshift = 8;
    pf.Gmask = 0x0000ff00;
    pf.Bshift = 16;
    pf.Bmask = 0x00ff0000;
    pf.Ashift = 24;
    pf.Amask = 0xff000000;
    SDL_Surface* glSurface = SDL_ConvertSurface(res_texture, &pf, SDL_SWSURFACE);
    SDL_FreeSurface(res_texture);

    glGenTextures(1, &ret->id);
    glBindTexture(GL_TEXTURE_2D, ret->id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    float aniso = max_anisotropy();
    if (aniso > 0.0f) {
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, aniso);
        static bool aniso_printed = false;
        if (!aniso_printed) {
            printf("texture anisotropic filtering: %g (driver max)\n", aniso);
            aniso_printed = true;
        }
    }
    glTexImage2D(GL_TEXTURE_2D, // target
                 0,  // level, 0 = base; the mip chain is generated below
                 GL_RGBA, // internalformat
                 glSurface->w,  // width
                 glSurface->h,  // height
                 0,  // border, always 0 in OpenGL ES
                 GL_RGBA,  // format
                 GL_UNSIGNED_BYTE, // type
                 glSurface->pixels);
    SDL_FreeSurface(glSurface);

    // Mipmap chain + trilinear minify: without a chain the anisotropy ratio
    // above has nothing to interpolate between, and minified parts shimmer.
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    return ret;
}
