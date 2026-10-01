#include "texture.h"

#include "resdir.h"

#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include <GL/glew.h>
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

Texture::~Texture() {
    glDeleteTextures(1, &id);
}

float max_anisotropy() {
    float max_aniso = 0.0f;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY, &max_aniso);
    return max_aniso;
}

static Texture *load_texture_file(const char *filename, bool mipmap) {
    SDL_Surface* res_texture = IMG_Load(filename);
    if (res_texture == NULL) {
        return NULL;
    }
    Texture * ret = new Texture;

    // SDL3: 32-bit names are inverted from SDL2 on little-endian. The
    // [R,G,B,A] byte order GL_RGBA reads is SDL_PIXELFORMAT_ABGR8888;
    // RGBA8888 would reverse the channels.
    SDL_Surface* glSurface = SDL_ConvertSurface(res_texture, SDL_PIXELFORMAT_ABGR8888);
    SDL_DestroySurface(res_texture);

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
    SDL_DestroySurface(glSurface);

    if (mipmap) {
        // Mipmap chain + trilinear minify (anisotropy needs the chain).
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    return ret;
}

/* --- the shared file-asset registry: one GL texture per (file, mipmap) pair. --- */
static std::map<std::string, Texture *> s_textures;

Texture *get_texture(const std::string &path, bool mipmap) {
    // Cache on the resolved path so "res/x" and an absolute hit one slot.
    const std::string file = resdir::path(path);
    const std::string key = file + (mipmap ? "#mip" : "#nomip");
    std::map<std::string, Texture *>::iterator it = s_textures.find(key);
    if(it != s_textures.end()) { return it->second; }
    Texture *tex = load_texture_file(file.c_str(), mipmap);
    if(tex == nullptr) {
        // Hot pink placeholder (cached so a missing file does not re-attempt).
        printf("get_texture: could not load '%s' -- using the hot-pink placeholder\n",
               path.c_str());
        const int w = 16, h = 16;
        std::vector<unsigned char> px((size_t)w * h * 4);
        for(size_t i = 0; i < px.size(); i += 4) {
            px[i + 0] = 255; px[i + 1] = 105; px[i + 2] = 180; px[i + 3] = 255;
        }
        tex = make_texture_r8(w, h, px.data(), false);
    }
    s_textures[key] = tex;
    return tex;
}

Texture *make_texture_r8(int w, int h, const unsigned char *rgba,
                         bool linear) {
    Texture *ret = new Texture;
    glGenTextures(1, &ret->id);
    glBindTexture(GL_TEXTURE_2D, ret->id);
    // NEAREST for discrete heatmap cells; linear=true for smooth content.
    const GLint filt = linear ? GL_LINEAR : GL_NEAREST;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filt);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filt);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    return ret;
}

void upload_texture_r8(Texture *tex, int w, int h, const unsigned char *rgba) {
    glBindTexture(GL_TEXTURE_2D, tex->id);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
}

Texture *make_coverage_texture(int w, int h, const unsigned char *r,
                               bool wrap_s) {
    Texture *ret = new Texture;
    glGenTextures(1, &ret->id);
    glBindTexture(GL_TEXTURE_2D, ret->id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                    wrap_s ? GL_REPEAT : GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, r);
    // Mip chain + trilinear minify (the deck minifies at distance / grazing angles).
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    float aniso = max_anisotropy();
    if (aniso > 0.0f) {
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, aniso);
    }
    return ret;
}

void upload_coverage_r8(Texture *tex, int w, int h, const unsigned char *r) {
    glBindTexture(GL_TEXTURE_2D, tex->id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, r);
    // Rebuild the chain for the new size (the placeholder's was trivial).
    glGenerateMipmap(GL_TEXTURE_2D);
}
