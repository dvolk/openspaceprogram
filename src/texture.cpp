#include "texture.h"

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

    // SDL3: the hand-built SDL_PixelFormat struct is gone; the format is an
    // enum. Gotcha -- SDL3's 32-bit names are inverted from SDL2 on
    // little-endian: the [R,G,B,A] byte order (what GL_RGBA below reads, R
    // first) is SDL_PIXELFORMAT_ABGR8888, while SDL_PIXELFORMAT_RGBA8888 is
    // [A,R,G,B]. Converting to RGBA8888 therefore reverses the channels
    // (gray parts render red, plume black->red, alpha->R). Target ABGR8888
    // (a no-op for the RGBA PNGs and a clean expand for RGB-only ones).
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
        // Mipmap chain + trilinear minify: without a chain the anisotropy
        // ratio above has nothing to interpolate between, and minified parts
        // shimmer.
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    return ret;
}

/* --- the shared file-asset registry -------------------------------------
   One GL texture per (file, mipmap) pair, shared by every part/pad that
   uses the file: a 100-part ship built from K part types uploads K
   textures, not 100. The map lives until process exit; the GL context
   teardown reclaims the objects, so there is no cleanup pass. All calls
   are main-thread (the job worker does pure math only), so no lock. */
static std::map<std::string, Texture *> s_textures;

/* "res/x.png" and "./res/x.png" must land in one cache slot. */
static std::string asset_key(const std::string &path) {
    if(path.compare(0, 2, "./") == 0) { return path.substr(2); }
    return path;
}

Texture *get_texture(const std::string &path, bool mipmap) {
    std::string key = asset_key(path) + (mipmap ? "#mip" : "#nomip");
    std::map<std::string, Texture *>::iterator it = s_textures.find(key);
    if(it != s_textures.end()) { return it->second; }
    Texture *tex = load_texture_file(path.c_str(), mipmap);
    if(tex == nullptr) {
        // Hot pink: a part with a broken texture still renders, visibly
        // wrong. Cache the placeholder too, so a missing file does not
        // re-attempt the load (and re-print) on every part built from it.
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
    // NEAREST: a heatmap is discrete cells; LINEAR would smear them into a
    // fake smooth gradient when the image is upscaled. linear=true is for
    // smooth content (the surface map) where the opposite is wanted.
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
    // Mip chain + trilinear minify: the deck minifies at distance / grazing
    // angles (the rim in orbit), where a single level shimmers; anisotropy
    // needs the chain to work on.
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
