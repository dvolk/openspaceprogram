#include "skybox.h"

#include <SDL3_image/SDL_image.h>
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>
#include <GL/glew.h>

#include "camera.h"
#include "resdir.h"
#include "shader.h"
#include "texture.h"

Skybox::~Skybox() {
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteTextures(1, &cubemap);
}

// One face as an RGB24 surface (the caller destroys it), or null with the
// reason printed. The cubemap upload assumes tightly packed RGB24 rows and
// same-size square faces, so normalize and check rather than silently skew
// or sample garbage.
static SDL_Surface *loadFaceRgb24(const std::string &face, int expect_w) {
    SDL_Surface *loaded = IMG_Load(resdir::path(face).c_str());
    if(!loaded) {
        printf("skybox: cannot load face '%s'\n", face.c_str());
        return nullptr;
    }
    SDL_Surface *image = loaded;
    if(image->format != SDL_PIXELFORMAT_RGB24) {
        image = SDL_ConvertSurface(loaded, SDL_PIXELFORMAT_RGB24);
        SDL_DestroySurface(loaded);
        if(!image) {
            printf("skybox: face '%s' failed to convert to RGB24\n", face.c_str());
            return nullptr;
        }
    }
    if(image->w != image->h || (expect_w != 0 && image->w != expect_w)) {
        printf("skybox: face '%s' is %dx%d -- faces must be same-size squares\n",
               face.c_str(), image->w, image->h);
        SDL_DestroySurface(image);
        return nullptr;
    }
    return image;
}

// 0 => a face is unusable (already reported); the caller keeps its sky.
static GLuint loadCubemap(const std::vector<std::string> &faces)
{
    GLuint textureID;
    glGenTextures(1, &textureID);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
    int face_w = 0;
    bool ok = true;
    for(GLuint i = 0; i < faces.size() && ok; i++) {
        SDL_Surface *image = loadFaceRgb24(faces[i], face_w);
        if(image == nullptr) { ok = false; break; }
        face_w = image->w;
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                     0,
                     GL_RGB8,
                     image->w,
                     image->h,
                     0,
                     GL_RGB,
                     GL_UNSIGNED_BYTE,
                     (unsigned char *)image->pixels);
        SDL_DestroySurface(image);
    }
    if(!ok) {
        // The face names live in a hand-editable system JSON, so every
        // failure here is data: report and let the caller keep the sky it
        // has rather than aborting the run over a picture.
        printf("skybox: keeping the current sky\n");
        fflush(stdout);
        glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glDeleteTextures(1, &textureID);
        return 0;
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);   // uploads are done

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    float aniso = max_anisotropy();
    if (aniso > 0.0f) {
        glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_ANISOTROPY, aniso);
    }
    // Mipmap chain + trilinear: calms starfield shimmer at distance.
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

    return textureID;
}

GLfloat skyboxVertices[] = {
    // Positions
    -1.0f,  1.0f, -1.0f,
    -1.0f, -1.0f, -1.0f,
    1.0f, -1.0f, -1.0f,
    1.0f, -1.0f, -1.0f,
    1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,

    -1.0f, -1.0f,  1.0f,
    -1.0f, -1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f,  1.0f,
    -1.0f, -1.0f,  1.0f,

    1.0f, -1.0f, -1.0f,
    1.0f, -1.0f,  1.0f,
    1.0f,  1.0f,  1.0f,
    1.0f,  1.0f,  1.0f,
    1.0f,  1.0f, -1.0f,
    1.0f, -1.0f, -1.0f,

    -1.0f, -1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,
    1.0f,  1.0f,  1.0f,
    1.0f,  1.0f,  1.0f,
    1.0f, -1.0f,  1.0f,
    -1.0f, -1.0f,  1.0f,

    -1.0f,  1.0f, -1.0f,
    1.0f,  1.0f, -1.0f,
    1.0f,  1.0f,  1.0f,
    1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f, -1.0f,

    -1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
    1.0f, -1.0f, -1.0f,
    1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
    1.0f, -1.0f,  1.0f
};

void Skybox::init(void) {
    // Setup skybox VAO
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
    glBindVertexArray(0);
}

void Skybox::load(const std::vector<std::string> &faces) {
    // The system loader validates the count (system.cpp); the assert is the
    // invariant twin for a caller that builds `faces` by hand.
    assert(faces.size() == 6 && "a cubemap needs six faces");
    GLuint tex = loadCubemap(faces);
    if(tex == 0) { return; }   // a face failed: keep the sky we already have
    if(cubemap != 0) { glDeleteTextures(1, &cubemap); }
    cubemap = tex;
}

void Skybox::Draw(const Camera * camera,
                  Shader * skyboxShader, const glm::dmat3 &skyRot) {
    const glm::dmat4 view = camera->GetView();
    // The cubemap is at rest in the root (inertial) frame; skyRot maps
    // root -> ship frame so the starfield drifts with the ship's rotation.
    const glm::dmat3 _rot = glm::dmat3(view) * skyRot; // clear to rotation
    const glm::mat4 _view = glm::mat4(_rot);
    const glm::mat4 projection = camera->GetProjection();

    skyboxShader->Bind();
    skyboxShader->setUniform_mat4(0, projection * _view);

    glBindVertexArray(vao);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(glGetUniformLocation(skyboxShader->m_program, "skybox"), 0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}
