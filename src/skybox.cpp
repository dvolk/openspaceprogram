#include "skybox.h"

#include <SDL3_image/SDL_image.h>
#include <cassert>
#include <filesystem>
#include <string>
#include <vector>
#include <GL/glew.h>

#include "camera.h"
#include "resdir.h"
#include "shader.h"
#include "texture.h"

GLuint skyboxVAO, skyboxVBO;
GLuint cubemapTexture;

Skybox::~Skybox() {
    glDeleteBuffers(1, &skyboxVBO);
    glDeleteVertexArrays(1, &skyboxVAO);
    glDeleteTextures(1, &cubemapTexture);
}

GLuint loadCubemap(std::vector<const GLchar*> faces)
{
    GLuint textureID;
    glGenTextures(1, &textureID);

    // The upload below assumes tightly packed RGB24 rows; validate (and
    // normalize) rather than silently skewing an RGBA or odd-sized face.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
    int face_w = 0;
    for(GLuint i = 0; i < faces.size(); i++) {
        SDL_Surface *loaded = IMG_Load(resdir::path(faces[i]).c_str());
        assert(loaded && "skybox face failed to load");
        SDL_Surface *image = loaded;
        if(image->format != SDL_PIXELFORMAT_RGB24) {
            image = SDL_ConvertSurface(loaded, SDL_PIXELFORMAT_RGB24);
            assert(image && "skybox face conversion to RGB24 failed");
        }
        assert(image->w == image->h && (face_w == 0 || image->w == face_w)
               && "skybox faces must be same-size squares");
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
        if(image != loaded) SDL_DestroySurface(image);
        SDL_DestroySurface(loaded);
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

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
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
    glBindVertexArray(0);

    // Cubemap (Skybox), GL face order (+X,-X,+Y,-Y,+Z,-Z). The real-star
    // faces from utils/make_skybox.py are opt-in staging: if all six exist
    // under tmp/newskybox/ use them, else the committed tiled placeholder.
    static const char* baked[] = {
        "tmp/newskybox/skybox_px.png", "tmp/newskybox/skybox_nx.png",
        "tmp/newskybox/skybox_py.png", "tmp/newskybox/skybox_ny.png",
        "tmp/newskybox/skybox_pz.png", "tmp/newskybox/skybox_nz.png"};
    std::vector<const GLchar*> faces;
    bool have_baked = true;
    for(const char* p : baked) {
        if(!std::filesystem::exists(resdir::path(p))) { have_baked = false; break; }
    }
    if(have_baked) {
        for(const char* p : baked) faces.push_back(p);
    } else {
        for(int i = 0; i < 6; i++) faces.push_back("res/textures/skybox.png");
    }
    cubemapTexture = loadCubemap(faces);
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

    glBindVertexArray(skyboxVAO);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(glGetUniformLocation(skyboxShader->m_program, "skybox"), 0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}
