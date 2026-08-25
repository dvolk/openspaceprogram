#include "skybox.h"

#include <SDL_image.h>
#include <map>
#include <string>
#include <vector>
#include <GL/glew.h>

#include "camera.h"
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

    // Faces may repeat (all six are the same starfield today), so decode
    // each distinct file once and upload the shared surface per face.
    std::map<std::string, SDL_Surface*> decoded;
    for(const GLchar* path : faces) {
        if(decoded.find(path) == decoded.end()) {
            decoded[std::string(path)] = IMG_Load(path);
        }
    }

    int width,height;
    unsigned char* image_data;

    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
    for(GLuint i = 0; i < faces.size(); i++) {
        SDL_Surface *image = decoded[std::string(faces[i])];
        width = image->w;
        height = image->h;
        image_data = (unsigned char *)image->pixels;

        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                     0,
                     GL_RGB,
                     width,
                     height,
                     0,
                     GL_RGB,
                     GL_UNSIGNED_BYTE,
                     image_data);
    }
    for(auto& kv : decoded) {
        SDL_FreeSurface(kv.second);
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    float aniso = max_anisotropy();
    if (aniso > 0.0f) {
        glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_ANISOTROPY, aniso);
    }
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

    // Cubemap (Skybox)
    std::vector<const GLchar*> faces;
    faces.push_back("res/skybox.png");
    faces.push_back("res/skybox.png");
    faces.push_back("res/skybox.png");
    faces.push_back("res/skybox.png");
    faces.push_back("res/skybox.png");
    faces.push_back("res/skybox.png");
    cubemapTexture = loadCubemap(faces);
}

void Skybox::Draw(const Camera * camera,
                  Shader * skyboxShader, const glm::dmat3 &skyRot) {
    glDepthFunc(GL_LEQUAL);

    const glm::dmat4 view = camera->GetView();
    // The cubemap is at rest in the root (star/inertial) frame, but the scene
    // is drawn in the ship's frame, which may be rotating. skyRot is the map
    // root -> ship frame, so the starfield drifts once per sidereal day while
    // standing on a spinning planet (identity = inertial world, as before).
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
    glDepthFunc(GL_LESS);
}
