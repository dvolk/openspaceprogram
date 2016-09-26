#include "skybox.h"

#include <SDL_image.h>
#include <vector>
#include <GL/glew.h>

#include "camera.h"
#include "shader.h"

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

  int width,height;
  unsigned char* image_data;

  glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
  for(GLuint i = 0; i < faces.size(); i++) {
    SDL_Surface *image = IMG_Load(faces[i]);
    width = image->w;
    height = image->h;
    image_data = (unsigned char *)image->pixels;

    // image = SOIL_load_image(faces[i], &width, &height, 0, SOIL_LOAD_RGB);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
		 0,
		 GL_RGB,
		 width,
		 height,
		 0,
		 GL_RGB,
		 GL_UNSIGNED_BYTE,
		 image_data);

    SDL_FreeSurface(image);
  }

  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
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
		  Shader * skyboxShader) {
  glDepthFunc(GL_LEQUAL);

  const glm::dmat4 view = camera->GetView();
  const glm::mat4 _view = glm::mat4(glm::mat3(view)); // clear to rotation
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
