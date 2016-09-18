#ifndef SHADER_INCLUDED_H
#define SHADER_INCLUDED_H

#include <string>
#include <GL/glew.h>
#include "camera.h"

class Shader
{
 public:
    Shader() {}
    void FromFile(const std::string& fileName);

    void Bind();
    /* void Update(const Transform& transform, glm::vec4& color, const Camera& camera, const glm::vec3& sunlightVec); */
    void Update(const glm::dmat4& Model, glm::vec4& color, const Camera& camera, const glm::vec3& sunlightVec);

    virtual ~Shader();
 protected:
 private:
    static const unsigned int NUM_SHADERS = 2;
    static const unsigned int NUM_UNIFORMS = 4;
    void operator=(const Shader& shader) {}
    Shader(const Shader& shader) {}

    std::string LoadShader(const std::string& fileName);
    void CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string& errorMessage);
    GLuint CreateShader(const std::string& text, unsigned int type);

    GLuint m_program;
    GLuint m_shaders[NUM_SHADERS];
    GLuint m_uniforms[NUM_UNIFORMS];
};

#endif
