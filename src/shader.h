#ifndef SHADER_INCLUDED_H
#define SHADER_INCLUDED_H

#include <string>
#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>

class Camera;

std::string LoadShader(const std::string& fileName);

class Shader
{
public:
    void FromFile(const std::string& fileName);

    void Bind();

    void registerAttribs(std::vector<const char *> names);
    void registerUniforms(std::vector<const char *> names);

    void setUniform_vec2(int index, const glm::vec2 & v2);
    void setUniform_vec3(int index, const glm::vec3 & v3);
    void setUniform_vec4(int index, const glm::vec4 & v4);
    void setUniform_mat4(int index, const glm::mat4 & m4);

    GLuint m_program;
    virtual ~Shader();
protected:
private:
    static const unsigned int NUM_SHADERS = 2;
    static const unsigned int MAX_NUM_UNIFORMS = 8;

    void CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string& errorMessage);
    GLuint CreateShader(const std::string& text, unsigned int type);

    GLuint m_shaders[NUM_SHADERS];
    GLuint m_uniforms[MAX_NUM_UNIFORMS];

    std::vector<const char *> attribNames;
    std::vector<const char *> uniformNames;
};

#endif
