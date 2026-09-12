#ifndef SHADER_INCLUDED_H
#define SHADER_INCLUDED_H

#include <string>
#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>

class Camera;
class Shader;

std::string LoadShader(const std::string& fileName);

/* Shared file-asset registry (shader.cpp): lookup-or-load, ONE program per
   file, so a shader is compiled once no matter how many systems use it.
   The registry owns the Shader (lives until process exit), so callers must
   never delete it. The attrib/uniform registration is part of the load
   (FromFile binds attribs and resolves uniform locations), so it is passed
   here and done exactly once -- every caller of one file must pass the
   same lists (they do: each shader file is used by one system). */
Shader *get_shader(const std::string &path,
                   const std::vector<const char *> &attribs,
                   const std::vector<const char *> &uniforms);

class Shader
{
public:
    void FromFile(const std::string& fileName);
    // Same, but explicit vertex/fragment paths (for effects that share
    // one vertex shader with several fragment shaders).
    void FromFile(const std::string& vertexFile, const std::string& fragmentFile);

    void Bind();

    void registerAttribs(std::vector<const char *> names);
    void registerUniforms(std::vector<const char *> names);

    /* true if (attribs, uniforms) is elementwise identical to the lists
       this Shader was registered with -- get_shader's cache-hit guard:
       uniform indices are positional, so a second caller with a different
       list would silently address the wrong uniforms. */
    bool registeredAs(const std::vector<const char *> &attribs,
                      const std::vector<const char *> &uniforms) const;

    void setUniform_i(int index, int v);
    void setUniform_vec1(int index, float v);
    void setUniform_vec2(int index, const glm::vec2 & v2);
    void setUniform_vec3(int index, const glm::vec3 & v3);
    void setUniform_vec4(int index, const glm::vec4 & v4);
    void setUniform_mat3(int index, const glm::mat3 & m3);
    void setUniform_mat4(int index, const glm::mat4 & m4);

    // Name-based variants: no-op if the uniform isn't in this program
    // (each shader only registers the uniforms it uses).
    void setUniform_i(const std::string& name, int v);
    void setUniform_vec1(const std::string& name, float v);
    void setUniform_vec2(const std::string& name, const glm::vec2 & v2);

    GLuint m_program;
    virtual ~Shader();
protected:
private:
    static const unsigned int NUM_SHADERS = 2;
    static const unsigned int MAX_NUM_UNIFORMS = 16;

    void CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string& errorMessage);
    GLuint CreateShader(const std::string& text, unsigned int type);
    int uniformIndex(const std::string& name);

    GLuint m_shaders[NUM_SHADERS];
    GLuint m_uniforms[MAX_NUM_UNIFORMS];

    std::vector<const char *> attribNames;
    std::vector<const char *> uniformNames;
};

#endif
