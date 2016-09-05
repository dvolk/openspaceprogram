#include <iostream>
#include <fstream>
#include "shader.h"
#include "gldebug.h"

void Shader::FromFile(const std::string& fileName)
{
    m_program = glCreateProgram();
    check_gl_error();
    m_shaders[0] = CreateShader(LoadShader(fileName + ".vs"), GL_VERTEX_SHADER);
    check_gl_error();
    m_shaders[1] = CreateShader(LoadShader(fileName + ".fs"), GL_FRAGMENT_SHADER);
    check_gl_error();

    for(unsigned int i = 0; i < NUM_SHADERS; i++) {
        glAttachShader(m_program, m_shaders[i]);
        check_gl_error();
    }

    glBindAttribLocation(m_program, 0, "position");
    check_gl_error();
    glBindAttribLocation(m_program, 1, "texCoord");
    check_gl_error();
    glBindAttribLocation(m_program, 2, "normal");
    check_gl_error();
    glBindAttribLocation(m_program, 3, "color");
    check_gl_error();

    glLinkProgram(m_program);
    check_gl_error();
    CheckShaderError(m_program, GL_LINK_STATUS, true, "Error linking shader program");
    check_gl_error();

    glValidateProgram(m_program);
    check_gl_error();
    CheckShaderError(m_program, GL_LINK_STATUS, true, "Invalid shader program");
    check_gl_error();

    m_uniforms[0] = glGetUniformLocation(m_program, "MVP");
    check_gl_error();
    m_uniforms[1] = glGetUniformLocation(m_program, "Normal");
    check_gl_error();
    m_uniforms[2] = glGetUniformLocation(m_program, "lightDirection");
    check_gl_error();
    m_uniforms[3] = glGetUniformLocation(m_program, "color");
    check_gl_error();
}

Shader::~Shader()
{
    for(unsigned int i = 0; i < NUM_SHADERS; i++)
        {
            glDetachShader(m_program, m_shaders[i]);
    check_gl_error();
            glDeleteShader(m_shaders[i]);
    check_gl_error();
        }

    glDeleteProgram(m_program);
    check_gl_error();
}

void Shader::Bind()
{
    glUseProgram(m_program);
    check_gl_error();
}

void Shader::Update(const glm::dmat4& Model, glm::vec4& color, const Camera& camera)
{
    const glm::dmat4 View = camera.GetView();
    // make sure View * Model happens with double precision
    const glm::dmat4 ModelView = View * Model;
    const glm::mat4 ModelViewFloat = ModelView;
    const glm::mat4 Projection = camera.GetProjection();
    const glm::mat4 MVP = Projection * ModelViewFloat;
    const glm::mat4 ModelFloat = Model;
    check_gl_error();
    glUniformMatrix4fv(m_uniforms[0], 1, GL_FALSE, &MVP[0][0]);
    check_gl_error();
    glUniformMatrix4fv(m_uniforms[1], 1, GL_FALSE, &ModelFloat[0][0]);
    check_gl_error();

    glUniform3f(m_uniforms[2], 0.0f, 0.0f, 1.0f);
    check_gl_error();
    glUniform4f(m_uniforms[3], color.x, color.y, color.z, color.w);
    check_gl_error();
}

std::string Shader::LoadShader(const std::string& fileName)
{
    std::ifstream file;
    file.open((fileName).c_str());

    std::string output;
    std::string line;

    if(file.is_open())
        {
            while(file.good())
                {
                    getline(file, line);
                    output.append(line + "\n");
                }
        }
    else
        {
            std::cerr << "Unable to load shader: " << fileName << std::endl;
        }

    return output;
}

void Shader::CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string& errorMessage)
{
    GLint success = 0;
    GLchar error[1024] = { 0 };

    if(isProgram)
        glGetProgramiv(shader, flag, &success);
    else
        glGetShaderiv(shader, flag, &success);

    if(success == GL_FALSE)
        {
            if(isProgram)
                glGetProgramInfoLog(shader, sizeof(error), NULL, error);
            else
                glGetShaderInfoLog(shader, sizeof(error), NULL, error);

            std::cerr << errorMessage << ": '" << error << "'" << std::endl;
        }
}

GLuint Shader::CreateShader(const std::string& text, unsigned int type)
{
    GLuint shader = glCreateShader(type);

    if(shader == 0)
        std::cerr << "Error compiling shader type " << type << std::endl;

    const GLchar* p[1];
    p[0] = text.c_str();
    GLint lengths[1];
    lengths[0] = text.length();

    glShaderSource(shader, 1, p, lengths);
    glCompileShader(shader);

    CheckShaderError(shader, GL_COMPILE_STATUS, false, "Error compiling shader!");

    return shader;
}
