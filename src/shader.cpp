#include <iostream>
#include <fstream>
#include "shader.h"
#include "gldebug.h"
#include "camera.h"

void Shader::FromFile(const std::string& fileName)
{
  m_program = glCreateProgram();
  check_gl_error();
  m_shaders[0] = CreateShader(LoadShader(fileName + ".vs"), GL_VERTEX_SHADER);
  check_gl_error();
  m_shaders[1] = CreateShader(LoadShader(fileName + ".fs"), GL_FRAGMENT_SHADER);
  check_gl_error();

  printf("Shader m_program: %d, vertex shader file %s\n", m_program, fileName.c_str());

  for(unsigned int i = 0; i < NUM_SHADERS; i++) {
    glAttachShader(m_program, m_shaders[i]);
    check_gl_error();
  }

  for(int i = 0; i < attribNames.size(); i++) {
    glBindAttribLocation(m_program, i, attribNames[i]);
    check_gl_error();
  }

  glLinkProgram(m_program);
  check_gl_error();
  CheckShaderError(m_program, GL_LINK_STATUS, true, "Error linking shader program");
  check_gl_error();

  glValidateProgram(m_program);
  check_gl_error();
  CheckShaderError(m_program, GL_LINK_STATUS, true, "Invalid shader program");
  check_gl_error();

  for(int i = 0; i < uniformNames.size(); i++) {
    m_uniforms[i] = glGetUniformLocation(m_program, uniformNames[i]);
    check_gl_error();

    if(m_uniforms[i] == -1) {
      printf("WARNING: shader %s has no uniform named %s (optimized out by shader compiler?)\n",
	     fileName.c_str(),
	     uniformNames[i]);
    }
  }
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
  //printf("program: %d\n", m_program);
  glUseProgram(m_program);
  check_gl_error();
}

void Shader::registerAttribs(std::vector<const char *> names) {
  for(int i = 0; i < names.size(); i++) {
    attribNames.push_back(names[i]);
  }
}

void Shader::registerUniforms(std::vector<const char *> names) {
  for(int i = 0; i < names.size(); i++) {
    uniformNames.push_back(names[i]);
  }
}

void Shader::setUniform_vec2(int index, const glm::vec2 & v2) {
  glUniform2f(m_uniforms[index], v2.x, v2.y);
}

void Shader::setUniform_vec3(int index, const glm::vec3 & v3) {
  glUniform3f(m_uniforms[index], v3.x, v3.y, v3.z);
}

void Shader::setUniform_vec4(int index, const glm::vec4 & v4) {
  glUniform4f(m_uniforms[index], v4.x, v4.y, v4.z, v4.w);
}

void Shader::setUniform_mat4(int index, const glm::mat4 & m4) {
  glUniformMatrix4fv(m_uniforms[index], 1, GL_FALSE, &m4[0][0]);
}

std::string LoadShader(const std::string& fileName)
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
