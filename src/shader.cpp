#include <iostream>
#include <fstream>
#include <cstring>
#include <map>
#include "shader.h"
#include "gldebug.h"
#include "camera.h"
#include "resdir.h"

void Shader::FromFile(const std::string& fileName)
{
    FromFile(fileName + ".vs", fileName + ".fs");
}

void Shader::FromFile(const std::string& vertexFile, const std::string& fragmentFile)
{
    m_program = glCreateProgram();
    check_gl_error();
    m_shaders[0] = CreateShader(LoadShader(vertexFile), GL_VERTEX_SHADER);
    check_gl_error();
    m_shaders[1] = CreateShader(LoadShader(fragmentFile), GL_FRAGMENT_SHADER);
    check_gl_error();

    printf("Shader m_program: %d, vertex shader file %s\n", m_program, vertexFile.c_str());

    for(unsigned int i = 0; i < NUM_SHADERS; i++) {
        glAttachShader(m_program, m_shaders[i]);
        check_gl_error();
    }

    for(unsigned int i = 0; i < attribNames.size(); i++) {
        glBindAttribLocation(m_program, i, attribNames[i]);
        check_gl_error();
    }

    glLinkProgram(m_program);
    check_gl_error();
    CheckShaderError(m_program, GL_LINK_STATUS, true, "Error linking shader program");
    check_gl_error();

    glValidateProgram(m_program);
    check_gl_error();
    CheckShaderError(m_program, GL_VALIDATE_STATUS, true, "Invalid shader program");
    check_gl_error();

    // m_uniforms is otherwise indeterminate (Shader is plain `new`ed);
    // setUniform_* must never hand a garbage location to glUniform*.
    for(unsigned int i = 0; i < MAX_NUM_UNIFORMS; i++) {
        m_uniforms[i] = GL_INVALID_INDEX;
    }
    if(uniformNames.size() > MAX_NUM_UNIFORMS) {
        std::cerr << "ERROR: shader " << vertexFile << " registered "
                  << uniformNames.size() << " uniforms but MAX_NUM_UNIFORMS is "
                  << MAX_NUM_UNIFORMS << "; the extra names are dropped "
                  "(bump MAX_NUM_UNIFORMS in shader.h)" << std::endl;
    }
    for(unsigned int i = 0; i < uniformNames.size() && i < MAX_NUM_UNIFORMS; i++) {
        m_uniforms[i] = glGetUniformLocation(m_program, uniformNames[i]);
        check_gl_error();

        if(m_uniforms[i] == GL_INVALID_INDEX) {
            printf("WARNING: shader %s has no uniform named %s (optimized out by shader compiler?)\n",
                   vertexFile.c_str(),
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
    // No check_gl_error() here: glUseProgram cannot generate an error unless
    // the program handle is already bogus, and this sits in the draw path (a
    // shader is bound per part, so the drain tracked the draw count). Any real
    // error surfaces at the draw that follows.
    glUseProgram(m_program);
}

void Shader::registerAttribs(std::vector<const char *> names) {
    for(unsigned int i = 0; i < names.size(); i++) {
        attribNames.push_back(names[i]);
    }
}

void Shader::registerUniforms(std::vector<const char *> names) {
    for(unsigned int i = 0; i < names.size(); i++) {
        uniformNames.push_back(names[i]);
    }
}

bool Shader::registeredAs(const std::vector<const char *> &attribs,
                          const std::vector<const char *> &uniforms) const {
    if(attribs.size() != attribNames.size() ||
       uniforms.size() != uniformNames.size()) {
        return false;
    }
    for(size_t i = 0; i < attribs.size(); i++) {
        if(strcmp(attribs[i], attribNames[i]) != 0) { return false; }
    }
    for(size_t i = 0; i < uniforms.size(); i++) {
        if(strcmp(uniforms[i], uniformNames[i]) != 0) { return false; }
    }
    return true;
}

// A registered-but-optimized-out uniform has location GL_INVALID_INDEX;
// writing it would raise GL_INVALID_OPERATION on every call, so no-op instead.
void Shader::setUniform_i(int index, int v) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniform1i(m_uniforms[index], v);
}

void Shader::setUniform_vec1(int index, float v) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniform1f(m_uniforms[index], v);
}

void Shader::setUniform_vec2(int index, const glm::vec2 & v2) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniform2f(m_uniforms[index], v2.x, v2.y);
}

void Shader::setUniform_vec3(int index, const glm::vec3 & v3) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniform3f(m_uniforms[index], v3.x, v3.y, v3.z);
}

void Shader::setUniform_vec4(int index, const glm::vec4 & v4) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniform4f(m_uniforms[index], v4.x, v4.y, v4.z, v4.w);
}

void Shader::setUniform_mat3(int index, const glm::mat3 & m3) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniformMatrix3fv(m_uniforms[index], 1, GL_FALSE, &m3[0][0]);
}

void Shader::setUniform_mat4(int index, const glm::mat4 & m4) {
    if(index < 0 || index >= (int)uniformNames.size() || m_uniforms[index] == GL_INVALID_INDEX) {
        return;
    }
    glUniformMatrix4fv(m_uniforms[index], 1, GL_FALSE, &m4[0][0]);
}

int Shader::uniformIndex(const std::string& name) {
    for(size_t i = 0; i < uniformNames.size(); i++) {
        if(strcmp(uniformNames[i], name.c_str()) == 0) {
            return (int)i;
        }
    }
    return -1;
}

void Shader::setUniform_i(const std::string& name, int v) {
    int i = uniformIndex(name);
    if(i >= 0) setUniform_i(i, v);
}

void Shader::setUniform_vec1(const std::string& name, float v) {
    int i = uniformIndex(name);
    if(i >= 0) setUniform_vec1(i, v);
}

void Shader::setUniform_vec2(const std::string& name, const glm::vec2 & v2) {
    int i = uniformIndex(name);
    if(i >= 0) setUniform_vec2(i, v2);
}

std::string LoadShader(const std::string& fileName)
{
    std::ifstream file;
    file.open(resdir::path(fileName).c_str());

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

/* --- the shared file-shader registry (see shader.h) ---------------------- */
static std::map<std::string, Shader *> s_shaders;

Shader *get_shader(const std::string &path,
                   const std::vector<const char *> &attribs,
                   const std::vector<const char *> &uniforms) {
    // Cache on the resolved path (the stem: FromFile appends .vs/.fs).
    std::string key = resdir::path(path);
    std::map<std::string, Shader *>::iterator it = s_shaders.find(key);
    if(it != s_shaders.end()) {
        Shader *s = it->second;
        // Registration is fixed at first load; a second caller with a
        // DIFFERENT list would silently mis-address the positional uniforms.
        if(!s->registeredAs(attribs, uniforms)) {
            printf("ERROR: get_shader('%s') was already loaded with a "
                   "different attrib/uniform list -- the first registration "
                   "wins; the caller's uniform indices will not match\n",
                   key.c_str());
        }
        return s;
    }
    Shader *s = new Shader;
    s->registerAttribs(attribs);
    s->registerUniforms(uniforms);
    // FromFile/LoadShader resolve for the open; `path` stays the logical
    // name in the log line below.
    s->FromFile(path);
    // A link failure is cached so a broken shader would not silently render nothing.
    GLint linked = 0;
    glGetProgramiv(s->m_program, GL_LINK_STATUS, &linked);
    if(linked == GL_FALSE) {
        printf("WARNING: shader '%s' failed to link -- its draws will be "
               "invisible\n", key.c_str());
    }
    s_shaders[key] = s;
    return s;
}

/* The world shader set (see shader.h). Each entry is the file plus its
   attrib/uniform registration -- positional, so the ORDER is part of the
   contract. The boot and Game::switchSystem both call this; the registry
   hands them the same program. */
WorldShaders get_world_shaders() {
    auto load = [](const char *path, std::vector<const char *> attribs,
                   std::vector<const char *> uniforms) {
        return get_shader(path, attribs, uniforms);
    };
    WorldShaders w;
    w.terrain = load("res/shaders/terrainShader",
                     { "position", "normal", "color" },
                     { "MVP", "Normal", "lightDirection", "color", "anchor" });
    w.sun = load("res/shaders/sunShader",
                 { "position", "normal", "color" },
                 { "MVP", "Normal", "lightDirection", "color" });
    // Atmosphere shell: Fresnel limb glow from orbit, interior sky dome
    // from the surface (the `inside` flag). See reports/atmosphere2026_08_25.
    w.atmosphere = load("res/shaders/atmosphereShader",
                        { "position", "normal" },
                        { "MVP", "Normal", "cameraPos", "color", "intensity",
                          "power", "lightDirection", "inside", "planetCenter" });
    // Cloud deck: a shell between the terrain and the atmosphere rim.
    // Coverage is baked (BuildClouds) on the job worker. "uvParam" binds
    // the mesh's color slot (attrib location 2): the unwrapped sphere
    // params the deck UV is built from.
    w.cloud = load("res/shaders/cloudShader",
                   { "position", "normal", "uvParam" },
                   { "MVP", "Normal", "cameraPos", "color", "lightDirection",
                     "drift", "planetCenter", "coverage_tex" });
    // Ocean surface: a transparent shell at sea level with animated wave
    // normals, Fresnel reflection and a specular sun glint. Land pokes
    // through via the depth test.
    w.ocean = load("res/shaders/oceanShader",
                   { "position", "normal" },
                   { "MVP", "Normal", "cameraPos", "seaColor",
                     "lightDirection", "time", "planetCenter" });
    // Planetary rings: flat annuli in the body's equatorial plane, drawn
    // over the opaque terrain. Two-sided Lambert (the sun can be above or
    // below the plane).
    w.ring = load("res/shaders/ringShader",
                  { "position", "normal" },
                  { "MVP", "Normal", "lightDirection", "albedo", "opacity",
                    "planetRadius" });
    return w;
}
