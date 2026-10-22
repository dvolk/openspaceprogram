#include "postfx.h"
#include "gldebug.h"
#include <cassert>
#include <iostream>
#include <utility>
#include <SDL3/SDL.h>

// Fullscreen quad: x, y in NDC + texture uv. Interleaved (x, y, u, v).
static const float QUAD_VERTS[16] = {
    -1.0f, -1.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f, 1.0f,
     1.0f,  1.0f, 1.0f, 1.0f,
};

// The composite must draw inside a *real* VAO: Mesa core profile answers
// GL_INVALID_OPERATION to any draw in the default VAO 0. Explicit
// position/uv vertices (not gl_VertexID) because the latter mis-mapped on AMD.
//
// Built-in effects: all share the quad vertex shader and the `scene` input
// (texture unit 0). `name` is requestable (aliases resolve to `canonical`);
// `params` are settable.
static const FXParam COLOR_PARAMS[] = {
    { "gamma",       0.25f, 3.0f, 1.0f },
    { "brightness",  0.0f,  2.0f, 1.0f },
    { "black_level", -0.5f, 0.5f, 0.0f },
    { "saturation",  0.0f,  2.0f, 1.0f },
};
static const char *CRT_EXTRA[] = { "resolution", "time", nullptr };
static const char *GRAIN_EXTRA[] = { "time", nullptr };
static const char *CAS_EXTRA[] = { "resolution", nullptr };
static const FXDef FX_DEFS[] = {
    { "crt",     "crt",     "res/shaders/fx_crt",     CRT_EXTRA,     nullptr,      0 },
    { "grain",   "grain",   "res/shaders/fx_grain",   GRAIN_EXTRA,   nullptr,      0 },
    { "cas",     "cas",     "res/shaders/fx_sharpen", CAS_EXTRA,     nullptr,      0 },
    { "sharpen", "cas",     "res/shaders/fx_sharpen", CAS_EXTRA,     nullptr,      0 },
    { "color",   "color",   "res/shaders/fx_color",   nullptr,       COLOR_PARAMS, 4 },
    { "gamma",   "color",   "res/shaders/fx_color",   nullptr,       COLOR_PARAMS, 4 },
};

static const FXDef *FindDef(const std::string& name) {
    for(size_t i = 0; i < sizeof(FX_DEFS) / sizeof(FX_DEFS[0]); i++) {
        if(name == FX_DEFS[i].name) return &FX_DEFS[i];
    }
    return nullptr;
}

PostFX::PostFX() :
    m_quadVAO(0),
    m_quadVBO(0),
    m_width(0),
    m_height(0)
{
    m_fbo[0] = m_fbo[1] = 0;
    m_colorTex[0] = m_colorTex[1] = 0;

    glGenVertexArrays(1, &m_quadVAO);
    check_gl_error();
    glGenBuffers(1, &m_quadVBO);
    check_gl_error();
    glBindVertexArray(m_quadVAO);
    check_gl_error();
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    check_gl_error();
    glBufferData(GL_ARRAY_BUFFER, sizeof(QUAD_VERTS), QUAD_VERTS, GL_STATIC_DRAW);
    check_gl_error();
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (const GLvoid*)0);
    check_gl_error();
    glEnableVertexAttribArray(0);
    check_gl_error();
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (const GLvoid*)(2 * sizeof(float)));
    check_gl_error();
    glEnableVertexAttribArray(1);
    check_gl_error();
    glBindVertexArray(0);
    check_gl_error();

    m_copy.reset(new Shader);
    m_copy->registerAttribs({ "position", "uv" });
    m_copy->registerUniforms({ "scene" });
    m_copy->FromFile("res/shaders/fx_quad.vs", "res/shaders/fx_copy.fs");
    /* This pass is what covers the window when no effect is on, so a missing
       or unlinked fx_copy.fs is a blank frame with a working UI -- not a
       degraded post-fx chain. Shader::FromFile only prints. */
    GLint linked = 0;
    glGetProgramiv(m_copy->m_program, GL_LINK_STATUS, &linked);
    assert(linked);
}

PostFX::~PostFX()
{
    if(m_fbo[0] != 0) glDeleteFramebuffers(1, &m_fbo[0]);
    if(m_fbo[1] != 0) glDeleteFramebuffers(1, &m_fbo[1]);
    if(m_colorTex[0] != 0) glDeleteTextures(1, &m_colorTex[0]);
    if(m_colorTex[1] != 0) glDeleteTextures(1, &m_colorTex[1]);
    if(m_quadVAO != 0) glDeleteVertexArrays(1, &m_quadVAO);
    if(m_quadVBO != 0) glDeleteBuffers(1, &m_quadVBO);
    check_gl_error();
}

static std::vector<std::string> BuildAvailable()
{
    // Canonical names in pass order, deduped (aliases collapse).
    std::vector<std::string> out;
    for(size_t i = 0; i < sizeof(FX_DEFS) / sizeof(FX_DEFS[0]); i++) {
        const std::string c = FX_DEFS[i].canonical;
        bool dup = false;
        for(size_t j = 0; j < out.size(); j++) {
            if(out[j] == c) { dup = true; break; }
        }
        if(!dup) out.push_back(c);
    }
    return out;
}

const std::vector<std::string>& PostFX::Available()
{
    static const std::vector<std::string> names = BuildAvailable();
    return names;
}

bool PostFX::AddEffect(const std::string& name)
{
    const FXDef *def = FindDef(name);
    if(def == nullptr) return false;

    // Idempotent: an effect with this canonical name already exists.
    for(size_t i = 0; i < m_effects.size(); i++) {
        if(m_effects[i].name == def->canonical) return true;
    }

    Effect e;
    e.def = def;
    e.name = def->canonical;
    e.enabled = false;
    for(int i = 0; i < def->n_params; i++) {
        e.param_values.push_back(def->params[i].neutral);
    }
    e.shader.reset(new Shader);
    e.shader->registerAttribs({ "position", "uv" });
    std::vector<const char *> uniforms;
    uniforms.push_back("scene");
    for(int i = 0; i < def->n_params; i++) {
        uniforms.push_back(def->params[i].name);
    }
    if(def->extra_uniforms != nullptr) {
        for(const char **u = def->extra_uniforms; *u != nullptr; u++) {
            uniforms.push_back(*u);
        }
    }
    e.shader->registerUniforms(uniforms);
    e.shader->FromFile("res/shaders/fx_quad.vs", std::string(def->fs) + ".fs");
    m_effects.push_back(std::move(e));
    return true;
}

bool PostFX::SetEnabled(const std::string& name, bool enabled)
{
    const FXDef *def = FindDef(name);
    if(def == nullptr) return false;
    if(!AddEffect(name)) return false;
    for(size_t i = 0; i < m_effects.size(); i++) {
        if(m_effects[i].name == def->canonical) {
            m_effects[i].enabled = enabled;
            return true;
        }
    }
    return false;
}

bool PostFX::IsEnabled(const std::string& name) const
{
    const FXDef *def = FindDef(name);
    if(def == nullptr) return false;
    for(size_t i = 0; i < m_effects.size(); i++) {
        if(m_effects[i].name == def->canonical) return m_effects[i].enabled;
    }
    return false;
}

std::vector<FXParam> PostFX::Params(const std::string& name)
{
    std::vector<FXParam> out;
    const FXDef *def = FindDef(name);
    if(def == nullptr) { return out; }
    for(int i = 0; i < def->n_params; i++) {
        out.push_back(def->params[i]);
    }
    return out;
}

static int ParamIndex(const FXDef *def, const std::string& param)
{
    for(int i = 0; i < def->n_params; i++) {
        if(param == def->params[i].name) return i;
    }
    return -1;
}

bool PostFX::SetParam(const std::string& name, const std::string& param,
                      float value)
{
    const FXDef *def = FindDef(name);
    if(def == nullptr) { return false; }
    int idx = ParamIndex(def, param);
    if(idx < 0) { return false; }
    for(size_t i = 0; i < m_effects.size(); i++) {
        if(m_effects[i].name == def->canonical) {
            m_effects[i].param_values[idx] = value;
            return true;
        }
    }
    return false;   // the effect has not been created yet
}

float PostFX::GetParam(const std::string& name, const std::string& param) const
{
    const FXDef *def = FindDef(name);
    if(def == nullptr) { return 0.0f; }
    int idx = ParamIndex(def, param);
    if(idx < 0) { return 0.0f; }
    for(size_t i = 0; i < m_effects.size(); i++) {
        if(m_effects[i].name == def->canonical) {
            return m_effects[i].param_values[idx];
        }
    }
    return def->params[idx].neutral;   // the effect has not been created yet
}

void PostFX::RebuildTargets(int width, int height)
{
    for(int i = 0; i < 2; i++) {
        if(m_fbo[i] != 0) glDeleteFramebuffers(1, &m_fbo[i]);
        if(m_colorTex[i] != 0) glDeleteTextures(1, &m_colorTex[i]);
        check_gl_error();
        m_fbo[i] = m_colorTex[i] = 0;

        glGenTextures(1, &m_colorTex[i]);
        glBindTexture(GL_TEXTURE_2D, m_colorTex[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        check_gl_error();

        glGenFramebuffers(1, &m_fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_colorTex[i], 0);
        // Validate EACH target, inside the loop: checking after it would only
        // ever see the last bound one.
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if(status != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "PostFX: framebuffer " << i << " incomplete ("
                      << status << ")" << std::endl;
        }
        check_gl_error();
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    check_gl_error();
}

void PostFX::Resize(int width, int height)
{
    if(width <= 0 || height <= 0) return;
    if(m_fbo[0] != 0 && width == m_width && height == m_height) return;
    m_width = width;
    m_height = height;
    RebuildTargets(width, height);
}

void PostFX::Composite(unsigned int sceneTexture)
{
    /* A 2D pass over the whole window: pin the raster state it assumes instead
       of inheriting whatever the 3D pass left (wireframe, blending, culling),
       and hand back what the UI and the next frame expect. */
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    check_gl_error();

    // Only the enabled effects run, in the order they were added; each reads
    // the previous image and writes one of the two targets, and the last
    // writes the window. With none enabled the identity pass covers the
    // window, so the call site has no "did anything composite?" branch.
    int nactive = 0;
    for(size_t i = 0; i < m_effects.size(); i++) {
        if(m_effects[i].enabled) nactive++;
    }
    if(nactive > 0 && m_fbo[0] == 0) {
        // Resize() runs at startup, so this is a caller bug, not a state to
        // paper over; sampling texture 0 would black the screen.
        std::cerr << "PostFX::Composite() before Resize()" << std::endl;
    } else if(nactive == 0) {
        m_copy->Bind();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sceneTexture);
        m_copy->setUniform_i("scene", 0);
        glBindVertexArray(m_quadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glUseProgram(0);
        check_gl_error();
    } else {
        GLuint read = sceneTexture;   // the current source image
        int run = 0;                  // how many active passes have run
        for(size_t i = 0; i < m_effects.size(); i++) {
            Effect &e = m_effects[i];
            if(!e.enabled) continue;

            const bool last = (run + 1 == nactive);
            const int write = run % 2;
            glBindFramebuffer(GL_FRAMEBUFFER, last ? 0 : m_fbo[write]);
            check_gl_error();

            e.shader->Bind();
            check_gl_error();

            glActiveTexture(GL_TEXTURE0);
            check_gl_error();
            glBindTexture(GL_TEXTURE_2D, read);
            check_gl_error();
            e.shader->setUniform_i("scene", 0);
            e.shader->setUniform_vec2("resolution",
                                     glm::vec2((float)m_width, (float)m_height));
            e.shader->setUniform_vec1("time",
                                     (float)(SDL_GetTicks() / 1000.0));
            for(int i = 0; i < e.def->n_params; i++) {
                e.shader->setUniform_vec1(e.def->params[i].name,
                                          e.param_values[i]);
            }

            glBindVertexArray(m_quadVAO);
            check_gl_error();
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            check_gl_error();
            glBindVertexArray(0);
            check_gl_error();

            glBindTexture(GL_TEXTURE_2D, 0);
            check_gl_error();
            glUseProgram(0);
            check_gl_error();

            if(!last) { read = m_colorTex[write]; }
            run++;
        }
    }

    // The window, culling and depth-testing back on: what the UI's own draws
    // and the next frame's 3D pass start from.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    check_gl_error();
}
