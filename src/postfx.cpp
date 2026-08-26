#include "postfx.h"
#include "gldebug.h"
#include <iostream>
#include <utility>
#include "SDL2/SDL.h"

// Fullscreen quad: x, y in NDC + texture uv. Interleaved (x, y, u, v).
static const float QUAD_VERTS[16] = {
    -1.0f, -1.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f, 1.0f,
     1.0f,  1.0f, 1.0f, 1.0f,
};

// The composite must draw inside a *real* VAO: this Mesa 26 core profile
// answers GL_INVALID_OPERATION to any draw (or attribute state change)
// made in the default VAO 0 (see tests/test_vertexless.c). Explicit
// position/uv vertices are used instead of a gl_VertexID vertex-less
// triangle because the latter produced a mis-mapped triangle on an
// AMD driver (game image only in one screen quadrant).

// Built-in effects. All share the fullscreen-quad vertex shader and the
// `scene` input (texture unit 0); each registers exactly the uniforms
// its fragment shader uses (setUniform_* by name no-ops the rest).
struct FXDef {
    const char *name;
    const char *fs;
    const char **uniforms; // nullptr-terminated
};
static const char *CRT_UNIFORMS[] = { "scene", "resolution", "time", nullptr };
static const char *GRAIN_UNIFORMS[] = { "scene", "time", nullptr };
static const char *CAS_UNIFORMS[] = { "scene", "resolution", nullptr };
static const FXDef FX_DEFS[] = {
    { "crt", "./res/fx_crt", CRT_UNIFORMS },
    { "grain", "./res/fx_grain", GRAIN_UNIFORMS },
    { "cas", "./res/fx_sharpen", CAS_UNIFORMS },
    { "sharpen", "./res/fx_sharpen", CAS_UNIFORMS },
};

PostFX::PostFX() :
    m_quadVAO(0),
    m_quadVBO(0),
    m_width(0),
    m_height(0)
{
    m_fbo[0] = m_fbo[1] = 0;
    m_colorTex[0] = m_colorTex[1] = 0;
    m_depthRB[0] = m_depthRB[1] = 0;

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
}

PostFX::~PostFX()
{
    if(m_fbo[0] != 0) glDeleteFramebuffers(1, &m_fbo[0]);
    if(m_fbo[1] != 0) glDeleteFramebuffers(1, &m_fbo[1]);
    if(m_colorTex[0] != 0) glDeleteTextures(1, &m_colorTex[0]);
    if(m_colorTex[1] != 0) glDeleteTextures(1, &m_colorTex[1]);
    if(m_depthRB[0] != 0) glDeleteRenderbuffers(1, &m_depthRB[0]);
    if(m_depthRB[1] != 0) glDeleteRenderbuffers(1, &m_depthRB[1]);
    if(m_quadVAO != 0) glDeleteVertexArrays(1, &m_quadVAO);
    if(m_quadVBO != 0) glDeleteBuffers(1, &m_quadVBO);
    check_gl_error();
}

const std::vector<std::string>& PostFX::Available()
{
    static const std::vector<std::string> names = { "crt", "grain", "cas" };
    return names;
}

bool PostFX::AddEffect(const std::string& name)
{
    for(size_t i = 0; i < sizeof(FX_DEFS) / sizeof(FX_DEFS[0]); i++) {
        const FXDef &d = FX_DEFS[i];
        if(name != d.name) continue;

        Effect e;
        e.name = d.name;
        e.shader.reset(new Shader);
        e.shader->registerAttribs({ "position", "uv" });
        std::vector<const char *> uniforms;
        for(const char **u = d.uniforms; *u != nullptr; u++) {
            uniforms.push_back(*u);
        }
        e.shader->registerUniforms(uniforms);
        e.shader->FromFile("./res/fx_quad.vs", std::string(d.fs) + ".fs");
        m_effects.push_back(std::move(e));
        return true;
    }
    return false;
}

bool PostFX::Active() const
{
    return !m_effects.empty();
}

void PostFX::RebuildTargets(int width, int height)
{
    for(int i = 0; i < 2; i++) {
        if(m_fbo[i] != 0) glDeleteFramebuffers(1, &m_fbo[i]);
        if(m_colorTex[i] != 0) glDeleteTextures(1, &m_colorTex[i]);
        if(m_depthRB[i] != 0) glDeleteRenderbuffers(1, &m_depthRB[i]);
        check_gl_error();
        m_fbo[i] = m_colorTex[i] = m_depthRB[i] = 0;

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

        // The scene shaders write gl_FragDepth (log depth), so the
        // targets need a depth attachment like the default framebuffer
        // does (only target 0 actually receives depth writes).
        glGenRenderbuffers(1, &m_depthRB[i]);
        glBindRenderbuffer(GL_RENDERBUFFER, m_depthRB[i]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        check_gl_error();

        glGenFramebuffers(1, &m_fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_colorTex[i], 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, m_depthRB[i]);
        check_gl_error();
    }

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    check_gl_error();

    if(status != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "PostFX: framebuffer incomplete (" << status << ")" << std::endl;
    }
}

void PostFX::Resize(int width, int height)
{
    if(width <= 0 || height <= 0) return;
    if(m_fbo[0] != 0 && width == m_width && height == m_height) return;
    m_width = width;
    m_height = height;
    RebuildTargets(width, height);
}

void PostFX::Begin()
{
    if(!Active()) return;
    if(m_fbo[0] == 0) {
        std::cerr << "PostFX::Begin() before Resize()" << std::endl;
        return;
    }
    // The viewport is owned by Renderer::onResize (window size) and
    // matches the target size, so it is left alone here.
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[0]);
    check_gl_error();
}

void PostFX::End()
{
    if(!Active()) return;

    glDisable(GL_DEPTH_TEST);
    check_gl_error();

    const int n = (int)m_effects.size();
    for(int i = 0; i < n; i++) {
        // Effect i reads target i; the last effect composites to the
        // screen, the others write to the ping-ponged target i+1.
        glBindFramebuffer(GL_FRAMEBUFFER, i + 1 < n ? m_fbo[i + 1] : 0);
        check_gl_error();

        m_effects[i].shader->Bind();
        check_gl_error();

        glActiveTexture(GL_TEXTURE0);
        check_gl_error();
        glBindTexture(GL_TEXTURE_2D, m_colorTex[i]);
        check_gl_error();
        m_effects[i].shader->setUniform_i("scene", 0);
        m_effects[i].shader->setUniform_vec2("resolution",
                                             glm::vec2((float)m_width, (float)m_height));
        m_effects[i].shader->setUniform_vec1("time",
                                             (float)(SDL_GetTicks() / 1000.0));

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
    }

    glEnable(GL_DEPTH_TEST);
    check_gl_error();
}
