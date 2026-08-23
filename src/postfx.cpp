#include "postfx.h"
#include "gldebug.h"
#include <iostream>
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
PostFX::PostFX() :
    m_fbo(0),
    m_colorTex(0),
    m_depthRB(0),
    m_quadVAO(0),
    m_quadVBO(0),
    m_width(0),
    m_height(0)
{
    m_crt.registerAttribs({ "position", "uv" });
    m_crt.registerUniforms({ "scene", "resolution", "time" });
    m_crt.FromFile("./res/crtShader");

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
    if(m_fbo != 0) glDeleteFramebuffers(1, &m_fbo);
    if(m_colorTex != 0) glDeleteTextures(1, &m_colorTex);
    if(m_depthRB != 0) glDeleteRenderbuffers(1, &m_depthRB);
    if(m_quadVAO != 0) glDeleteVertexArrays(1, &m_quadVAO);
    if(m_quadVBO != 0) glDeleteBuffers(1, &m_quadVBO);
    check_gl_error();
}

void PostFX::RebuildTarget(int width, int height)
{
    if(m_fbo != 0) glDeleteFramebuffers(1, &m_fbo);
    if(m_colorTex != 0) glDeleteTextures(1, &m_colorTex);
    if(m_depthRB != 0) glDeleteRenderbuffers(1, &m_depthRB);
    check_gl_error();
    m_fbo = m_colorTex = m_depthRB = 0;

    glGenTextures(1, &m_colorTex);
    glBindTexture(GL_TEXTURE_2D, m_colorTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    check_gl_error();

    // The scene shaders write gl_FragDepth (log depth), so the target
    // needs a depth attachment like the default framebuffer does.
    glGenRenderbuffers(1, &m_depthRB);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRB);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    check_gl_error();

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, m_colorTex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, m_depthRB);
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
    if(m_fbo != 0 && width == m_width && height == m_height) return;
    m_width = width;
    m_height = height;
    RebuildTarget(width, height);
}

void PostFX::Begin()
{
    if(m_fbo == 0) {
        std::cerr << "PostFX::Begin() before Resize()" << std::endl;
        return;
    }
    // The viewport is owned by Renderer::onResize (window size) and
    // matches the FBO size, so it is left alone here.
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    check_gl_error();
}

void PostFX::End()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    check_gl_error();

    glDisable(GL_DEPTH_TEST);
    check_gl_error();

    m_crt.Bind();
    check_gl_error();

    glActiveTexture(GL_TEXTURE0);
    check_gl_error();
    glBindTexture(GL_TEXTURE_2D, m_colorTex);
    check_gl_error();
    m_crt.setUniform_i(0, 0);
    m_crt.setUniform_vec2(1, glm::vec2((float)m_width, (float)m_height));
    m_crt.setUniform_vec1(2, (float)(SDL_GetTicks() / 1000.0));

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

    glEnable(GL_DEPTH_TEST);
    check_gl_error();
}
