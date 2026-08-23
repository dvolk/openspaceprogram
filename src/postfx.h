#ifndef POSTFX_INCLUDED_H
#define POSTFX_INCLUDED_H

#include <GL/glew.h>
#include "shader.h"

// Post-processing pass: the scene renders into an offscreen FBO (Begin),
// then the FBO is composited to the screen through the CRT shader (End).
// Call Begin before the 3D scene and End after it (before the UI), so the
// HUD is drawn on top and stays crisp.
class PostFX
{
public:
    PostFX();
    ~PostFX();

    // (Re)create the offscreen target. Call at startup and on resize.
    void Resize(int width, int height);

    void Begin();
    void End();

private:
    void RebuildTarget(int width, int height);

    GLuint m_fbo;
    GLuint m_colorTex;
    GLuint m_depthRB;
    GLuint m_quadVAO;
    GLuint m_quadVBO;
    int m_width;
    int m_height;
    Shader m_crt;
};

#endif
