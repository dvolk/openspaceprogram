#ifndef POSTFX_INCLUDED_H
#define POSTFX_INCLUDED_H

#include <GL/glew.h>
#include <memory>
#include <string>
#include <vector>
#include "shader.h"

// Post-processing chain. With no effects the scene renders straight to
// the screen (zero cost). With N effects the scene renders into target 0
// and each effect is a fullscreen pass: effect i reads target i and
// writes to target i+1, the last effect compositing to the screen. The
// two targets ping-pong, so any number of effects stacks. Call Begin
// before the 3D scene and End after it (before the UI), so the HUD is
// drawn on top and stays crisp.
class PostFX
{
public:
    PostFX();
    ~PostFX();

    // Built-in effect names, as accepted by AddEffect.
    static const std::vector<std::string>& Available();
    // Load an effect by name ("crt", "grain", "cas"); false if unknown.
    // Effects apply in the order they are added.
    bool AddEffect(const std::string& name);

    bool Active() const;

    // (Re)create the offscreen targets. Call at startup and on resize.
    void Resize(int width, int height);

    void Begin();
    void End();

private:
    void RebuildTargets(int width, int height);

    // The Shader is owned by pointer: Shader's destructor deletes the GL
    // program, so a by-value copy/move (vector reallocation, a temporary
    // in AddEffect) would free objects a second Effect still references.
    struct Effect {
        std::string name;
        std::unique_ptr<Shader> shader;
    };
    std::vector<Effect> m_effects;

    GLuint m_fbo[2];
    GLuint m_colorTex[2];
    GLuint m_depthRB[2];
    GLuint m_quadVAO;
    GLuint m_quadVBO;
    int m_width;
    int m_height;
};

#endif
