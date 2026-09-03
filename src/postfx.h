#ifndef POSTFX_INCLUDED_H
#define POSTFX_INCLUDED_H

#include <GL/glew.h>
#include <memory>
#include <string>
#include <vector>
#include "shader.h"

// Post-processing chain. With no effects active the scene renders
// straight to the screen (zero cost). With N active effects the scene
// renders into target 0 and each effect is a fullscreen pass that reads
// one target and writes the other (the two targets ping-pong, so any
// number of effects stacks); the last effect composites to the screen.
// Call Begin before the 3D scene and End after it (before the UI), so the
// HUD is drawn on top and stays crisp.
//
// Effects are toggleable at runtime (Settings): AddEffect creates one (in
// the disabled state), SetEnabled flips it on/off, and the passes run in
// the order the effects were added, skipping disabled ones.
class PostFX
{
public:
    PostFX();
    ~PostFX();

    // Built-in effect names, canonical, in the order the passes run.
    static const std::vector<std::string>& Available();
    // Create an effect by name ("crt", "grain", "cas", "gamma"; "sharpen"
    // is an alias for "cas"); false if unknown. Idempotent: adding a name
    // that already exists is a no-op. New effects start disabled.
    bool AddEffect(const std::string& name);

    // Enable/disable an effect by name (alias-resolved); false if unknown.
    // Creates the effect on first enable so it can be toggled at runtime.
    bool SetEnabled(const std::string& name, bool enabled);
    bool IsEnabled(const std::string& name) const;

    // The per-effect strength knob. Only the "gamma" effect uses it (as its
    // "gamma" uniform; 1.0 = neutral). Set/Get are no-ops returning
    // defaults for effects without a param.
    bool SetParam(const std::string& name, float value);
    float GetParam(const std::string& name) const;

    // True if any effect is enabled (drives Begin/End).
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
        std::string name;         // canonical name (alias-resolved)
        std::unique_ptr<Shader> shader;
        bool enabled = false;     // toggled from Settings / --postfx
        float param = 1.0f;       // strength; the "gamma" pass reads it as its "gamma" uniform (1.0 = neutral)
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
