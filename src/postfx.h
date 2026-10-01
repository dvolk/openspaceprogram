#ifndef POSTFX_INCLUDED_H
#define POSTFX_INCLUDED_H

#include <GL/glew.h>
#include <memory>
#include <string>
#include <vector>
#include "shader.h"

// A settable per-effect parameter: name (== uniform), slider range, neutral value.
struct FXParam {
    const char *name;
    float min;
    float max;
    float neutral;
};

// A built-in effect definition (the FX_DEFS table in postfx.cpp).
struct FXDef {
    const char *name;
    const char *canonical;
    const char *fs;
    const char **extra_uniforms; // nullptr-terminated
    const FXParam *params;       // settable parameters (nullptr = none)
    int n_params;
};

// Post-processing chain. No effects = zero cost (scene goes straight to
// the screen). N effects = scene into target 0, ping-pong fullscreen passes,
// last composites to the screen. Call Begin before the 3D scene and End
// after it (before the UI). Effects are toggleable at runtime (Settings).
class PostFX
{
public:
    PostFX();
    ~PostFX();

    // Built-in effect names, canonical, in the order the passes run.
    static const std::vector<std::string>& Available();
    // Create an effect by name (aliases: "sharpen"->"cas", "gamma"->"color").
    // Idempotent. New effects start disabled.
    bool AddEffect(const std::string& name);

    // Enable/disable (alias-resolved); creates on first enable.
    bool SetEnabled(const std::string& name, bool enabled);
    bool IsEnabled(const std::string& name) const;

    // The effect's settable parameters (slider order == return order).
    static std::vector<FXParam> Params(const std::string& name);

    // Set/get a parameter value. false / 0.0 for unknown effect or parameter.
    bool SetParam(const std::string& name, const std::string& param,
                  float value);
    float GetParam(const std::string& name, const std::string& param) const;

    // True if any effect is enabled (drives Begin/End).
    bool Active() const;

    // (Re)create the offscreen targets. Call at startup and on resize.
    void Resize(int width, int height);

    void Begin();
    void End();

private:
    void RebuildTargets(int width, int height);

    // The Shader is owned by pointer: Shader's destructor deletes the GL
    // program, so a by-value copy/move would free objects still referenced.
    struct Effect {
        const FXDef *def;         // its FX_DEFS entry (uniforms + parameters)
        std::string name;         // canonical name (alias-resolved)
        std::unique_ptr<Shader> shader;
        bool enabled = false;     // toggled from Settings / --postfx
        std::vector<float> param_values;  // parallel to def->params (neutral defaults)
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
