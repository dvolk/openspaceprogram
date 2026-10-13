#ifndef POSTFX_INCLUDED_H
#define POSTFX_INCLUDED_H

#include <GL/glew.h>
#include <memory>
#include <string>
#include <vector>
#include "shader.h"

// A per-effect parameter: name (== uniform), slider range, neutral value.
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

// Post-processing chain: no effects = zero cost; otherwise ping-pong fullscreen
// passes, the last compositing to the screen. Begin before the 3D scene, End
// after it.
class PostFX
{
public:
    PostFX();
    ~PostFX();

    // Built-in effect names, canonical, in pass order.
    static const std::vector<std::string>& Available();
    // Create an effect by name (aliases resolve to canonical). Idempotent;
    // starts disabled.
    bool AddEffect(const std::string& name);

    // Enable/disable (alias-resolved); creates on first enable.
    bool SetEnabled(const std::string& name, bool enabled);
    bool IsEnabled(const std::string& name) const;

    // The effect's settable parameters (slider order == return order).
    static std::vector<FXParam> Params(const std::string& name);

    // Set/get a parameter; false / 0.0 if unknown.
    bool SetParam(const std::string& name, const std::string& param,
                  float value);
    float GetParam(const std::string& name, const std::string& param) const;

    // True if any effect is enabled (drives Begin/End).
    bool Active() const;

    // (Re)create the offscreen targets (startup + resize).
    void Resize(int width, int height);

    void Begin();
    void End();

private:
    void RebuildTargets(int width, int height);

    // Shader held by pointer: its destructor deletes the GL program, so a
    // by-value copy would double-free.
    struct Effect {
        const FXDef *def;         // FX_DEFS entry
        std::string name;         // canonical (alias-resolved)
        std::unique_ptr<Shader> shader;
        bool enabled = false;     // Settings / --postfx
        std::vector<float> param_values;  // parallel to def->params
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
