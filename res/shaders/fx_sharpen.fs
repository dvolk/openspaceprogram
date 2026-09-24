#version 450

in vec2 texcoord0;

out vec4 fragColor;

uniform sampler2D scene;
uniform vec2 resolution;

// Adaptive contrast (CAS): unsharp masking whose neighborhood adapts to
// local contrast. Near a strong edge the kernel widens and the taps are
// distance-weighted, so the blur that gets re-sharpened actually
// represents the edge -- that's what stops the ringing a fixed-kernel
// unsharp mask would produce on hard edges. (Public-GLSL form of AMD's
// CAS.)
vec3 cas(sampler2D tex, vec2 uv, vec2 texel, float strength)
{
    vec3 center = texture(tex, uv).rgb;

    vec2 delta1 = texel;
    vec2 delta2 = texel * 2.0;

    vec3 c1 = texture(tex, uv + vec2(delta1.x,  0.0)).rgb;
    vec3 c2 = texture(tex, uv - vec2(delta1.x,  0.0)).rgb;
    vec3 c3 = texture(tex, uv + vec2(0.0, delta1.y)).rgb;
    vec3 c4 = texture(tex, uv - vec2(0.0, delta1.y)).rgb;

    // How much the near neighbors deviate from the center: large means a
    // strong edge, so widen both kernels.
    vec3 delta = (c1 + c2 + c3 + c4) * 0.25 - center;
    float edge = max(length(vec2(length(delta.rg), delta.b)),
                     length(vec2(delta.r, length(delta.gb))));
    delta1 *= mix(1.0, 2.0, clamp(edge * 10.0, 1.0, 2.0));
    delta2 *= mix(1.0, 2.0, clamp(edge * 10.0, 1.0, 2.0));

    vec3 sum = vec3(0.0);
    float alphaTotal = 0.0;
#define CAS_TAP(dx, dy, w) \
    { \
        vec3 c = texture(tex, uv + vec2(dx, dy)).rgb; \
        float a = (w) / distance(vec2(dx, dy), vec2(0.0)); \
        sum += c * a; \
        alphaTotal += a; \
    }
    CAS_TAP( delta2.x,  0.0, 1.0)
    CAS_TAP(-delta2.x,  0.0, 1.0)
    CAS_TAP( 0.0, delta2.y, 1.0)
    CAS_TAP( 0.0,-delta2.y, 1.0)
    CAS_TAP( delta1.x, delta1.y, 0.5)
    CAS_TAP(-delta1.x, delta1.y, 0.5)
    CAS_TAP( delta1.x,-delta1.y, 0.5)
    CAS_TAP(-delta1.x,-delta1.y, 0.5)
#undef CAS_TAP

    vec3 blurred = sum / alphaTotal;

    return blurred + (center - blurred) * strength;
}

void main()
{
    vec2 texel = 1.0 / resolution;
    // strength > 1 sharpens: 1.15 is subtle, ~1.4 gets aggressive.
    vec3 col = cas(scene, texcoord0, texel, 1.15);
    fragColor = vec4(clamp(col, 0.0, 1.0), 1.0);
}
