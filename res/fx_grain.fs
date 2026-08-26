#version 450

in vec2 texcoord0;

out vec4 fragColor;

uniform sampler2D scene;
uniform float time;

// Precision-safe 2D->1D hash into [0,1). The classic
// fract(sin(dot(p, ...)) * 43758.5) hash quantizes in fp32 once p gets
// large (high resolution, long session times) and the grain turns into
// blocky static; this fract-based form does not suffer that.
float hash21(vec2 p)
{
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

void main()
{
    vec3 col = texture(scene, texcoord0).rgb;

    // Re-roll the noise at 24 Hz -- film cadence -- instead of on every
    // render frame, so the grain "boils" like real film regardless of
    // the (possibly much higher) frame rate.
    float frame = floor(time * 24.0);
    float n = hash21(gl_FragCoord.xy + frame * 12.9898);

    // The same value on all three channels reads as film grain rather
    // than color static.
    col += (n - 0.5) * 0.08;

    fragColor = vec4(clamp(col, 0.0, 1.0), 1.0);
}
