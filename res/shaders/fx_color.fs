#version 450

in vec2 texcoord0;

out vec4 fragColor;

uniform sampler2D scene;
uniform float gamma;       // 1.0 = no-op
uniform float brightness;  // 1.0 = no-op
uniform float black_level; // 0.0 = no-op
uniform float saturation;  // 1.0 = no-op

void main()
{
    vec3 col = texture(scene, texcoord0).rgb;

    // Color adjustments, applied in this order: correction, then
    // gain/offset, then saturation.
    // Gamma: col^(1/gamma). > 1.0 brightens (linear -> sRGB lands at ~2.2),
    // < 1.0 darkens.
    col = pow(col, vec3(1.0 / gamma));
    // Brightness: a multiplicative gain on every channel.
    col *= brightness;
    // Black level: the level a true-black signal displays at -- positive
    // lifts the blacks (haze), negative crushes them.
    col += black_level;
    // Saturation: mix toward the luminance (0.0 = grayscale,
    // > 1.0 oversaturates).
    float luma = dot(col, vec3(0.2126, 0.7152, 0.0722));
    col = mix(vec3(luma), col, saturation);

    fragColor = vec4(clamp(col, 0.0, 1.0), 1.0);
}
