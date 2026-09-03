#version 450

in vec2 texcoord0;

out vec4 fragColor;

uniform sampler2D scene;
uniform float gamma;

void main()
{
    vec3 col = texture(scene, texcoord0).rgb;

    // General gamma adjustment: col^(1/gamma). gamma == 1.0 is a no-op;
    // > 1.0 brightens (linear -> sRGB lands at ~2.2), < 1.0 darkens.
    col = pow(col, vec3(1.0 / gamma));

    fragColor = vec4(clamp(col, 0.0, 1.0), 1.0);
}
