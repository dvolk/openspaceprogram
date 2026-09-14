#version 450

in vec3 normal0;
in vec4 color0;

out vec4 fragColor;

//uniform sampler2D sampler;
uniform vec3 lightDirection;
uniform vec4 color;

void main()
{
    float light = clamp(dot(-lightDirection, normal0), 0.05, 1.0);
    // Opaque geometry: write alpha=1 explicitly. `color0 * light` would
    // scale the implicit alpha=1 by the light level, leaking the scene's
    // lighting into the framebuffer's alpha channel (in-game that is
    // ignored; the F12 screenshot saves it and the night side shows up
    // translucent in the PNG).
    fragColor = vec4(color0.rgb * light, 1.0);
}
