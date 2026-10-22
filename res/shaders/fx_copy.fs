#version 450

in vec2 texcoord0;

out vec4 fragColor;

uniform sampler2D scene;

// The identity pass. PostFX always ends on the window, so the "no effects"
// path is this rather than a separate blit the call site has to know about.
void main()
{
    fragColor = texture(scene, texcoord0);
}
