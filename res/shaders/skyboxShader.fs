#version 450

in vec3 texcoord0;

out vec4 fragColor;

uniform samplerCube skybox;

/* Fake exposure (render.cpp skyGain). The star field and the sun's disk share
   one 8-bit ceiling and the pipeline has no HDR headroom, so the brightest
   stars land at the same value as the star itself. `gain` fades the sky while
   the sun is in view; 1.0 = no fade. */
uniform float gain;

void main()
{
    fragColor = vec4(texture(skybox, texcoord0).rgb * gain, 1.0);
}
