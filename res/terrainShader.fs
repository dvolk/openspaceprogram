#version 450

in vec3 normal0;
in vec4 color0;

out vec4 fragColor;

//uniform sampler2D sampler;
uniform vec3 lightDirection;
uniform vec4 color;

void main()
{
    fragColor = color0 * clamp(dot(-lightDirection, normal0), 0.05, 1.0);
}
