#version 120

varying vec2 texCoord0;
varying vec3 normal0;
varying float logz;

//uniform sampler2D sampler;
uniform vec3 lightDirection;
uniform vec4 color;

void main()
{
    gl_FragColor = color * clamp(dot(-lightDirection, normal0), 0.05, 1.0);
    gl_FragDepth = logz;
}
