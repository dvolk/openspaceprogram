#version 120

varying vec3 normal0;
varying vec4 color0;
varying float logz;

//uniform sampler2D sampler;
uniform vec3 lightDirection;
uniform vec4 color;

void main()
{
    gl_FragColor = color0;
    gl_FragDepth = logz;
}
