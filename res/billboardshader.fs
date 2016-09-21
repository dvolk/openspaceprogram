#version 120

varying vec4 color0;
varying float logz;

void main()
{
    gl_FragColor = color0;
    gl_FragDepth = logz;
}
