#version 120

attribute vec2 position;
attribute vec2 uv;
varying vec2 texcoord0;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    texcoord0 = uv;
}
