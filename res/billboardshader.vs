#version 120

attribute vec3 position;
attribute vec2 texcoord;
attribute vec3 normal;
attribute vec3 color;

uniform mat4 MVP;
uniform vec4 color_uniform;

varying vec4 color0;
varying vec2 texcoord0;

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    color0 = color_uniform;
    texcoord0 = texcoord;
}
