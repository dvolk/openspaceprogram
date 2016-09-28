#version 120

attribute vec3 position;

uniform mat4 MVP;
uniform vec4 color;

varying vec4 color0;

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    color0 = color;
}
