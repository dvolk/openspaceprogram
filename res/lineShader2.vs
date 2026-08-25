#version 450

in vec3 position;

uniform mat4 MVP;
uniform vec4 color;

out vec4 color0;

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    color0 = color;
}
