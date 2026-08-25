#version 450

in vec3 position;
in vec2 texcoord;
in vec3 normal;
in vec3 color;

uniform mat4 MVP;
uniform vec4 color_uniform;

out vec4 color0;
out vec2 texcoord0;

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    color0 = color_uniform;
    texcoord0 = texcoord;
}
