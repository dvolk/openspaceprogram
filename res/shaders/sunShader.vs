#version 450

in vec3 position;
in vec3 normal;
in vec3 color;

out vec3 normal0;
out vec4 color0;

uniform mat4 MVP;
uniform mat4 Normal;

void main()
{

    gl_Position = MVP * vec4(position, 1.0);
    normal0 = (Normal * vec4(normal, 0.0)).xyz;
    color0 = vec4(color, 1.0);
}
