#version 450

in vec3 position;
in vec2 uv;
in vec3 normal;

out vec3 normal0;
out vec2 uv0;

uniform mat4 MVP;
uniform mat4 Normal;

void main()
{

    gl_Position = MVP * vec4(position, 1.0);
    normal0 = (Normal * vec4(normal, 0.0)).xyz;
    uv0 = uv;
}
