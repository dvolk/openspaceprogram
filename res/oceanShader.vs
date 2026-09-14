#version 450

in vec3 position;
in vec3 normal;

out vec3 worldPos0;
out vec3 worldNormal0;
out vec3 bodyPos0;

uniform mat4 MVP;
uniform mat4 Normal;   // body-to-world model matrix

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    worldPos0 = (Normal * vec4(position, 1.0)).xyz;
    worldNormal0 = (Normal * vec4(normal, 0.0)).xyz;
    bodyPos0 = position;   // body-space for stable wave patterns
}
