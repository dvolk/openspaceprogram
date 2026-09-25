#version 450

in vec3 position;
in vec3 normal;

out vec3 worldNormal0;
out vec3 localPos0;

uniform mat4 MVP;
uniform mat4 Normal;   // actually the Model matrix (same convention as terrain)

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    worldNormal0 = (Normal * vec4(normal, 0.0)).xyz;
    // Body-local position: the planet is a sphere at the local origin, so
    // the sun-shadow test can stay in this frame (no huge world coords).
    localPos0 = position;
}
