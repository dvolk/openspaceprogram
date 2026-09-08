#version 450

in vec3 position;
in vec3 normal;

out vec3 worldPos0;
out vec3 worldNormal0;
out vec3 localDir;
out float logz;

uniform mat4 MVP;
uniform mat4 Normal;   // actually the Model matrix (same convention as terrain)

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    worldPos0 = (Normal * vec4(position, 1.0)).xyz;
    worldNormal0 = (Normal * vec4(normal, 0.0)).xyz;
    // The deck is a sphere centred on the body, so the local position IS
    // its direction: the FBM samples it in the body's frame (seedRot in
    // the fragment shader), keeping the pattern fixed to the surface as
    // the ship flies by.
    localDir = normalize(position);

    // Logarithmic depth — must stay identical to terrainShader.vs (C=11,
    // far=1e13) or the depth test against the terrain and the far-plane
    // skybox breaks. See reports/atmosphere2026_08_25.
    const float C = 11;
    const float far = 1e13;
    const float FC = 1.0 / log(far * C + 1);

    logz = log(gl_Position.w * C + 1) * FC;
    gl_Position.z = (2.0 * logz - 1.0) * gl_Position.w;
}
