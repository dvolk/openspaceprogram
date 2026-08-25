#version 120

attribute vec3 position;
attribute vec3 normal;

varying vec3 worldPos0;
varying vec3 worldNormal0;
varying float logz;

uniform mat4 MVP;
uniform mat4 Normal;   // actually the Model matrix (same convention as terrain)

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    worldPos0 = (Normal * vec4(position, 1.0)).xyz;
    worldNormal0 = (Normal * vec4(normal, 0.0)).xyz;

    // Logarithmic depth — must stay identical to terrainShader.vs (C=11,
    // far=1e13) or the depth test against the terrain and the far-plane
    // skybox breaks. See reports/atmosphere2026_08_25.
    const float C = 11;
    const float far = 1e13;
    const float FC = 1.0 / log(far * C + 1);

    logz = log(gl_Position.w * C + 1) * FC;
    gl_Position.z = (2.0 * logz - 1.0) * gl_Position.w;
}
