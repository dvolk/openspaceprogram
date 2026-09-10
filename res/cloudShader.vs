#version 450

in vec3 position;
in vec3 normal;
in vec3 uvParam;   // unwrapped sphere params (phi/2pi, theta/pi), see create_atmosphere_mesh

out vec3 worldPos0;
out vec3 worldNormal0;
out vec2 uv0;
out float logz;

uniform mat4 MVP;
uniform mat4 Normal;   // actually the Model matrix (same convention as terrain)

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    worldPos0 = (Normal * vec4(position, 1.0)).xyz;
    worldNormal0 = (Normal * vec4(normal, 0.0)).xyz;

    // Equirectangular UV (the layout the coverage bake uses; the baker in
    // TerrainBody::BuildClouds must agree): lon = atan2(x, z) is the game's
    // convention (lon 0 = +Z), row 0 = north pole (v=0). Taken from the
    // mesh's UNWRAPPED sphere params (color slot), not atan(position):
    // atan has a branch cut, and interpolating u across it stretches one
    // meridian of the coverage map into a pole-to-pole smear. With phi
    // unwrapped, u = 0.75 - phi/2pi varies continuously over [-0.25, 0.75]
    // and the texture's REPEAT wrap closes the seam. v = theta/pi.
    uv0 = vec2(0.75 - uvParam.x, uvParam.y);

    // Logarithmic depth — must stay identical to terrainShader.vs (C=11,
    // far=1e13) or the depth test against the terrain and the far-plane
    // skybox breaks. See reports/atmosphere2026_08_25.
    const float C = 11;
    const float far = 1e13;
    const float FC = 1.0 / log(far * C + 1);

    logz = log(gl_Position.w * C + 1) * FC;
    gl_Position.z = (2.0 * logz - 1.0) * gl_Position.w;
}
