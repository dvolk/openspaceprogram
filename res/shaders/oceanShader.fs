#version 450

in vec3 worldPos0;
in vec3 worldNormal0;
in vec3 bodyPos0;

out vec4 fragColor;

uniform vec3 cameraPos;
uniform vec3 seaColor;
uniform vec3 lightDirection;   // direction light travels (sun -> planet)
uniform float time;
uniform vec3 planetCenter;
uniform mat4 Normal;           // body-to-world (shared with VS)

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 V = normalize(worldPos0 - cameraPos);   // camera -> surface
    vec3 sunDir = -lightDirection;               // towards the sun

    // Wave normal perturbation: three sine pairs at different scales and
    // speeds, computed in body space so the pattern is fixed to the planet
    // surface and rotates with it. Faded out with camera distance -- the
    // sine frequencies alias into moiré once a pixel covers more than a
    // wavelength (roughly above ~50 km), so the ocean goes flat from orbit.
    float dist = length(worldPos0 - cameraPos);
    float waveFade = 1.0 - smoothstep(20000.0, 80000.0, dist);
    vec3 p = bodyPos0;
    float w1 = sin(p.x * 0.031 + time * 1.2) * cos(p.z * 0.027 + time * 0.9);
    float w2 = sin(p.z * 0.019 - time * 0.7) * cos(p.y * 0.023 + time * 1.1);
    float w3 = sin(p.y * 0.013 + time * 0.5) * cos(p.x * 0.017 - time * 0.8);
    vec3 waveN = vec3(w1 + w3 * 0.5, w2 * 0.7, w2 + w1 * 0.5);
    vec3 wn = mat3(Normal) * waveN;
    wn -= dot(wn, N) * N;
    N = normalize(N + wn * 0.04 * waveFade);

    // Fresnel: water reflects more at glancing angles (horizon) and is
    // more transparent looking straight down.
    float cosTheta = clamp(dot(N, -V), 0.0, 1.0);
    float fresnel = pow(1.0 - cosTheta, 4.0);

    // Specular sun highlight (Blinn-Phong, tight lobe for a sharp glint).
    vec3 H = normalize(sunDir - V);
    float spec = pow(max(dot(N, H), 0.0), 512.0) * 3.0;

    // Depth approximation: looking straight down = more water between
    // camera and sea floor = darker.  Glancing = shallow = lighter.
    vec3 deepColor = seaColor * 0.25;
    vec3 waterColor = mix(seaColor, deepColor, cosTheta);

    // Sun lighting + a dim ambient so the night side isn't pure black.
    float diff = max(dot(N, sunDir), 0.0);
    vec3 color = waterColor * (0.06 + diff * 0.94)
               + vec3(1.0, 0.97, 0.92) * spec;

    // Alpha: mostly opaque (you barely see the sea floor from orbit),
    // fully reflective at the horizon.
    float alpha = mix(0.82, 0.98, fresnel);

    fragColor = vec4(color, alpha);
}
