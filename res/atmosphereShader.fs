#version 450

in vec3 worldPos0;
in vec3 worldNormal0;
in float logz;

out vec4 fragColor;

uniform vec3 cameraPos;
uniform vec3 color;
uniform float intensity;
uniform float power;
uniform vec3 lightDirection;   // direction light travels (sun -> planet), same as the terrain

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 V = normalize(worldPos0 - cameraPos);   // camera -> surface point

    // Fresnel limb term: 0 at the disc centre, 1 at the limb. The near
    // hemisphere is strictly in front of the terrain, so this blends a
    // transparent centre into a bright rim (the atmospheric ring) and a
    // horizon haze over the surface. Sign/derivation in the report.
    float rim = clamp(1.0 + dot(N, V), 0.0, 1.0);

    // Day/night (Lambertian): the atmosphere catches the sun a little
    // *before* the surface does (the dawn/dusk glow), so shift its ramp
    // earlier than the terrain's (clamp(dot, 0.05, 1)) by PRELIGHT. The true
    // night side still falls to a faint ambient so it doesn't glow uniformly.
    const float PRELIGHT = 0.35;
    float sunFace = clamp(dot(-lightDirection, N) + PRELIGHT, 0.02, 1.0);

    // Backlit halo (forward scattering): when the sun sits *behind* the
    // planet, the limb glows all round even though that side is night. That
    // is the case sunFace can't light (its normal faces away from the sun),
    // so add a term that peaks when the sun is directly behind the point we
    // see. It is zero on the day side (sun in front) and on the true night
    // side (sun to the side), so it never creates a uniform neon ring.
    const float HALO = 0.8;
    float backlit = max(0.0, dot(-lightDirection, V)) * HALO;

    float a = pow(rim, power) * intensity * max(sunFace, backlit);

    fragColor = vec4(color, a);
    gl_FragDepth = logz;   // must match the vertex shader / terrain log-depth
}
