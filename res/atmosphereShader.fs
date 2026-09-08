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
uniform float inside;          // 1 when the camera is inside the shell (surface view)
uniform vec3 planetCenter;     // body centre, world coords (for the camera's local vertical)

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 V = normalize(worldPos0 - cameraPos);   // camera -> surface point
    vec3 sunDir = -lightDirection;               // direction towards the sun
    vec3 sky;
    float a;

    if(inside > 0.5) {
        // Surface view: the surviving fragments are the air above and
        // around the camera (the near wall wins the depth test against
        // the far side, so the sky reads as the local atmosphere, not
        // the planet's far limb). Haze grows with slant path: 0 straight
        // overhead (N ~ V), 1 at the horizon (N ~ V perpendicular).
        float haze = clamp(1.0 - dot(N, V), 0.0, 1.0);

        // Day/night is set by the sun's altitude AT THE CAMERA (a global
        // factor), not by each point's own sun incidence: on a day side
        // the whole dome is lit, including the anti-sun limb, so driving
        // opacity per-point let the starfield bleed through the blue. The
        // per-point facing below only shifts the colour (sun side bright,
        // anti-sun deep). Night falls to 0 so the starfield shows.
        vec3 camUp = normalize(cameraPos - planetCenter);
        float sunAlt = dot(sunDir, camUp);                 // -1..+1
        float skyDay = smoothstep(-0.08, 0.22, sunAlt);    // 0 night .. 1 day

        float local = dot(sunDir, N);                      // -1..+1 point facing
        float glow = max(0.0, dot(sunDir, V)); glow *= glow;  // towards the sun

        sky = mix(color, vec3(1.0), 0.30 * (0.5 + 0.5 * local));  // sun side bright
        sky = mix(sky, vec3(0.85, 0.90, 1.0), 0.45 * haze);       // horizon haze
        sky += glow * vec3(1.0, 0.55, 0.30) * 0.55;               // warm sun glow

        float aBase = skyDay * (0.9 + 0.35 * haze) * (0.8 + 0.2 * (0.5 + 0.5 * local));
        // Dawn/dusk + daytime glare toward the sun; fades out on the true
        // night side (sun well below the horizon) so it never neons the dark.
        a = max(aBase, glow * 0.35 * clamp(sunAlt * 2.0 + 0.5, 0.0, 1.0));
        a *= intensity * 2.0;
    } else {
        // Orbital view: Fresnel limb term, 0 at the disc centre, 1 at the
        // limb. The near hemisphere is strictly in front of the terrain, so
        // this blends a transparent centre into a bright rim (the
        // atmospheric ring) and a horizon haze over the surface.
        float rim = clamp(1.0 + dot(N, V), 0.0, 1.0);

        // Day/night (Lambertian): the atmosphere catches the sun a little
        // *before* the surface does (the dawn/dusk glow), so shift its ramp
        // earlier than the terrain's (clamp(dot, 0.05, 1)) by PRELIGHT. The
        // true night side still falls to a faint ambient so it doesn't glow
        // uniformly.
        const float PRELIGHT = 0.35;
        float sunFace = clamp(dot(-lightDirection, N) + PRELIGHT, 0.02, 1.0);

        // Backlit halo (forward scattering): when the sun sits *behind* the
        // planet, the limb glows all round even though that side is night.
        // That is the case sunFace can't light (its normal faces away from
        // the sun), so add a term that peaks when the sun is directly
        // behind the point we see. It is zero on the day side (sun in front)
        // and on the true night side (sun to the side), so it never creates
        // a uniform neon ring.
        const float HALO = 0.8;
        float backlit = max(0.0, dot(-lightDirection, V)) * HALO;

        a = pow(rim, power) * intensity * max(sunFace, backlit);
        sky = color;
    }

    fragColor = vec4(sky, clamp(a, 0.0, 1.0));
    gl_FragDepth = logz;   // must match the vertex shader / terrain log-depth
}
