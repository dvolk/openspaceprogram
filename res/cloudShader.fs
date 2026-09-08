#version 450

in vec3 worldPos0;
in vec3 worldNormal0;
in vec3 localDir;
in float logz;

out vec4 fragColor;

uniform vec3 cameraPos;
uniform vec3 color;           // cloud tint (white)
uniform vec3 lightDirection;  // direction light travels (sun -> planet), same as the terrain
uniform mat3 seedRot;         // the body's noise orientation (the terrain's seed)
uniform float freq;           // pattern scale in unit-sphere coordinates
uniform float drift;          // pattern offset (sim time * wind speed)
uniform float coverage;       // 0..1, how much of the deck is cloudy
uniform vec3 planetCenter;    // body centre, world coords (for the camera's local vertical)

// Value noise (8 corner hashes + trilinear): cheaper per fragment than
// simplex and the deck only draws where visible, so the cost is bounded.
float hash13(vec3 p) {
    p = fract(p * 0.3183099);
    p += dot(p, p.zyx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float vnoise(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = hash13(i + vec3(0.0, 0.0, 0.0));
    float b = hash13(i + vec3(1.0, 0.0, 0.0));
    float c = hash13(i + vec3(0.0, 1.0, 0.0));
    float d = hash13(i + vec3(1.0, 1.0, 0.0));
    float e = hash13(i + vec3(0.0, 0.0, 1.0));
    float g = hash13(i + vec3(1.0, 0.0, 1.0));
    float h = hash13(i + vec3(0.0, 1.0, 1.0));
    float n = hash13(i + vec3(1.0, 1.0, 1.0));
    return mix(mix(mix(a, b, f.x), mix(c, d, f.x), f.y),
               mix(mix(e, g, f.x), mix(h, n, f.x), f.y), f.z);
}

float fbm(vec3 p) {
    float v = 0.0;
    float amp = 0.5;
    for(int i = 0; i < 4; i++) {
        v += amp * vnoise(p);
        p = p * 2.03 + vec3(11.7, 7.3, 5.9);
        amp *= 0.5;
    }
    return v;
}

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 V = normalize(worldPos0 - cameraPos);   // camera -> surface point
    vec3 sunDir = -lightDirection;               // direction towards the sun

    // Coverage: FBM in the body's noise frame (localDir is the deck
    // direction in the body's own frame, seedRot its per-body orientation),
    // drifting along x with `drift` (the wind, relative to the surface).
    vec3 p = seedRot * localDir * freq;
    p.x += drift;
    float T = mix(0.78, 0.38, coverage);
    float cover = smoothstep(T - 0.12, T + 0.12, fbm(p));

    // Lambert with a small ambient: the deck is white, so its terminator
    // falls where the terrain's does (the same lightDirection).
    float diff = clamp(dot(N, sunDir), 0.0, 1.0);

    // Day/night is set by the sun's altitude AT THE CAMERA (a global
    // factor, like the atmosphere): the deck fades on the night side so
    // it never reads as a black disc over the starfield.
    vec3 camUp = normalize(cameraPos - planetCenter);
    float sunAlt = dot(sunDir, camUp);
    float day = smoothstep(-0.1, 0.25, sunAlt);

    vec3 base = color * (0.12 + 0.95 * diff);
    // Warm edge when the sun sits behind the deck (forward scatter), the
    // same glow the atmosphere shell adds to its rim.
    float back = max(0.0, dot(sunDir, V));
    back *= back;
    base += back * vec3(1.0, 0.5, 0.25) * 0.5;

    fragColor = vec4(base, cover * mix(0.35, 1.0, day));
    gl_FragDepth = logz;   // must match the vertex shader / terrain log-depth
}
