#version 450

in vec3 worldPos0;
in vec3 worldNormal0;
in vec2 uv0;
in float logz;

out vec4 fragColor;

uniform vec3 cameraPos;
uniform vec3 color;           // cloud tint (white)
uniform vec3 lightDirection;  // direction light travels (sun -> planet), same as the terrain
uniform float drift;          // horizontal UV offset (sim time * wind)
uniform vec3 planetCenter;    // body centre, world coords (for the camera's local vertical)
uniform sampler2D coverage_tex;   // the baked deck map (equirectangular R8)

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 V = normalize(worldPos0 - cameraPos);   // camera -> surface point
    vec3 sunDir = -lightDirection;               // direction towards the sun

    // Coverage: the baked map (terragen.h bakes the FBM once at load, so
    // the per-fragment cost is one texture fetch, not a per-frame FBM);
    // the drift scrolls it horizontally (the map wraps in S, and its seam
    // is one direction, so the scroll is seamless).
    float cover = texture(coverage_tex, vec2(uv0.x + drift, uv0.y)).r;

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
