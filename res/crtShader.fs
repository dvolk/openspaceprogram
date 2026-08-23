#version 120

varying vec2 texcoord0;

uniform sampler2D scene;
uniform vec2 resolution;
uniform float time;

const float PI = 3.14159265;

// Barrel distortion: shift the sample point outward with radius so the
// image bulges like a CRT tube. k is the strength.
vec2 distort(vec2 c, float k)
{
    float aspect = resolution.x / resolution.y;
    vec2 ac = vec2(c.x * aspect, c.y);
    float r2 = dot(ac, ac);
    return c + c * r2 * k;
}

void main()
{
    vec2 c = texcoord0 - 0.5;

    // RGB fringing: sample each channel with a slightly different barrel
    // strength so edges get a red/blue fringe. distort() works in centered
    // coords, so map back to [0,1] texture space before sampling.
    vec3 col;
    col.r = texture2D(scene, distort(c, 0.080) + 0.5).r;
    col.g = texture2D(scene, distort(c, 0.100) + 0.5).g;
    col.b = texture2D(scene, distort(c, 0.120) + 0.5).b;

    // Where the distorted sample (centered coords) falls outside
    // [-0.5, 0.5] (the corners), fade to black like the rounded panel of
    // a real tube.
    vec2 d = distort(c, 0.100);
    float oob = max(abs(d.x) - 0.5, abs(d.y) - 0.5) * 2.0;
    float panel = 1.0 - smoothstep(0.0, 0.12, oob);

    // Scanlines: one per ~2 px of vertical resolution.
    float scan = 0.93 + 0.07 * sin(texcoord0.y * resolution.y * PI);

    // Soft vignette (aspect-corrected radius).
    float aspect = resolution.x / resolution.y;
    vec2 ac = vec2(c.x * aspect, c.y);
    float vig = 1.0 - 0.25 * dot(ac, ac);

    // Slow, subtle flicker.
    float flicker = 0.985 + 0.015 * sin(time * 2.1) * sin(time * 5.3);

    gl_FragColor = vec4(col * scan * vig * panel * flicker, 1.0);
}
