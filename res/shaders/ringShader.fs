#version 450

in vec3 worldNormal0;

out vec4 fragColor;

uniform vec3 lightDirection;  // direction light travels (sun -> planet), same as the terrain
uniform float albedo;         // band brightness
uniform float opacity;        // band transparency (0..1)

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 sunDir = -lightDirection;               // direction towards the sun

    // Two-sided Lambert: a flat annulus is seen from both faces (culling is
    // off in DrawRings), and the sun can sit above or below the ring plane,
    // so use the absolute dot -- the ring lights from either side instead of
    // going dark when the sun is on the "wrong" side of the plane.
    float diff = clamp(abs(dot(N, sunDir)), 0.0, 1.0);

    // A little ambient so the ring's night side isn't a black disc over the
    // starfield (the same floor the cloud deck uses).
    vec3 base = vec3(albedo) * (0.10 + 0.90 * diff);

    fragColor = vec4(base, opacity);
}
