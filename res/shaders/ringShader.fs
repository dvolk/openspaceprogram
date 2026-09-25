#version 450

in vec3 worldNormal0;
in vec3 localPos0;

out vec4 fragColor;

uniform vec3 lightDirection;  // direction light travels (sun -> planet), same as the terrain
uniform mat4 Normal;          // body-to-world (Model); pulls the sun into body space
uniform float albedo;         // band brightness
uniform float opacity;        // band transparency (0..1)
uniform float planetRadius;   // [m] occluding sphere at the body centre

void main()
{
    vec3 N = normalize(worldNormal0);
    vec3 sunDir = -lightDirection;               // direction towards the sun

    // Two-sided Lambert: a flat annulus is seen from both faces (culling is
    // off in DrawRings), and the sun can sit above or below the ring plane,
    // so use the absolute dot -- the ring lights from either side instead of
    // going dark when the sun is on the "wrong" side of the plane.
    float diff = clamp(abs(dot(N, sunDir)), 0.0, 1.0);

    // Planet shadow: a distant sun casts a cylindrical umbra along sunDir.
    // Body-local: annulus verts live there and the planet is a sphere at the
    // origin. Ray P + t*S (t >= 0 toward the sun) vs that sphere; for a
    // directional light the occluder is a cylinder of radius planetRadius.
    vec3 S = normalize(transpose(mat3(Normal)) * sunDir);
    vec3 P = localPos0;
    float t = -dot(P, S);   // closest-approach parameter along the sunward ray
    float shadow = 1.0;
    if (t > 0.0) {
        float d = length(P + t * S);   // miss distance to the body centre
        // Soft edge ~ sun angular size * distance, with a small floor so the
        // silhouette doesn't alias into a hard binary cut at the limb.
        float soft = max(t * 0.002, planetRadius * 0.01);
        shadow = smoothstep(planetRadius - soft, planetRadius + soft, d);
    }

    // A little ambient so the ring's night side isn't a black disc over the
    // starfield. Shadow hits only the direct term so the umbra reads as
    // night, not a punched-out hole.
    vec3 base = vec3(albedo) * (0.10 + 0.90 * diff * shadow);

    fragColor = vec4(base, opacity);
}
