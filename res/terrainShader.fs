#version 450

in vec3 normal0;
in vec4 color0;
in vec3 up0;
in vec3 bodyPos0;
in vec3 bodyNormal0;

out vec4 fragColor;

uniform sampler2D detailTex;
uniform vec3 lightDirection;
uniform vec4 color;

// Triplanar projection: sample the detail texture on the three
// body-space planes, blend by the normal's dominant axis. The texture
// tiles seamlessly (torus-wrapped noise), so REPEAT wrap is clean.
// Scale: 64 m per tile (matches gen_terrain_detail.py's world periods).
float triplanar(vec3 pos, vec3 norm) {
    const float scale = 1.0 / 64.0;
    vec3 blend = abs(norm);
    blend = max(blend - 0.2, 0.0);
    blend /= (blend.x + blend.y + blend.z + 0.00001);

    float x = texture(detailTex, pos.yz * scale).r;
    float y = texture(detailTex, pos.xz * scale).r;
    float z = texture(detailTex, pos.xy * scale).r;

    return x * blend.x + y * blend.y + z * blend.z;
}

void main()
{
    vec3 N = normalize(normal0);
    vec3 up = normalize(up0);
    float NdotL = dot(-lightDirection, N);
    float diffuse = max(NdotL, 0.0);

    // Hemisphere ambient: sky-facing surfaces get a cool skylight,
    // downward-facing surfaces get a dim warm ground bounce.
    // Sky light fades when the sun is below the horizon (no scatter).
    float sky = dot(N, up) * 0.5 + 0.5;
    float sunUp = smoothstep(-0.1, 0.3, dot(up, -lightDirection));
    vec3 ambient = mix(vec3(0.02, 0.018, 0.015),
                       vec3(0.08, 0.09, 0.12) * sunUp, sky);

    // Detail texture: darken-only modulation of the baked vertex colour.
    // The map is centred on 255 (identity) with a floor of 90, so it
    // never re-colours sea or gas-giant bands -- only adds ground grain.
    float detail = triplanar(bodyPos0, normalize(bodyNormal0));
    vec3 albedo = color0.rgb * mix(1.0, detail, 0.5);

    fragColor = vec4(albedo * (diffuse + ambient), 1.0);
}
