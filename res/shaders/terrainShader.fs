#version 450

in vec3 normal0;
in vec4 color0;
in vec3 up0;
in vec3 bodyPos0;
in vec3 bodyNormal0;

out vec4 fragColor;

uniform sampler2D detailTex;
uniform mat4 Normal;   // body-to-world (shared with vertex shader)
uniform vec3 lightDirection;
uniform vec4 color;

// Triplanar projection: sample the detail texture on the three
// body-space planes, blend by the normal's dominant axis. The texture
// tiles seamlessly (torus-wrapped noise), so REPEAT wrap is clean.
//
// Two scales, sampled and multiplied in main(): the landed layer
// (512 m tile -> ~4-256 m mottling, the ground read) and the flying
// layer (65536 m tile -> ~512 m-32 km, regional tonal patches). Both
// are darkening-only (the map is centred on 255), so their product
// still only darkens -- it can never re-colour sea or gas-giant bands.
const float texScaleLanded = 1.0 / 512.0;
const float texScaleFlying = 1.0 / 65536.0;

float triplanar(vec3 pos, vec3 norm, float scale) {
    vec3 blend = abs(norm);
    blend = max(blend - 0.2, 0.0);
    blend /= (blend.x + blend.y + blend.z + 0.00001);

    float x = texture(detailTex, pos.yz * scale).r;
    float y = texture(detailTex, pos.xz * scale).r;
    float z = texture(detailTex, pos.xy * scale).r;

    return x * blend.x + y * blend.y + z * blend.z;
}

// Body-space height gradient from the dominant projection only (2 extra
// texture samples). Adjacent terrain fragments almost always share the
// same dominant axis, so the branch is warp-coherent. Driven by the
// LANDED scale only: at the flying scale the eps step is sub-texel and
// the finite difference would read as noise.
vec3 detailGradient(vec3 pos, vec3 norm, float scale) {
    vec3 blend = abs(norm);
    const float eps = 0.5;
    vec3 grad = vec3(0.0);

    if (blend.x >= blend.y && blend.x >= blend.z) {
        float h0 = texture(detailTex, pos.yz * scale).r;
        float hy = texture(detailTex, (pos.yz + vec2(eps, 0)) * scale).r;
        float hz = texture(detailTex, (pos.yz + vec2(0, eps)) * scale).r;
        grad = vec3(0, hy - h0, hz - h0) / eps;
    } else if (blend.y >= blend.z) {
        float h0 = texture(detailTex, pos.xz * scale).r;
        float hx = texture(detailTex, (pos.xz + vec2(eps, 0)) * scale).r;
        float hz = texture(detailTex, (pos.xz + vec2(0, eps)) * scale).r;
        grad = vec3(hx - h0, 0, hz - h0) / eps;
    } else {
        float h0 = texture(detailTex, pos.xy * scale).r;
        float hx = texture(detailTex, (pos.xy + vec2(eps, 0)) * scale).r;
        float hy = texture(detailTex, (pos.xy + vec2(0, eps)) * scale).r;
        grad = vec3(hx - h0, hy - h0, 0) / eps;
    }
    return grad;
}

void main()
{
    vec3 N = normalize(normal0);
    vec3 up = normalize(up0);
    vec3 bNorm = normalize(bodyNormal0);

    // Detail normal: perturb the shading normal with the height gradient
    // so micro-relief catches light. Gradient is body-space; rotate to
    // world via mat3(Normal), project onto the tangent plane, subtract.
    vec3 grad = mat3(Normal) * detailGradient(bodyPos0, bNorm, texScaleLanded);
    grad -= dot(grad, N) * N;
    vec3 Nd = normalize(N - grad * 0.15);

    float NdotL = dot(-lightDirection, Nd);
    float diffuse = max(NdotL, 0.0);

    // Hemisphere ambient: sky-facing surfaces get a cool skylight,
    // downward-facing surfaces get a dim warm ground bounce.
    // Sky light fades when the sun is below the horizon (no scatter).
    float sky = dot(Nd, up) * 0.5 + 0.5;
    float sunUp = smoothstep(-0.1, 0.3, dot(up, -lightDirection));
    vec3 ambient = mix(vec3(0.02, 0.018, 0.015),
                       vec3(0.08, 0.09, 0.12) * sunUp, sky);

    // Detail texture: darken-only modulation of the baked vertex colour.
    // Two scales multiplied (see texScaleLanded/Flying): the landed layer
    // is the ground read, the flying layer adds regional tonal patches.
    // The map is centred on 255 (identity) with a floor of 90, so the
    // product never re-colours sea or gas-giant bands -- only darkens.
    float detail = triplanar(bodyPos0, bNorm, texScaleLanded)
                 * triplanar(bodyPos0, bNorm, texScaleFlying);
    vec3 albedo = color0.rgb * mix(1.0, detail, 0.4);

    fragColor = vec4(albedo * (diffuse + ambient), 1.0);
}
