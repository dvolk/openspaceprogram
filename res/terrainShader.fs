#version 450

in vec3 normal0;
in vec4 color0;
in vec3 up0;

out vec4 fragColor;

//uniform sampler2D sampler;
uniform vec3 lightDirection;
uniform vec4 color;

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

    fragColor = vec4(color0.rgb * (diffuse + ambient), 1.0);
}
