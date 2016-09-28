#version 120

varying vec3 normal0;
varying vec2 uv0;
varying float logz;

uniform vec3 lightDirection;

uniform sampler2D texture;

void main()
{
    vec2 uv1 = uv0;
    uv1.y = 1 - uv1.y; // ??
    vec4 tex_color = texture2D(texture, uv1);
    const float min_light = 0.15;
    const float max_light = 1.0;
    gl_FragColor = tex_color * clamp(dot(-lightDirection, normal0), min_light, max_light);
    gl_FragDepth = logz;
}
