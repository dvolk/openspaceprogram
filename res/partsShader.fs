#version 450

in vec3 normal0;
in vec2 uv0;
in float logz;

out vec4 fragColor;

uniform vec3 lightDirection;
uniform float shadow;

uniform sampler2D mytexture;

void main()
{
    vec2 uv1 = uv0;
    uv1.y = 1 - uv1.y; // ??
    vec4 tex_color = texture(mytexture, uv1);
    const float min_light = 0.15;
    const float max_light = 1.0;
    fragColor = tex_color * clamp(dot(-lightDirection, normal0), min_light, max_light) * shadow;
    gl_FragDepth = logz;
}
