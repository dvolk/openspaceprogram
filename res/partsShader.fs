#version 450

in vec3 normal0;
in vec2 uv0;

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
    float light = clamp(dot(-lightDirection, normal0), min_light, max_light);
    // Opaque part: write alpha=1 explicitly. `tex_color * ...` would scale
    // the alpha by the light (and shadow), leaking the scene's lighting
    // into the framebuffer's alpha channel -- invisible in-game, but the
    // F12 screenshot saved shaded parts as translucent.
    fragColor = vec4(tex_color.rgb * light * shadow, 1.0);
}
