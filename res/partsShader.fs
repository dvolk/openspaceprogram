#version 450

in vec3 normal0;
in vec2 uv0;

out vec4 fragColor;

uniform vec3 lightDirection;
uniform float shadow;

/* Authoring overrides (VAB): alpha < 1 draws a translucent ghost; tint
   multiplies the lit color for selection highlighting. Defaults (1.0, white)
   reproduce the plain opaque part. */
uniform float alpha;
uniform vec3 tint;

uniform sampler2D mytexture;

void main()
{
    vec2 uv1 = uv0;
    uv1.y = 1 - uv1.y; // ??
    vec4 tex_color = texture(mytexture, uv1);
    const float min_light = 0.15;
    const float max_light = 1.0;
    float light = clamp(dot(-lightDirection, normal0), min_light, max_light);
    // Alpha is written from the uniform, NOT from the lit color: scaling the
    // framebuffer alpha by light/shadow leaks the scene's lighting into it
    // (invisible in-game, but the F12 screenshot saved shaded parts as
    // translucent). Opaque parts keep alpha = 1.
    fragColor = vec4(tex_color.rgb * light * shadow * tint, alpha);
}
