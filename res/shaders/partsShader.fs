#version 450

in vec3 normal0;
in vec2 uv0;

out vec4 fragColor;

uniform vec3 lightDirection;
// 1 = sunlit, 0 = inside a body's shadow (see ComputeTerrainShadow).
uniform float shadow;

/* Authoring overrides (VAB): alpha < 1 draws a translucent ghost; tint
   multiplies the lit color for selection highlighting; flat = 1 replaces
   the directional light with uniform studio lighting (the editor look).
   Defaults (1.0, white, 0.0) reproduce the plain opaque flight part. */
uniform float alpha;
uniform vec3 tint;
uniform float flatLight;   // "flat" is a reserved GLSL interpolation keyword

uniform sampler2D mytexture;

void main()
{
    vec2 uv1 = uv0;
    uv1.y = 1 - uv1.y; // ??
    vec4 tex_color = texture(mytexture, uv1);
    const float min_light = 0.15;
    const float max_light = 1.0;
    /* In a body's shadow there is no sun left to shape the hull, so the
       directional gradient goes away entirely rather than just dimming: a
       shadowed ship must not keep a bright side pointing at the occluded
       sun. 0.03 sits at or below every other night floor in the scene
       (terrain 0-0.02, ocean 0.06, rings 0.10, clouds 0.12), so a night
       hull is the darkest thing in view. */
    const float night_light = 0.03;
    float light = clamp(dot(-lightDirection, normal0), min_light, max_light);
    light = mix(light, max_light, flatLight);
    light = mix(night_light, light, shadow);
    // Alpha is written from the uniform, NOT from the lit color: scaling the
    // framebuffer alpha by light/shadow leaks the scene's lighting into it
    // (invisible in-game, but the F12 screenshot saved shaded parts as
    // translucent). Opaque parts keep alpha = 1.
    fragColor = vec4(tex_color.rgb * light * tint, alpha);
}
