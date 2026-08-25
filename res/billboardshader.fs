#version 450

in vec4 color0;
in vec2 texcoord0;

out vec4 fragColor;

uniform sampler2D mytexture;

void main()
{
    vec4 tex_color = texture(mytexture, texcoord0);
    if(tex_color.a > 0.01) {
        fragColor = color0 * tex_color.a;
    }
    else {
        discard;
    }
}
