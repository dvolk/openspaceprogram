#version 120

varying vec4 color0;
varying vec2 texcoord0;

uniform sampler2D mytexture;

void main()
{
    vec4 tex_color = texture2D(mytexture, texcoord0);
    if(tex_color.a > 0.01) {
        gl_FragColor = color0 * tex_color.a;
    }
    else {
        discard;
    }
}
