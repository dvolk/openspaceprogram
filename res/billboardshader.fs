#version 120

varying vec4 color0;
varying vec2 texcoord0;

uniform sampler2D mytexture;

void main()
{
    if(color0.a < 0.5) {
        discard;
    }
    else {
        gl_FragColor = texture2D(mytexture, texcoord0);
    }
}
