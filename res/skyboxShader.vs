#version 120

attribute vec3 position;
varying vec3 texcoord0;

uniform mat4 projectionview;

void main()
{
    vec4 pos = projectionview * vec4(position, 1.0);
    gl_Position = pos.xyww;
    texcoord0 = position;
}  
