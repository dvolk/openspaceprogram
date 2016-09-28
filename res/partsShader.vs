#version 120

attribute vec3 position;
attribute vec2 uv;
attribute vec3 normal;

varying vec3 normal0;
varying vec2 uv0;
varying float logz;

uniform mat4 MVP;
uniform mat4 Normal;

void main()
{

    gl_Position = MVP * vec4(position, 1.0);
    normal0 = (Normal * vec4(normal, 0.0)).xyz;

    // depth buffer hack. see:
    // http://outerra.blogspot.com/2009/08/logarithmic-z-buffer.html

    const float C=11;
    const float far = 1000000000;
    const float FC = 1.0/log(far*C + 1);

    logz = log(gl_Position.w * C + 1) * FC;
    gl_Position.z = (2 * logz - 1) * gl_Position.w;
    uv0 = uv;
}
