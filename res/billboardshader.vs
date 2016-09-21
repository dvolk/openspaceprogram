#version 120

attribute vec3 position;
attribute vec3 normal;
attribute vec3 color;

uniform mat4 MVP;

varying vec4 color0;
varying float logz;

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    color0 = vec4(color, 1.0);

    const float C=11;
    const float far = 100000000000;
    const float FC = 1.0/log(far*C + 1);
            
    logz = log(gl_Position.w * C + 1) * FC;
    gl_Position.z = (2 * logz - 1) * gl_Position.w;
}
