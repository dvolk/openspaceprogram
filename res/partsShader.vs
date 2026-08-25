#version 450

in vec3 position;
in vec2 uv;
in vec3 normal;

out vec3 normal0;
out vec2 uv0;
out float logz;

uniform mat4 MVP;
uniform mat4 Normal;

void main()
{

    gl_Position = MVP * vec4(position, 1.0);
    normal0 = (Normal * vec4(normal, 0.0)).xyz;

    // depth buffer hack. see:
    // http://outerra.blogspot.com/2009/08/logarithmic-z-buffer.html

    const float C=11;
    // 1e13, NOT 10000000000000: the GLSL compiler truncates int literals > 2^32
    // to 32 bits (measured far = 1.3e9), which clipped far bodies to depth 1.0.
    const float far = 1e13;
    const float FC = 1.0/log(far*C + 1);

    logz = log(gl_Position.w * C + 1) * FC;
    gl_Position.z = (2 * logz - 1) * gl_Position.w;
    uv0 = uv;
}
