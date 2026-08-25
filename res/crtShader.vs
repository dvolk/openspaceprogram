#version 450

in vec2 position;
in vec2 uv;
out vec2 texcoord0;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    texcoord0 = uv;
}
