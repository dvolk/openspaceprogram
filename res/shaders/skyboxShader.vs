#version 450

in vec3 position;
out vec3 texcoord0;

uniform mat4 projectionview;

void main()
{
    vec4 pos = projectionview * vec4(position, 1.0);
    // Far-plane cube for the cubemap lookup: push it to the reverse-Z far
    // extreme (0.0) so it isn't clipped and its xy/w still give the view
    // direction. The starfield is drawn first as a pure background (depth test
    // off, see render.cpp), so this depth only positions the cube -- the
    // bodies are painted over it, not occluded by a depth tie.
    gl_Position = vec4(pos.xy, 0.0, pos.w);
    texcoord0 = position;
}  
