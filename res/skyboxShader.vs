#version 450

in vec3 position;
out vec3 texcoord0;

uniform mat4 projectionview;

void main()
{
    vec4 pos = projectionview * vec4(position, 1.0);
    // Far-plane cube for the cubemap lookup, at the REVERSE-Z far depth (0.0):
    // nearer fragments have the larger depth, so 0.0 is the farthest value and
    // the terrain (depth > 0.0) occludes it under the global GEQUAL test.
    // Same xy as the old `pos.xyww` trick; only the far extreme is flipped
    // (1.0 was far under standard Z, 0.0 is far under reverse-Z).
    gl_Position = vec4(pos.xy, 0.0, pos.w);
    texcoord0 = position;
}  
