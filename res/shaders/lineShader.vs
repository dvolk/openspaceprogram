#version 450

in vec3 pos;

uniform mat4 VP;

void main() {
  gl_Position = VP * vec4(pos, 1);
}
