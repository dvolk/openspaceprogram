#version 330

attribute vec3 pos;

uniform mat4 VP;

void main() {
  gl_Position = VP * vec4(pos, 1);
}
