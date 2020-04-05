#version 410

layout (location=0) out vec4 GCOLOR;
uniform vec3 color;

void main() {
  GCOLOR = vec4(color, 1);
}
