#version 130
#extension GL_ARB_explicit_attrib_location : enable

layout (location=0) out vec4 GCOLOR;
uniform vec3 color;

void main() {
  GCOLOR = vec4(color, 1);
}
