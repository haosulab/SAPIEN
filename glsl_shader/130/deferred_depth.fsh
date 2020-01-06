#version 130
#extension GL_ARB_explicit_attrib_location : enable

uniform sampler2D depthtex0;  // depth

in vec2 texcoord;
out vec4 FragColor;
void main() {
  FragColor = vec4(texture(depthtex0, texcoord).xxx, 1);
}
