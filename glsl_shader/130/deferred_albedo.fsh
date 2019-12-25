#version 130
#extension GL_ARB_explicit_attrib_location : enable

in vec2 texcoord;

uniform sampler2D colortex0;  // albedo

out vec4 FragColor;
void main() {
  FragColor = vec4(texture(colortex0, texcoord).rgb, 1);
}
