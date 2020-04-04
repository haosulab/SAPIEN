#version 130
#extension GL_ARB_explicit_attrib_location : enable

uniform sampler2D colortex2;  // normal

in vec2 texcoord;
out vec4 FragColor;

void main() {
  vec3 normal = texture(colortex2, texcoord).xyz;
  FragColor = vec4(normal, 1.f);
}
