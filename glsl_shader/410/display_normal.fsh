#version 410

in vec2 texcoord;

uniform sampler2D colortex0;  // albedo
uniform sampler2D colortex1;  // 
uniform sampler2D colortex2;  // normal
uniform sampler2D colortex3;

uniform sampler2D depthtex0;  // depth

out vec4 FragColor;

void main() {
  FragColor = vec4(texture(colortex2, texcoord).xyz, 1.f);
}
