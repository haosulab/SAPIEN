#version 450 

layout(set = 0, binding = 0) uniform sampler2D samplerLighting2;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outLighting;

void main() {
  outLighting = pow(texture(samplerLighting2, inUV), vec4(1/2.2, 1/2.2, 1/2.2, 1));
}
