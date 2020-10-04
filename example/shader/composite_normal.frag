#version 450 

layout(set = 0, binding = 4) uniform sampler2D normalSampler;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

void main() {
  outColor = abs(texture(normalSampler, inUV));
}
