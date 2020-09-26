#version 450 

layout(set = 0, binding = 7) uniform sampler2D customSampler;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

void main() {
  outColor = vec4(texture(customSampler, inUV).xyz, 1.f);
}
