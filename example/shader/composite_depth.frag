#version 450 

layout(set = 0, binding = 6) uniform sampler2D depthSampler;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

void main() {
  outColor = vec4(texture(depthSampler, inUV).xxx, 1.f);
}
