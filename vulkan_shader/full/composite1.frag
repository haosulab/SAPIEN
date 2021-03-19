#version 450

layout(set = 0, binding = 0) uniform sampler2D samplerLightingSSR;

layout (location = 0) in vec2 inUV;
layout (location = 0) out vec4 outLightingSSRBlur;

const float RADIUS = 3;

void main() {
  vec2 resolution = textureSize(samplerLightingSSR, 0);
  float windowSize = RADIUS * 2 + 1;
  float samples = windowSize * windowSize;

  vec4 color = vec4(0);

  for (float ry = -RADIUS; ry <= RADIUS; ry += 1) {
    for (float rx = -RADIUS; rx <= RADIUS; rx += 1)
    {
    	color += texture(samplerLightingSSR, inUV + vec2(rx,ry)/resolution);
    }
  }
  color /= samples;
  outLightingSSRBlur = color;
}
