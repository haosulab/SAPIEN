#version 450

layout(set = 0, binding = 0) uniform sampler2D samplerLightingSSRBlur;
layout(set = 0, binding = 1) uniform sampler2D samplerLighting;

layout (location = 0) in vec2 inUV;
layout (location = 0) out vec4 outLightingSSR;

void main()
{
  vec4 ssr = texture(samplerLightingSSRBlur, inUV);
  vec4 light = texture(samplerLighting, inUV);

  outLightingSSR = light + vec4(ssr.rgb * ssr.a, 0.);
}
