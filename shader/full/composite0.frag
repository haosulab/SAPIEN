#version 450

layout(set = 0, binding = 0) uniform sampler2D samplerLighting;
layout(set = 0, binding = 1) uniform sampler2D samplerLighting1;
layout(set = 0, binding = 2) uniform sampler2D samplerAlbedo2;
layout(set = 0, binding = 3) uniform sampler2D samplerGbufferDepth;
layout(set = 0, binding = 4) uniform sampler2D samplerGbuffer1Depth;
layout(set = 0, binding = 5) uniform sampler2D samplerGbuffer2Depth;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outDepthView;

void main() {
  float d0 = texture(samplerGbufferDepth, inUV).x;
  float d1 = texture(samplerGbuffer1Depth, inUV).x;
  float d2 = texture(samplerGbuffer2Depth, inUV).x;

  vec4 outColor0 = texture(samplerLighting, inUV);
  vec4 outColor1 = texture(samplerLighting1, inUV);
  vec4 outColor2 = texture(samplerAlbedo2, inUV);

  // depth composite for 0 and 2
  float factor = step(d0, d2);
  outColor0 = outColor0 * factor + outColor2 * (1 - factor);

  // blend for 02 and 1
  vec3 blend = outColor1.a * outColor1.rgb + (1 - outColor1.a) * outColor0.rgb;
  factor = step(min(d0, d2), d1);
  outColor = vec4((1 - factor)* blend + factor * outColor0.rgb, 1.f);

  outColor = pow(outColor, vec4(1/2.2, 1/2.2, 1/2.2, 1));
  outDepthView = vec4(vec3(d0), 1);
}
