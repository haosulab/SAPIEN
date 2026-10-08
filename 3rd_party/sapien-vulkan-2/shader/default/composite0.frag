#version 450

layout (constant_id = 0) const float exposure = 1.0;

layout(set = 0, binding = 0) uniform sampler2D samplerLighting;
layout(set = 0, binding = 1) uniform usampler2D samplerSegmentation;
layout(set = 0, binding = 2) uniform sampler2D samplerPositionRaw;

layout(set = 0, binding = 3) uniform sampler2D samplerLineDepth;
layout(set = 0, binding = 4) uniform sampler2D samplerLine;
layout(set = 0, binding = 5) uniform sampler2D samplerPointDepth;
layout(set = 0, binding = 6) uniform sampler2D samplerPoint;
layout(set = 0, binding = 7) uniform sampler2D samplerGbufferDepth;


layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outDepthLinear;
layout(location = 2) out vec4 outSegmentationView0;
layout(location = 3) out vec4 outSegmentationView1;
layout(location = 4) out vec4 outPosition;

#define SET_NUM 1
#include "./camera_set.glsl"

vec4 colors[60] = {
  vec4(0.8,  0.4,  0.4 , 1 ),
  vec4(0.8,  0.41, 0.24, 1 ),
  vec4(0.8,  0.75, 0.32, 1 ),
  vec4(0.6,  0.8,  0.4 , 1 ),
  vec4(0.35, 0.8,  0.24, 1 ),
  vec4(0.32, 0.8,  0.51, 1 ),
  vec4(0.4,  0.8,  0.8 , 1 ),
  vec4(0.24, 0.63, 0.8 , 1 ),
  vec4(0.32, 0.37, 0.8 , 1 ),
  vec4(0.6,  0.4,  0.8 , 1 ),
  vec4(0.69, 0.24, 0.8 , 1 ),
  vec4(0.8,  0.32, 0.61, 1 ),
  vec4(0.8,  0.32, 0.32, 1 ),
  vec4(0.8,  0.64, 0.4 , 1 ),
  vec4(0.8,  0.74, 0.24, 1 ),
  vec4(0.56, 0.8,  0.32, 1 ),
  vec4(0.4,  0.8,  0.44, 1 ),
  vec4(0.24, 0.8,  0.46, 1 ),
  vec4(0.32, 0.8,  0.8 , 1 ),
  vec4(0.4,  0.56, 0.8 , 1 ),
  vec4(0.24, 0.3,  0.8 , 1 ),
  vec4(0.56, 0.32, 0.8 , 1 ),
  vec4(0.8,  0.4,  0.76, 1 ),
  vec4(0.8,  0.24, 0.58, 1 ),
  vec4(0.8,  0.24, 0.24, 1 ),
  vec4(0.8,  0.61, 0.32, 1 ),
  vec4(0.72, 0.8,  0.4 , 1 ),
  vec4(0.52, 0.8,  0.24, 1 ),
  vec4(0.32, 0.8,  0.37, 1 ),
  vec4(0.4,  0.8,  0.68, 1 ),
  vec4(0.24, 0.8,  0.8 , 1 ),
  vec4(0.32, 0.51, 0.8 , 1 ),
  vec4(0.48, 0.4,  0.8 , 1 ),
  vec4(0.52, 0.24, 0.8 , 1 ),
  vec4(0.8,  0.32, 0.75, 1 ),
  vec4(0.8,  0.4,  0.52, 1 ),
  vec4(0.8,  0.52, 0.4 , 1 ),
  vec4(0.8,  0.58, 0.24, 1 ),
  vec4(0.7,  0.8,  0.32, 1 ),
  vec4(0.48, 0.8,  0.4 , 1 ),
  vec4(0.24, 0.8,  0.3 , 1 ),
  vec4(0.32, 0.8,  0.66, 1 ),
  vec4(0.4,  0.68, 0.8 , 1 ),
  vec4(0.24, 0.46, 0.8 , 1 ),
  vec4(0.42, 0.32, 0.8 , 1 ),
  vec4(0.72, 0.4,  0.8 , 1 ),
  vec4(0.8,  0.24, 0.74, 1 ),
  vec4(0.8,  0.32, 0.46, 1 ),
  vec4(0.8,  0.46, 0.32, 1 ),
  vec4(0.8,  0.76, 0.4 , 1 ),
  vec4(0.69, 0.8,  0.24, 1 ),
  vec4(0.42, 0.8,  0.32, 1 ),
  vec4(0.4,  0.8,  0.56, 1 ),
  vec4(0.24, 0.8,  0.63, 1 ),
  vec4(0.32, 0.66, 0.8 , 1 ),
  vec4(0.4,  0.44, 0.8 , 1 ),
  vec4(0.35, 0.24, 0.8 , 1 ),
  vec4(0.7,  0.32, 0.8 , 1 ),
  vec4(0.8,  0.4,  0.64, 1 ),
  vec4(0.8,  0.24, 0.41, 1 )
};


void main() {
  outColor = texture(samplerLighting, inUV);
  outColor.rgb = pow(outColor.rgb * exposure, vec3(1/2.2));
  outColor = clamp(outColor, vec4(0), vec4(1));

  vec3 position = texture(samplerPositionRaw, inUV).xyz;

  outDepthLinear.x = position.z;

  uvec2 seg = texture(samplerSegmentation, inUV).xy;
  outSegmentationView0 = mix(vec4(0,0,0,1), colors[seg.x % 60], sign(seg.x));
  outSegmentationView1 = mix(vec4(0,0,0,1), colors[seg.y % 60], sign(seg.y));

  vec4 lineColor = texture(samplerLine, inUV);
  if (texture(samplerLineDepth, inUV).x < 1) {
    outColor = vec4(lineColor.xyz, 1);
  }

  vec4 pointColor = texture(samplerPoint, inUV);
  if (texture(samplerPointDepth, inUV).x < 1) {
    outColor = vec4(pointColor.xyz, 1);
  }

  outPosition = vec4(position, texture(samplerGbufferDepth, inUV).x);
}
