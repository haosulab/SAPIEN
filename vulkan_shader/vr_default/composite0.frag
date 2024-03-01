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

vec3 sRGB(vec3 x) {
  bvec3 cutoff = lessThan(x, vec3(0.0031308));
  vec3 higher = vec3(1.055) * pow(x, vec3(1.0/2.4)) - vec3(0.055);
  vec3 lower = x * vec3(12.92);
  return clamp(mix(higher, lower, cutoff), 0.0, 1.0);
}

const mat3 ACESInputMat = mat3(
    0.59719, 0.35458, 0.04823,
    0.07600, 0.90834, 0.01566,
    0.02840, 0.13383, 0.83777
);

const mat3 ACESOutputMat = mat3(
     1.60475, -0.53108, -0.07367,
    -0.10208,  1.10813, -0.00605,
    -0.00327, -0.07276,  1.07602
);

vec3 RRTAndODTFit(vec3 v)
{
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

vec3 ACESsRGB(vec3 color) {
    color = color * ACESInputMat;
    color = RRTAndODTFit(color);
    color = color * ACESOutputMat;
    color = clamp(color, 0.0, 1.0);
    return sRGB(color);
}

void main() {
  outColor = texture(samplerLighting, inUV);
  outColor.rgb = pow((outColor.rgb) * exposure, vec3(1/2.2));
  outColor = clamp(outColor, vec4(0), vec4(1));

  vec3 position = texture(samplerPositionRaw, inUV).xyz;
  outDepthLinear.x = position.z;

  float depth = texture(samplerGbufferDepth, inUV).x;

  uvec2 seg = texture(samplerSegmentation, inUV).xy;
  outSegmentationView0 = mix(vec4(0,0,0,1), colors[seg.x % 60], sign(seg.x));
  outSegmentationView1 = mix(vec4(0,0,0,1), colors[seg.y % 60], sign(seg.y));

  vec4 lineColor = texture(samplerLine, inUV);
  float lineDepth = texture(samplerLineDepth, inUV).x;
  if (lineDepth < depth) {
    outColor = vec4(mix(outColor.rgb, lineColor.rgb, lineColor.a), 1);
    depth = lineDepth;
  }

  vec4 pointColor = texture(samplerPoint, inUV);
  if (texture(samplerPointDepth, inUV).x < 1) {
    outColor = vec4(pointColor.xyz, 1);
  }

  outPosition = vec4(position, texture(samplerGbufferDepth, inUV).x);
}
