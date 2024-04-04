#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define SET_NUM 0
#include "./camera_set.glsl"
#undef SET_NUM

#define SET_NUM 1
#include "./object_set.glsl"
#undef SET_NUM

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 tangent;
layout(location = 4) in vec3 bitangent;

layout(location = 0) out vec4 outPositionRaw;
layout(location = 1) out vec2 outUV;
layout(location = 2) out flat uvec4 outSegmentation;
layout(location = 3) out vec3 objectCoord;
layout(location = 4) out mat3 outTbn;

void ensureTbn(inout vec3 tangent, inout vec3 bitangent, inout vec3 normal) {
  if (length(tangent) < 0.01 || length(bitangent) < 0.01) {
    vec3 wx = vec3(1, 0, 0);
    if (abs(dot(normal, vec3(1, 0, 0))) > 0.95) {
      wx = vec3(0, 1, 0);
    }
    vec3 wy = cross(normal, wx);
    wx = cross(wy, normal);
    tangent = wx;
    bitangent = wy;
  }
}

void main() {
  outSegmentation = objectDataBuffer.segmentation;
  mat4 modelView = cameraBuffer.viewMatrix * objectTransformBuffer.modelMatrix;
  mat3 normalMatrix = mat3(transpose(inverse(modelView)));

  vec3 T = tangent;
  vec3 B = bitangent;
  vec3 N = normal;
  ensureTbn(T, B, N);

  objectCoord = position;
  outPositionRaw = modelView * vec4(position, 1);
  outUV = uv;
  gl_Position = cameraBuffer.projectionMatrix * outPositionRaw;
  vec3 outTangent = normalize(normalMatrix * T);
  vec3 outBitangent = normalize(normalMatrix * B);
  vec3 outNormal = normalize(normalMatrix * N);
  outTbn = mat3(outTangent, outBitangent, outNormal);
}
