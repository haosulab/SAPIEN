#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec4 inPosition;
layout(location = 1) in flat vec4 inNdcRadius;

layout(set = 0, binding = 0) uniform LightBuffer {
  mat4 viewMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrix;
  mat4 projectionMatrixInverse;
  int width;
  int height;
} lightBuffer;

#define SET_NUM 1
#include "./object_set.glsl"
#undef SET_NUM

void main() {
  if (gl_PointCoord.s * gl_PointCoord.s + gl_PointCoord.t * gl_PointCoord.t > 1) {
    discard;
  }
}
