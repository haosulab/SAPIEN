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
layout(location = 1) in float scale;
layout(location = 2) in vec4 color;

layout(location = 0) out vec4 outPositionRaw;
layout(location = 1) out vec4 outColor;

void main() {
  mat4 modelView = cameraBuffer.viewMatrix * objectTransformBuffer.modelMatrix;
  outPositionRaw = modelView * vec4(position, 1);
  gl_Position = cameraBuffer.projectionMatrix * outPositionRaw;
  outColor = color;
}
