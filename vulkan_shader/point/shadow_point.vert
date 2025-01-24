#version 450
//
// Copyright 2025 Hillbot Inc.
// Copyright 2020-2024 UCSD SU Lab
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at:
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

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

layout(location = 0) in vec3 position;
layout(location = 1) in float scale;
layout(location = 2) in vec4 color;

layout(location = 0) out vec4 outPosition;
layout(location = 1) out flat vec4 outNdcRadius;

void main() {
  mat4 modelView = lightBuffer.viewMatrix * objectTransformBuffer.modelMatrix;
  outPosition = modelView * vec4(position, 1);

  float radius = scale;

  gl_PointSize = lightBuffer.projectionMatrix[0][0] * lightBuffer.width * radius;

  gl_Position = lightBuffer.projectionMatrix * outPosition;
  outNdcRadius = vec4(gl_Position.xyz / gl_Position.w, radius);
}