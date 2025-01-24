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
// usage
// #define SET_NUM 1
// #include "scene_set.glsl"
// #undef SET_NUM

#include "../common/lights.glsl"
#include "../common/shadow.glsl"

layout(set = SET_NUM, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  DirectionalLight directionalLights[3];
  SpotLight spotLights[10];
  PointLight pointLights[10];
  SpotLight texturedLights[1];
} sceneBuffer;

struct LightBuffer {
  mat4 viewMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrix;
  mat4 projectionMatrixInverse;
  int width;
  int height;
};

layout(set = SET_NUM, binding = 1) uniform ShadowBuffer {
  LightBuffer directionalLightBuffers[1];
  LightBuffer spotLightBuffers[10];
  LightBuffer pointLightBuffers[18];
  LightBuffer texturedLightBuffers[1];
} shadowBuffer;

layout(set = SET_NUM, binding = 2) uniform samplerCube samplerPointLightDepths[3];
layout(set = SET_NUM, binding = 3) uniform sampler2D samplerDirectionalLightDepths[1];
layout(set = SET_NUM, binding = 4) uniform sampler2D samplerTexturedLightDepths[1];
layout(set = SET_NUM, binding = 5) uniform sampler2D samplerSpotLightDepths[10];
layout(set = SET_NUM, binding = 6) uniform sampler2D samplerTexturedLightTextures[1];

layout(set = SET_NUM, binding = 7) uniform samplerCube samplerEnvironment;
layout(set = SET_NUM, binding = 8) uniform sampler2D samplerBRDFLUT;