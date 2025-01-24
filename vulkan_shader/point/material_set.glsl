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

layout(set = SET_NUM, binding = 0) uniform MaterialBuffer {
  vec4 emission;
  vec4 baseColor;
  float fresnel;
  float roughness;
  float metallic;
  float transmission;
  float ior;
  float transmissionRoughness;
  int textureMask;
  int padding1;
  vec4 textureTransforms[6];
} materialBuffer;

layout(set = SET_NUM, binding = 1) uniform sampler2D colorTexture;
layout(set = SET_NUM, binding = 2) uniform sampler2D roughnessTexture;
layout(set = SET_NUM, binding = 3) uniform sampler2D normalTexture;
layout(set = SET_NUM, binding = 4) uniform sampler2D metallicTexture;
layout(set = SET_NUM, binding = 5) uniform sampler2D emissionTexture;
layout(set = SET_NUM, binding = 6) uniform sampler2D transmissionTexture;