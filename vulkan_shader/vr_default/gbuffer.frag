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

#define SET_NUM 0
#include "./camera_set.glsl"
#undef SET_NUM

#define SET_NUM 1
#include "./object_set.glsl"
#undef SET_NUM

#define SET_NUM 2
#include "./material_set.glsl"
#undef SET_NUM


layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec2 inUV;
layout(location = 2) in flat uvec4 inSegmentation;
layout(location = 3) in vec3 objectCoord;
layout(location = 4) in mat3 inTbn;

layout(location = 0) out vec4 outAlbedo;
layout(location = 1) out vec4 outPositionRaw;
layout(location = 2) out vec4 outSpecular;
layout(location = 3) out vec4 outNormal;
layout(location = 4) out uvec4 outSegmentation;
layout(location = 5) out vec4 outCustom;
layout(location = 6) out vec4 outEmission;

void main() {
  outCustom = vec4(objectCoord, 1);
  outSegmentation = inSegmentation;

  outPositionRaw = inPosition;

  vec4 p1 = cameraBuffer.projectionMatrix * inPosition;
  p1 /= p1.w;
  vec2 p1s = p1.xy / p1.z;

  if ((materialBuffer.textureMask & 16) != 0) {
    outEmission = texture(emissionTexture, inUV * materialBuffer.textureTransforms[4].zw + materialBuffer.textureTransforms[4].xy);
  } else {
    outEmission = materialBuffer.emission;
  }

  if ((materialBuffer.textureMask & 1) != 0) {
    outAlbedo = texture(colorTexture, inUV * materialBuffer.textureTransforms[0].zw + materialBuffer.textureTransforms[0].xy);
  } else {
    outAlbedo = materialBuffer.baseColor;
  }

  if (outAlbedo.a == 0) {
    discard;
  }

  outSpecular.r = materialBuffer.fresnel * 0.08;

  if ((materialBuffer.textureMask & 2) != 0) {
    outSpecular.g = texture(roughnessTexture, inUV * materialBuffer.textureTransforms[1].zw + materialBuffer.textureTransforms[1].xy).r;
  } else {
    outSpecular.g = materialBuffer.roughness;
  }

  if ((materialBuffer.textureMask & 8) != 0) {
    outSpecular.b = texture(metallicTexture, inUV * materialBuffer.textureTransforms[3].zw + materialBuffer.textureTransforms[3].xy).r;
  } else {
    outSpecular.b = materialBuffer.metallic;
  }

  if (objectDataBuffer.shadeFlat == 0) {
    if ((materialBuffer.textureMask & 4) != 0) {
      outNormal = vec4(normalize(inTbn * (texture(normalTexture, inUV * materialBuffer.textureTransforms[2].zw + materialBuffer.textureTransforms[2].xy).xyz * 2 - 1)), 0);
    } else {
      outNormal = vec4(normalize(inTbn * vec3(0, 0, 1)), 0);
    }
  } else {
    vec4 fdx = dFdx(inPosition);
    vec4 fdy = dFdy(inPosition);
    vec3 normal = -normalize(cross(fdx.xyz, fdy.xyz));
    outNormal = vec4(normal, 0);
  }
}