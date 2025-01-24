#version 460
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
#extension GL_EXT_ray_tracing : require

#include "ray.glsl"
#include "push_constant.glsl"

layout(location=0) rayPayloadInEXT Ray ray;

layout(set = 1, binding = 10) uniform samplerCube samplerEnvironment;  // TODO: check this

void main() {
  ray.radiance = vec3(0.0);
  ray.albedo = ray.radiance;
  ray.normal = vec3(0.0);
  ray.segmentation = uvec4(0);
  ray.attenuation = vec3(0.0);
  ray.depth = maxDepth + 1;
  ray.origin = vec3(0.0);
  ray.alpha = 0.0;
}