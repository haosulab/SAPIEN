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
#include "./point.glsl"

struct Vertex {
  float x, y, z;
  float nx, ny, nz;
  float u, v;

  float tx, ty, tz;
  float bx, by, bz;
};

struct Object {
  uvec4 segmentation;
  float transparency;
  int shadeFlat;
  int padding0;
  int padding1;
};

float maxVec3(vec3 v) {
  return max(v.x, max(v.y, v.z));
}

layout(set = SET_NUM, binding = 0) uniform accelerationStructureEXT tlas;

layout(set = SET_NUM, binding = 1) readonly buffer GeometryInstances
{
  GeometryInstance i[];
} geometryInstances;

layout(set = SET_NUM, binding = 2) readonly buffer Materials
{
  Material m;
} materials[];

layout(set = SET_NUM, binding = 3) readonly buffer TextureIndices
{
  TextureIndex t[];
} textureIndices;

layout(set = SET_NUM, binding = 4) uniform sampler2D textures[];

layout(set = SET_NUM, binding = 5) readonly buffer PointLights
{
  PointLight l[];
} pointLights;

layout(set = SET_NUM, binding = 6) readonly buffer DirectionalLights
{
  DirectionalLight l[];
} directionalLights;

layout(set = SET_NUM, binding = 7) readonly buffer SpotLights
{
  SpotLight l[];
} spotLights;


layout(std430, set = SET_NUM, binding = 8) readonly buffer Vertices {
  Vertex v[];
} vertices[];

layout(set = SET_NUM, binding = 9) readonly buffer Indices {
  uint i[];
} indices[];

layout(set = SET_NUM, binding = 10) uniform samplerCube samplerEnvironment;

layout(set = SET_NUM, binding = 11) readonly buffer Objects {
  Object o[];
} objects;

layout(set = SET_NUM, binding = 12) readonly buffer ParallelogramLights
{
  ParallelogramLight l[];
} parallelogramLights;

layout(set = SET_NUM, binding = 13) readonly buffer PointInstances
{
  int pointInstances[];
};

layout(std430, set = SET_NUM, binding = 14) readonly buffer Points {
  Point p[];
} points[];