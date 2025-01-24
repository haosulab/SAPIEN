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
struct DirectionalLight {
  vec3 direction;
  float softness;
  vec3 rgb;
  float padding;
};

struct PointLight {
  vec3 position;
  float radius;
  vec3 rgb;
  float padding;
};

struct SpotLight {
  mat4 viewMat;
  mat4 projMat;
  vec3 rgb;
  int padding0;
  vec3 position;
  int padding1;
  float fovInner;
  float fovOuter;
  int textureId;
  int padding2;
};

struct ParallelogramLight {
  vec3 rgb;
  float padding0;
  vec3 position;
  float padding1;
  vec3 edge0;
  float padding2;
  vec3 edge1;
  float padding3;
};