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
#define M_PI 3.141592
#define M_1_PI 0.318309886

uint tea(uint val0, uint val1) {
  uint v0 = val0;
  uint v1 = val1;
  uint sum = 0;
  const uint delta = 0x9E3779B9;
  const uint k0 = 0xa341316c;
  const uint k1 = 0xc8013ea4;
  const uint k2 = 0xad90777d;
  const uint k3 = 0x7e95761e;

  for (uint i = 0; i < 16; ++i) {
    sum += delta;
    v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
    v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
  }
  return v0;
}

// log(N) time should be okay
float halton(int i, int b) {
  float f = 1;
  float r = 0.f;
  while (i > 0) {
    f /= b;
    r += f * (i % b);
    i /= b;
  }
  return r;
}

vec2 halton2d(int i, int b1, int b2) {
  return vec2(halton(i, b1), halton(i, b2));
}

uint lcg(inout uint prev)
{
  const uint LCG_A = 1664525u;
  const uint LCG_C = 1013904223u;
  prev = (LCG_A * prev + LCG_C);
  return prev & 0x00FFFFFF;
}

float rnd(inout uint prev) {
  return (float(lcg(prev)) / float(0x01000000));
}

vec3 cosineSampleHemisphere(inout uint seed) {
  float theta = rnd(seed) * M_PI * 2.0;
  float z2 = rnd(seed);
  float xy = sqrt(1.0 - z2);
  return vec3(cos(theta) * xy, sin(theta) * xy, sqrt(z2));
}