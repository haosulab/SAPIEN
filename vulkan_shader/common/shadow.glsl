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
vec3 project(mat4 proj, vec3 point) {
  vec4 v = proj * vec4(point, 1);
  return v.xyz / v.w;
}

const int PCF_SampleCount = 25;
vec2 PCF_Samples[PCF_SampleCount] = {
  {-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {2, -2},
  {-2, -1}, {-1, -1}, {0, -1}, {1, -1}, {2, -1},
  {-2, 0}, {-1, 0}, {0, 0}, {1, 0}, {2, 0},
  {-2, 1}, {-1, 1}, {0, 1}, {1, 1}, {2, 1},
  {-2, 2}, {-1, 2}, {0, 2}, {1, 2}, {2, 2}
};

float ShadowMapPCF(
    sampler2D shadowTex, vec3 projCoord, float resolution, float searchUV, float filterSize)
{
	float shadow = 0.0f;
	vec2 grad = fract(projCoord.xy * resolution + 0.5f);

	for (int i = 0; i < PCF_SampleCount; i++)
	{
    vec4 tmp = textureGather(shadowTex, projCoord.xy + filterSize * PCF_Samples[i] * searchUV);
    tmp.x = tmp.x < projCoord.z ? 0.0f : 1.0f;
    tmp.y = tmp.y < projCoord.z ? 0.0f : 1.0f;
    tmp.z = tmp.z < projCoord.z ? 0.0f : 1.0f;
    tmp.w = tmp.w < projCoord.z ? 0.0f : 1.0f;
    shadow += mix(mix(tmp.w, tmp.z, grad.x), mix(tmp.x, tmp.y, grad.x), grad.y);
  }
	return shadow / PCF_SampleCount;
}