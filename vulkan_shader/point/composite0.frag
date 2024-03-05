#version 450

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_DIRECTIONAL_LIGHT_SHADOWS = 1;
layout (constant_id = 3) const int NUM_POINT_LIGHT_SHADOWS = 3;
layout (constant_id = 4) const int NUM_TEXTURED_LIGHT_SHADOWS = 1;
layout (constant_id = 5) const int NUM_SPOT_LIGHT_SHADOWS = 10;
layout (constant_id = 6) const int NUM_SPOT_LIGHTS = 10;

layout(set = 0, binding = 0) uniform sampler2D samplerPointColor;
layout(set = 0, binding = 1) uniform sampler2D samplerPointNormal;
layout(set = 0, binding = 2) uniform sampler2D samplerPointPosition;

#define SET_NUM 2
#include "./scene_set.glsl"
#undef SET_NUM

#define SET_NUM 1
#include "./camera_set.glsl"
#undef SET_NUM

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outPoint;

vec3 diffuseIBL(vec3 albedo, vec3 N) {
  N = vec3(-N.y, N.z, -N.x);
  vec3 color = textureLod(samplerEnvironment, N, 5).rgb;
  return color * albedo;
}

vec3 specularIBL(vec3 fresnel, float roughness, vec3 N, vec3 V) {
  float dotNV = max(dot(N, V), 0);
  vec3 R = 2 * dot(N, V) * N - V;
  R = vec3(-R.y, R.z, -R.x);
  vec3 color = textureLod(samplerEnvironment, R, roughness * 5).rgb;
  vec2 envBRDF = texture(samplerBRDFLUT, vec2(roughness, dotNV)).xy;
  return color * (fresnel * envBRDF.x + envBRDF.y);
}

void main() {
  vec3 fresnel = vec3(0.04);
  float roughness = 0.01;

  vec3 color = vec3(0.0);

  vec3 diffuseAlbedo = texture(samplerPointColor, inUV).xyz;
  vec3 normal = texture(samplerPointNormal, inUV).xyz;
  vec3 position = texture(samplerPointPosition, inUV).xyz;

  vec3 camDir = -normalize(position);

  vec3 wnormal = mat3(cameraBuffer.viewMatrixInverse) * normal;
  color += diffuseIBL(diffuseAlbedo, wnormal);
  color += specularIBL(fresnel, roughness,
                       wnormal,
                       mat3(cameraBuffer.viewMatrixInverse) * camDir);

  color += sceneBuffer.ambientLight.rgb * diffuseAlbedo;

  outPoint = vec4(color, 1.0);
}
