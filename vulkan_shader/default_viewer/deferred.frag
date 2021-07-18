#version 450

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_DIRECTIONAL_LIGHT_SHADOWS = 1;
layout (constant_id = 3) const int NUM_POINT_LIGHT_SHADOWS = 3;
layout (constant_id = 4) const int NUM_CUSTOM_LIGHT_SHADOWS = 1;
layout (constant_id = 6) const int NUM_SPOT_LIGHTS = 10;
layout (constant_id = 5) const int NUM_SPOT_LIGHT_SHADOWS = 10;

#include "../common/lights.glsl"

layout(set = 0, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  DirectionalLight directionalLights[3];
  SpotLight spotLights[10];
  PointLight pointLights[10];
} sceneBuffer;

struct LightBuffer {
  mat4 viewMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrix;
  mat4 projectionMatrixInverse;
};

layout(set = 0, binding = 1) uniform ShadowBuffer {
  LightBuffer directionalLightBuffers[3];
  LightBuffer spotLightBuffers[10];
  LightBuffer pointLightBuffers[60];
  LightBuffer customLightBuffers[1];
} shadowBuffer;

layout(set = 0, binding = 2) uniform samplerCubeArray samplerPointLightDepths;
layout(set = 0, binding = 3) uniform sampler2DArray samplerDirectionalLightDepths;
layout(set = 0, binding = 4) uniform sampler2DArray samplerCustomLightDepths;
layout(set = 0, binding = 5) uniform sampler2DArray samplerSpotLightDepths;

layout(set = 1, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  mat4 prevViewMatrix;
  mat4 prevViewMatrixInverse;
  float width;
  float height;
} cameraBuffer;

layout(set = 2, binding = 0) uniform sampler2D samplerAlbedo;
layout(set = 2, binding = 1) uniform sampler2D samplerPosition;
layout(set = 2, binding = 2) uniform sampler2D samplerSpecular;
layout(set = 2, binding = 3) uniform sampler2D samplerNormal;
layout(set = 2, binding = 4) uniform sampler2D samplerGbufferDepth;
layout(set = 2, binding = 5) uniform sampler2D samplerCustom;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outLighting;

vec4 world2camera(vec4 pos) {
  return cameraBuffer.viewMatrix * pos;
}

vec3 getBackgroundColor(vec3 texcoord) {
  return vec3(0.89411765, 0.83137255, 0.72156863) - 0.2;
}


const float eps = 1e-2;
void main() {
  vec3 albedo = texture(samplerAlbedo, inUV).xyz;
  vec3 frm = texture(samplerSpecular, inUV).xyz;
  float specular = frm.x;
  float roughness = frm.y;
  float metallic = frm.z;

  vec3 normal = normalize(texture(samplerNormal, inUV).xyz);
  float depth = texture(samplerGbufferDepth, inUV).x;
  vec4 csPosition = cameraBuffer.projectionMatrixInverse * (vec4(inUV * 2 - 1, depth, 1));
  csPosition /= csPosition.w;

  vec3 camDir = -normalize(csPosition.xyz);

  vec3 diffuseAlbedo = albedo * (1 - metallic);
  vec3 fresnel = specular * (1 - metallic) + albedo * metallic;

  vec3 color = vec3(0.f);

  // point light
  for (int i = 0; i < NUM_POINT_LIGHT_SHADOWS; ++i) {
    vec3 pos = world2camera(vec4(sceneBuffer.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;

    vec3 wsl = vec3(cameraBuffer.viewMatrixInverse * vec4(l - normal * eps, 0));

    mat4 shadowProj = shadowBuffer.pointLightBuffers[6 * i].projectionMatrix;
    vec3 v = abs(wsl);
    vec4 p = shadowProj * vec4(0, 0, -max(max(v.x, v.y), v.z), 1);
    float pixelDepth = p.z / p.w;
    float shadowDepth = texture(samplerPointLightDepths, vec4(-wsl, i)).x;

    float visibility = step(pixelDepth - shadowDepth, 0);
    color += visibility * computePointLight(
        sceneBuffer.pointLights[i].emission.rgb,
        l, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  for (int i = NUM_POINT_LIGHT_SHADOWS; i < NUM_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(sceneBuffer.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    color += computePointLight(
        sceneBuffer.pointLights[i].emission.rgb,
        l, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  // directional light
  for (int i = 0; i < NUM_DIRECTIONAL_LIGHT_SHADOWS; ++i) {
    mat4 shadowView = shadowBuffer.directionalLightBuffers[i].viewMatrix;
    mat4 shadowProj = shadowBuffer.directionalLightBuffers[i].projectionMatrix;

    vec4 ssPosition = shadowView * cameraBuffer.viewMatrixInverse * vec4((csPosition.xyz + normal * eps), 1);
    vec4 shadowMapCoord = shadowProj * ssPosition;
    shadowMapCoord /= shadowMapCoord.w;
    shadowMapCoord.xy = shadowMapCoord.xy * 0.5 + 0.5;

    float visibility = step(shadowMapCoord.z - texture(samplerDirectionalLightDepths, vec3(shadowMapCoord.xy, i)).x, 0);
    color += visibility * computeDirectionalLight(
        mat3(cameraBuffer.viewMatrix) * sceneBuffer.directionalLights[i].direction.xyz,
        sceneBuffer.directionalLights[i].emission.rgb,
        normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  for (int i = NUM_DIRECTIONAL_LIGHT_SHADOWS; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    color += computeDirectionalLight(
        mat3(cameraBuffer.viewMatrix) * sceneBuffer.directionalLights[i].direction.xyz,
        sceneBuffer.directionalLights[i].emission.rgb,
        normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  // spot light
  for (int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; ++i) {
    mat4 shadowView = shadowBuffer.spotLightBuffers[i].viewMatrix;
    mat4 shadowProj = shadowBuffer.spotLightBuffers[i].projectionMatrix;

    vec4 ssPosition = shadowView * cameraBuffer.viewMatrixInverse * vec4((csPosition.xyz + normal * eps), 1);
    vec4 shadowMapCoord = shadowProj * ssPosition;
    shadowMapCoord /= shadowMapCoord.w;
    shadowMapCoord.xy = shadowMapCoord.xy * 0.5 + 0.5;

    float visibility = step(shadowMapCoord.z - texture(samplerSpotLightDepths, vec3(shadowMapCoord.xy, i)).x, 0);

    vec3 pos = world2camera(vec4(sceneBuffer.spotLights[i].position.xyz, 1.f)).xyz;
    vec3 centerDir = mat3(cameraBuffer.viewMatrix) * sceneBuffer.spotLights[i].direction.xyz;
    vec3 l = pos - csPosition.xyz;
    color += visibility * computeSpotLight(
        sceneBuffer.spotLights[i].emission.a,
        sceneBuffer.spotLights[i].direction.a,
        centerDir,
        sceneBuffer.spotLights[i].emission.rgb,
        l, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  for (int i = NUM_SPOT_LIGHT_SHADOWS; i < NUM_SPOT_LIGHTS; ++i) {
    vec3 pos = world2camera(vec4(sceneBuffer.spotLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    vec3 centerDir = mat3(cameraBuffer.viewMatrix) * sceneBuffer.spotLights[i].direction.xyz;
    color += computeSpotLight(
        sceneBuffer.spotLights[i].emission.a,
        sceneBuffer.spotLights[i].direction.a,
        centerDir,
        sceneBuffer.spotLights[i].emission.rgb,
        l, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }


  for (int i = 0; i < NUM_CUSTOM_LIGHT_SHADOWS; ++i) {
    mat4 shadowView = shadowBuffer.customLightBuffers[i].viewMatrix;
    mat4 shadowProj = shadowBuffer.customLightBuffers[i].projectionMatrix;

    vec4 ssPosition = shadowView * cameraBuffer.viewMatrixInverse * vec4((csPosition.xyz + normal * eps), 1);
    vec4 shadowMapCoord = shadowProj * ssPosition;
    shadowMapCoord /= shadowMapCoord.w;
    shadowMapCoord.xy = shadowMapCoord.xy * 0.5 + 0.5;

    float visibility = step(shadowMapCoord.z - texture(samplerCustomLightDepths, vec3(shadowMapCoord.xy, i)).x, 0);
    // visibility *= pow(texture(samplerLightMap, shadowMapCoord.xy).x, 2.2);  // un-gamma
    visibility *= step(shadowMapCoord.x, 1) * step(0, shadowMapCoord.x) * step(shadowMapCoord.y, 1) * step(0, shadowMapCoord.y);

    vec4 lightWPos = shadowBuffer.customLightBuffers[i].viewMatrixInverse * vec4(0,0,0,1);
    vec4 lightCPos = cameraBuffer.viewMatrix * lightWPos;
    vec3 l = (lightCPos.xyz - csPosition.xyz);

    color += visibility * computePointLight(vec3(1.f), l, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  color += sceneBuffer.ambientLight.rgb * diffuseAlbedo;

  if (depth == 1) {
    outLighting = vec4(getBackgroundColor((cameraBuffer.viewMatrixInverse * csPosition).xyz), 1.f);
  } else {
    outLighting = vec4(color, 1);
  }
}
