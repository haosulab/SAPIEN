#version 450 

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_DIRECTIONAL_LIGHT_SHADOWS = 1;
layout (constant_id = 3) const int NUM_POINT_LIGHT_SHADOWS = 3;
layout (constant_id = 4) const int NUM_CUSTOM_LIGHT_SHADOWS = 1;

struct PointLight {
  vec4 position;
  vec4 emission;
};
struct DirectionalLight {
  vec4 direction;
  vec4 emission;
};

layout(set = 0, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  DirectionalLight directionalLights[3];
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
  LightBuffer pointLightBuffers[60];
  LightBuffer customLightBuffers[1];
} shadowBuffer;

layout(set = 0, binding = 2) uniform samplerCubeArray samplerPointLightDepths;
layout(set = 0, binding = 3) uniform sampler2DArray samplerDirectionalLightDepths;
layout(set = 0, binding = 4) uniform sampler2DArray samplerCustomLightDepths;
// layout(set = 0, binding = 5) uniform sampler2D samplerLightMap;

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

float diffuse(float NoL) {
  return NoL / 3.141592653589793f;
}

vec3 ggx(float NoL, float NoV, float NoH, float VoH, float roughness, vec3 fresnel) {
  float alpha = roughness * roughness;
  float alpha2 = alpha * alpha;

  float k = (alpha + 2 * roughness + 1.0) / 8.0;

  float FMi = ((-5.55473) * VoH - 6.98316) * VoH;
  vec3 frac = (fresnel + (1 - fresnel) * pow(2.0, FMi)) * alpha2;
  float nom0 = NoH * NoH * (alpha2 - 1) + 1;
  float nom1 = NoV * (1 - k) + k;
  float nom2 = NoL * (1 - k) + k;
  float nom = clamp((4 * 3.141592653589793f * nom0 * nom0 * nom1 * nom2), 1e-6, 4 * 3.141592653589793f);
  vec3 spec = frac / nom;

  return spec * NoL;
}

vec3 computeDirectionalLight(int index, vec3 normal, vec3 camDir, vec3 diffuseAlbedo, float roughness, vec3 fresnel) {
  vec3 lightDir = -normalize((cameraBuffer.viewMatrix *
                              vec4(sceneBuffer.directionalLights[index].direction.xyz, 0)).xyz);

  vec3 H = lightDir + camDir;
  float H2 = dot(H, H);
  H = H2 < 1e-6 ? vec3(0) : normalize(H);
  float NoH = clamp(dot(normal, H), 1e-6, 1);
  float VoH = clamp(dot(camDir, H), 1e-6, 1);
  float NoL = clamp(dot(normal, lightDir), 0, 1);
  float NoV = clamp(dot(normal, camDir), 1e-6, 1);

  vec3 color = diffuseAlbedo * sceneBuffer.directionalLights[index].emission.rgb * diffuse(NoL);
  color += sceneBuffer.directionalLights[index].emission.rgb * ggx(NoL, NoV, NoH, VoH, roughness, fresnel);
  return color;
}

vec3 computePointLight(vec3 emission, vec3 l, vec3 normal, vec3 camDir, vec3 diffuseAlbedo, float roughness, vec3 fresnel) {
  float d = max(length(l), 0.0001);

  if (length(l) == 0) {
    return vec3(0.f);
  }

  vec3 lightDir = normalize(l);

  vec3 H = lightDir + camDir;
  float H2 = dot(H, H);
  H = H2 < 1e-6 ? vec3(0) : normalize(H);
  float NoH = clamp(dot(normal, H), 1e-6, 1);
  float VoH = clamp(dot(camDir, H), 1e-6, 1);
  float NoL = clamp(dot(normal, lightDir), 0, 1);
  float NoV = clamp(dot(normal, camDir), 1e-6, 1);

  vec3 color = diffuseAlbedo * emission * diffuse(NoL) / d / d;
  color += emission * ggx(NoL, NoV, NoH, VoH, roughness, fresnel) / d / d;
  return color;
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

  for (int i = 0; i < NUM_DIRECTIONAL_LIGHT_SHADOWS; ++i) {
    mat4 shadowView = shadowBuffer.directionalLightBuffers[i].viewMatrix;
    mat4 shadowProj = shadowBuffer.directionalLightBuffers[i].projectionMatrix;

    vec4 ssPosition = shadowView * cameraBuffer.viewMatrixInverse * vec4((csPosition.xyz + normal * eps), 1);
    vec4 shadowMapCoord = shadowProj * ssPosition;
    shadowMapCoord /= shadowMapCoord.w;
    shadowMapCoord.xy = shadowMapCoord.xy * 0.5 + 0.5;

    float visibility = step(shadowMapCoord.z - texture(samplerDirectionalLightDepths, vec3(shadowMapCoord.xy, i)).x, 0);
    color += visibility * computeDirectionalLight(i, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  for (int i = NUM_DIRECTIONAL_LIGHT_SHADOWS; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    color += computeDirectionalLight(i, normal, camDir, diffuseAlbedo, roughness, fresnel);
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
