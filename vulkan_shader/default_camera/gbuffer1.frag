#version 450

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_DIRECTIONAL_LIGHT_SHADOWS = 1;
layout (constant_id = 3) const int NUM_POINT_LIGHT_SHADOWS = 3;
layout (constant_id = 4) const int NUM_CUSTOM_LIGHT_SHADOWS = 1;
layout (constant_id = 6) const int NUM_SPOT_LIGHTS = 10;
layout (constant_id = 5) const int NUM_SPOT_LIGHT_SHADOWS = 10;

layout(set = 0, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  mat4 prevViewMatrix;
  mat4 prevViewMatrixInverse;
  float width;
  float height;
} cameraBuffer;

layout(set = 1, binding = 0) uniform ObjectBuffer {
  mat4 modelMatrix;
  mat4 prevModelMatrix;
  uvec4 segmentation;
  float transparency;
} objectBuffer;

layout(set = 2, binding = 0) uniform MaterialBuffer {
  vec4 baseColor;
  float fresnel;
  float roughness;
  float metallic;
  float transparency;
  int textureMask;
} materialBuffer;

layout(set = 2, binding = 1) uniform sampler2D colorTexture;
layout(set = 2, binding = 2) uniform sampler2D roughnessTexture;
layout(set = 2, binding = 3) uniform sampler2D normalTexture;
layout(set = 2, binding = 4) uniform sampler2D metallicTexture;


#include "../common/lights.glsl"

layout(set = 3, binding = 0) uniform SceneBuffer {
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

layout(set = 3, binding = 1) uniform ShadowBuffer {
  LightBuffer directionalLightBuffers[3];
  LightBuffer spotLightBuffers[10];
  LightBuffer pointLightBuffers[60];
  LightBuffer customLightBuffers[1];
} shadowBuffer;

layout(set = 3, binding = 2) uniform samplerCubeArray samplerPointLightDepths;
layout(set = 3, binding = 3) uniform sampler2DArray samplerDirectionalLightDepths;
layout(set = 3, binding = 4) uniform sampler2DArray samplerCustomLightDepths;
layout(set = 3, binding = 5) uniform sampler2DArray samplerSpotLightDepths;


vec4 world2camera(vec4 pos) {
  return cameraBuffer.viewMatrix * pos;
}

vec3 getBackgroundColor(vec3 texcoord) {
  return vec3(1,1,1);
}

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec2 inUV;
layout(location = 2) in flat uvec4 inSegmentation;
layout(location = 3) in vec3 objectCoord;
layout(location = 4) in mat3 inTbn;

layout(location = 0) out vec4 outLighting1;
layout(location = 1) out vec4 outNormal1;
layout(location = 2) out uvec4 outSegmentation1;
layout(location = 3) out vec4 outPosition1;

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

void main() {
  outSegmentation1 = inSegmentation;

  vec4 albedo;
  vec4 frm;

  if ((materialBuffer.textureMask & 1) != 0) {
    albedo = texture(colorTexture, inUV);
  } else {
    albedo = materialBuffer.baseColor;
  }

  albedo.a *=  (1.f - objectBuffer.transparency);

  if (albedo.a == 0) {
    discard;
  }

  frm.r = materialBuffer.fresnel;

  if ((materialBuffer.textureMask & 2) != 0) {
    frm.g = texture(roughnessTexture, inUV).r;
  } else {
    frm.g = materialBuffer.roughness;
  }

  if ((materialBuffer.textureMask & 8) != 0) {
    frm.b = texture(metallicTexture, inUV).r;
  } else {
    frm.b = materialBuffer.metallic;
  }

  if ((materialBuffer.textureMask & 4) != 0) {
    outNormal1 = vec4(normalize(inTbn * (texture(normalTexture, inUV).xyz * 2 - 1)), 0);
  } else {
    outNormal1 = vec4(normalize(inTbn * vec3(0, 0, 1)), 0);
  }
  outNormal1 = outNormal1;

  float specular = frm.x;
  float roughness = frm.y;
  float metallic = frm.z;

  vec3 normal = outNormal1.xyz;
  vec4 csPosition = inPosition;
  outPosition1 = inPosition;
  csPosition /= csPosition.w;

  vec3 camDir = -normalize(csPosition.xyz);

  vec3 diffuseAlbedo = albedo.rgb * (1 - metallic);
  vec3 fresnel = specular * (1 - metallic) + albedo.rgb * metallic;

  vec3 color = vec3(0.f);
  for (int i = 0; i < NUM_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(sceneBuffer.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    color += computePointLight(
        sceneBuffer.pointLights[i].emission.rgb,
        l, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  for (int i = 0; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    color += computeDirectionalLight(i, normal, camDir, diffuseAlbedo, roughness, fresnel);
  }

  for (int i = 0; i < NUM_SPOT_LIGHTS; ++i) {
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

  color += sceneBuffer.ambientLight.rgb * diffuseAlbedo;

  outLighting1 = vec4(color, albedo.a);
}
