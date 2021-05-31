#version 450

layout(set = 0, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
} cameraBuffer;

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

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_SPOT_LIGHTS = 10;

#include "../common/lights.glsl"

layout(set = 3, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  DirectionalLight directionalLights[3];
  PointLight pointLights[10];
  SpotLight spotLights[10];
} sceneBuffer;


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

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormal;
layout(location = 2) out uvec4 outSegmentation;

void main() {
  outSegmentation = inSegmentation;

  vec4 albedo;
  vec4 frm;

  if ((materialBuffer.textureMask & 1) != 0) {
    albedo = texture(colorTexture, inUV);
  } else {
    albedo = materialBuffer.baseColor;
  }

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
    outNormal = vec4(normalize(inTbn * (texture(normalTexture, inUV).xyz * 2 - 1)), 1);
  } else {
    outNormal = vec4(normalize(inTbn * vec3(0, 0, 1)), 1);
  }

  float specular = frm.x;
  float roughness = frm.y;
  float metallic = frm.z;

  vec3 normal = outNormal.xyz;
  vec4 csPosition = inPosition;
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
    color += computeDirectionalLight(
        mat3(cameraBuffer.viewMatrix) * sceneBuffer.directionalLights[i].direction.xyz,
        sceneBuffer.directionalLights[i].emission.rgb,
        normal, camDir, diffuseAlbedo, roughness, fresnel);
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

  color += sceneBuffer.ambientLight.rgb * albedo.rgb;

  outColor = vec4(color, albedo.a);
  outColor = pow(outColor, vec4(1/2.2));
}
