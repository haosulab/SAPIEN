#version 450 

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;

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
  DirectionalLight directionalLights[NUM_DIRECTIONAL_LIGHTS > 0 ? NUM_DIRECTIONAL_LIGHTS : 1];
  PointLight pointLights[NUM_POINT_LIGHTS > 0 ? NUM_POINT_LIGHTS : 1];
} sceneBuffer;

layout(set = 1, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
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
  return vec3(1,1,1);
}

float diffuse(float NoL) {
  return max(NoL, 0.f) / 3.141592653589793f;
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

void main() {
  vec3 albedo = texture(samplerAlbedo, inUV).xyz;
  vec3 frm = texture(samplerSpecular, inUV).xyz;
  float specular = frm.x;
  float roughness = frm.y;
  float metallic = frm.z;

  vec3 normal = texture(samplerNormal, inUV).xyz * 2 - 1;
  float depth = texture(samplerGbufferDepth, inUV).x;
  vec4 csPosition = cameraBuffer.projectionMatrixInverse * (vec4(inUV * 2 - 1, depth, 1));
  csPosition /= csPosition.w;

  vec3 camDir = -normalize(csPosition.xyz);

  vec3 diffuseAlbedo = albedo * (1 - metallic);
  vec3 fresnel = specular * (1 - metallic) + albedo * metallic;

  vec3 color = vec3(0.f);
  for (int i = 0; i < NUM_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(sceneBuffer.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    float d = max(length(l), 0.0001);

    if (length(l) == 0) {
      continue;
    }

    vec3 lightDir = normalize(l);

    vec3 H = lightDir + camDir;
    float H2 = dot(H, H);
    H = H2 < 1e-6 ? vec3(0) : normalize(H);
    float NoH = dot(normal, H);
    float VoH = dot(camDir, H);
    float NoL = dot(normal, lightDir);
    float NoV = dot(normal, camDir);

    color += diffuseAlbedo * sceneBuffer.pointLights[i].emission.rgb * diffuse(NoL) / d / d;
    color += sceneBuffer.pointLights[i].emission.rgb * ggx(NoL, NoV, NoH, VoH, roughness, fresnel) / d / d;
  }

  for (int i = 0; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    if (length(sceneBuffer.directionalLights[i].direction.xyz) == 0) {
      continue;
    }

    vec3 lightDir = -normalize((cameraBuffer.viewMatrix *
                                vec4(sceneBuffer.directionalLights[i].direction.xyz, 0)).xyz);

    vec3 H = lightDir + camDir;
    float H2 = dot(H, H);
    H = H2 < 1e-6 ? vec3(0) : normalize(H);
    float NoH = dot(normal, H);
    float VoH = dot(camDir, H);
    float NoL = dot(normal, lightDir);
    float NoV = dot(normal, camDir);

    color += diffuseAlbedo * sceneBuffer.directionalLights[i].emission.rgb * diffuse(NoL);
    color += sceneBuffer.directionalLights[i].emission.rgb * ggx(NoL, NoV, NoH, VoH, roughness, fresnel);
  }

  color += sceneBuffer.ambientLight.rgb * albedo;

  if (depth == 1) {
    outLighting = vec4(getBackgroundColor((cameraBuffer.viewMatrixInverse * csPosition).xyz), 1.f);
  } else {
    outLighting = vec4(color, 1);
  }
}
