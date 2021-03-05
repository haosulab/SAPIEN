#version 450 

const float PI = 3.1415926535897932384626433832795;

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
  DirectionalLight directionalLights[NUM_DIRECTIONAL_LIGHTS];
  PointLight pointLights[NUM_POINT_LIGHTS];
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
layout(set = 2, binding = 4) uniform sampler2D samplerDepth;
layout(set = 2, binding = 5) uniform sampler2D samplerObjectCoord;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outLighting2;

vec4 world2camera(vec4 pos) {
  return cameraBuffer.viewMatrix * pos;
}

vec3 ggx(vec3 L, vec3 V, vec3 N, vec3 baseColor, float specular, float roughness, float metallic) {
  vec3 albedo = baseColor * (1 - metallic);
  vec3 fresnel = specular * (1 - metallic) + baseColor * metallic;

  vec3 H = normalize((L+V) / 2.f);
  N = faceforward(N, -V, N);
  float NoV = clamp(dot(N, V), 1e-6f, 1.f);
  float NoL = clamp(dot(N, L), 1e-6f, 1.f);
  float NoH = clamp(dot(N, H), 1e-6f, 1.f);
  float VoH = clamp(dot(V, H), 1e-6f, 1.f);
  float alpha = roughness * roughness;
  float alpha2 = alpha * alpha;
  float k = (alpha + 2 * roughness + 1.0) / 8.0;
  float FMi = ((-5.55473) * VoH - 5.98316) * VoH;
  vec3 frac0 = fresnel + (1 - fresnel) * pow(2.0, FMi);
  vec3 frac = frac0 * alpha2;
  float nom0 = NoH * NoH * (alpha2 - 1) + 1;
  float nom1 = NoV * (1 - k) + k;
  float nom2 = NoL * (1 - k) + k;
  float nom = clamp(4 * PI * nom0 * nom0 * nom1 * nom2, 1e-6, 4 * PI);
  vec3 spec = frac / nom;

  return max((albedo / PI + spec) * NoL, 0.f);
}

void main() {
  vec3 albedo = texture(samplerAlbedo, inUV).xyz;
  vec3 srm = texture(samplerSpecular, inUV).xyz;
  float F0 = srm.x;
  float roughness = srm.y;
  float metallic = srm.z;

  vec3 normal = texture(samplerNormal, inUV).xyz;
  vec4 csPosition = texture(samplerPosition, inUV);
  vec3 camDir = -normalize(csPosition.xyz);

  vec3 color = albedo * sceneBuffer.ambientLight.rgb;
  
  // directional lights
  for (int i = 0; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    vec3 lightDir = -normalize(
        (cameraBuffer.viewMatrix * vec4(sceneBuffer.directionalLights[i].direction.xyz, 0)).xyz);
    vec3 emission = sceneBuffer.directionalLights[i].emission.rgb;
    color += emission * ggx(lightDir, camDir, normal, albedo, F0, roughness, metallic);
  }

  // point lights
  for (int i = 0; i < NUM_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(sceneBuffer.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    float d = max(length(l), 1e-6);
    vec3 lightDir = l / d;

    vec3 emission = sceneBuffer.pointLights[i].emission.rgb;
    color += emission * ggx(lightDir, camDir, normal, albedo, F0, roughness, metallic) / (d*d);
  }

  outLighting2 = vec4(color, 1);
}
