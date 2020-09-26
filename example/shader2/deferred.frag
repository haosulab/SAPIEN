#version 450 

layout(set = 1, binding = 0) uniform CameraUBO {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
} cameraUBO;

layout (constant_id = 0) const uint NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const uint NUM_POINT_LIGHTS = 10;
struct PointLight {
  vec4 position;
  vec4 emission;
};
struct DirectionalLight {
  vec4 direction;
  vec4 emission;
};
layout(set = 0, binding = 0) uniform SceneUBO {
  vec4 ambientLight;
  DirectionalLight directionalLights[NUM_DIRECTIONAL_LIGHTS];
  PointLight pointLights[NUM_POINT_LIGHTS];
} sceneUBO;

layout(set = 2, binding = 0) uniform sampler2D albedoSampler;
layout(set = 2, binding = 1) uniform sampler2D positionSampler;
layout(set = 2, binding = 2) uniform sampler2D specularSampler;
layout(set = 2, binding = 3) uniform sampler2D normalSampler;
layout(set = 2, binding = 4) uniform sampler2D depthSampler;
layout(set = 2, binding = 5) uniform sampler2D customSampler;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

vec4 world2camera(vec4 pos) {
  return cameraUBO.viewMatrix * pos;
}

vec3 getBackgroundColor(vec3 texcoord) {
  return vec3(1,1,1);
  // float r = sqrt(texcoord.x * texcoord.x + texcoord.z * texcoord.z);
  // float angle = atan(texcoord.y / r) * 57.3;

  // vec3 horizonColor = vec3(0.9, 0.9, 0.9);
  // vec3 zenithColor = vec3(0.522, 0.757, 0.914);
  // vec3 groundColor = vec3(0.5, 0.410, 0.271);
  
  // return mix(mix(zenithColor, horizonColor, smoothstep(15.f, 5.f, angle)),
  //            groundColor,
  //            smoothstep(-5.f, -15.f, angle));
}

float diffuse(vec3 l, vec3 v, vec3 n) {
  return max(dot(l, n), 0.f) / 3.141592653589793f;
}

float SmithG1(vec3 v, vec3 normal, float a2) {
  float dotNV = dot(v, normal);
  return 2 * dotNV / (dotNV + sqrt(a2 + (1-a2) * dotNV * dotNV));
}

float SmithGGXMasking(vec3 wi, vec3 wo, vec3 normal, float a2) {
  return SmithG1(wi, normal, a2) * SmithG1(wo, normal, a2);
}

float ggx(vec3 wi, vec3 wo, vec3 normal, float roughness, float ks) {
  float a2 = roughness * roughness;
  float F0 = ks;
  if (dot(wi, normal) > 0 && dot(wo, normal) > 0) {
    vec3 wm = normalize(wi + wo);
    float dotMN = dot(wm, normal);
    // float F = fresnel_schlick(dot(wi, wm), 5.f, F0, 1);
    float F = F0;
    float G = SmithGGXMasking(wi, wo, normal, a2);
    float D2 = dotMN * dotMN * (a2 - 1) + 1; D2 = D2 * D2;
    float D = a2 / (3.141592653589793f * D2);
    return F * G * D;
  } else {
    return 0.f;
  }
}

void main() {
  vec3 albedo = texture(albedoSampler, inUV).xyz;
  vec3 srm = texture(specularSampler, inUV).xyz;
  float F0 = srm.x;
  float roughness = srm.y;
  float metallic = srm.z;

  vec3 normal = texture(normalSampler, inUV).xyz;
  vec4 csPosition = texture(positionSampler, inUV);
  // vec4 csPosition = getCameraSpacePosition(inUV);
  vec3 camDir = -normalize(csPosition.xyz);

  vec3 color = vec3(0.f);
  for (int i = 0; i < NUM_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(sceneUBO.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    float d = max(length(l), 0.0001);

    if (length(l) == 0) {
      continue;
    }

    vec3 lightDir = normalize(l);

    // diffuse
    color += (1 - metallic) * albedo * sceneUBO.pointLights[i].emission.rgb *
             diffuse(lightDir, camDir, normal) / d / d;

    // metallic
    color += metallic * albedo * sceneUBO.pointLights[i].emission.rgb *
             ggx(lightDir, camDir, normal, roughness, 1.f) / d / d;

    // specular
    color += sceneUBO.pointLights[i].emission.rgb * ggx(lightDir, camDir, normal, roughness, F0) / d / d;
  }

  for (int i = 0; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    if (length(sceneUBO.directionalLights[i].direction.xyz) == 0) {
      continue;
    }

    vec3 lightDir = -normalize((cameraUBO.viewMatrix *
                                vec4(sceneUBO.directionalLights[i].direction.xyz, 0)).xyz);

    // diffuse
    color += (1 - metallic) * albedo * sceneUBO.directionalLights[i].emission.rgb *
             diffuse(lightDir, camDir, normal);

    color += (1 - metallic) * albedo * sceneUBO.directionalLights[i].emission.rgb *
             diffuse(lightDir, camDir, normal);

    // metallic
    color += metallic * albedo * sceneUBO.directionalLights[i].emission.rgb *
             ggx(lightDir, camDir, normal, roughness, 1.f);

    // specular
    color += sceneUBO.directionalLights[i].emission.rgb * ggx(lightDir, camDir, normal, roughness, F0);
  }

  color += sceneUBO.ambientLight.rgb * albedo;

  float depth = texture(depthSampler, inUV).x;
  if (depth == 1) {
    outColor = vec4(getBackgroundColor((cameraUBO.viewMatrixInverse * csPosition).xyz), 1.f);
  } else {
    outColor = vec4(color, 1);
  }

  // outColor = vec4(abs(normal), 1);
  // outColor = vec4(depth, depth, depth, 1);
  // outColor = vec4(csPosition.xy, 0, 1);
  // outColor = vec4(wsPosition.xyz, 1);
  // outColor = vec4(wsNormal, 1);
  // outColor = vec4(ffnormal, 1);
  // outColor = vec4(-csPosition.xyz / 6, 1);
}
