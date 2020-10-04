#version 450

//=== geometry resources ===//

layout(set = 3, binding = 0) uniform MaterialUBO {
  vec4 baseColor;  // rgba
  float specular;
  float roughness;
  float metallic;
  float transparency;
  int hasColorTexture;
  int hasSpecularTexture;
  int hasNormalTexture;
  int hasHeightTexture;
} material;

layout(set = 3, binding = 1) uniform sampler2D colorTexture;
layout(set = 3, binding = 2) uniform sampler2D specularTexture;
layout(set = 3, binding = 3) uniform sampler2D normalTexture;
layout(set = 3, binding = 4) uniform sampler2D heightTexture;

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec2 inUV;
layout(location = 2) in flat uvec4 inSegmentation;
layout(location = 3) in mat3 inTbn;

layout(location = 0) out vec4 outLighting;
layout(location = 1) out vec4 outAlbedo;
layout(location = 2) out vec4 outPosition;
layout(location = 3) out vec4 outSpecular;
layout(location = 4) out vec4 outNormal;
layout(location = 5) out uvec4 outSegmentation;
layout(location = 6) out vec4 outCustom;


//=== lighting resources ===//
layout(push_constant) uniform PushConstants {
  float visibility;
} pushConstants;

layout(binding = 0, set = 1) uniform CameraUBO {
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

vec4 world2camera(vec4 pos) {
  return cameraUBO.viewMatrix * pos;
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
  outSegmentation = inSegmentation;

  outPosition = inPosition;
  if (material.hasColorTexture != 0) {
    outAlbedo = texture(colorTexture, inUV);
  } else {
    outAlbedo = material.baseColor;
  }

  float finalAlpha = outAlbedo.a * pushConstants.visibility *  (1 - material.transparency);
  if (finalAlpha <= 0) {
    discard;
  }

  if (material.hasSpecularTexture != 0) {
    outSpecular.r = texture(specularTexture, inUV).r;
  } else {
    outSpecular.r = material.specular;
  }
  outSpecular.g = material.roughness;
  outSpecular.b = material.metallic;

  if (material.hasNormalTexture != 0) {
    outNormal = vec4(normalize(inTbn * texture(normalTexture, inUV).xyz), 0);
  } else {
    outNormal = vec4(normalize(inTbn * vec3(0, 0, 1)), 0);
  }

  vec3 albedo = outAlbedo.rgb;
  vec3 srm = outSpecular.xyz;
  float F0 = srm.x;
  float roughness = srm.y;
  float metallic = srm.z;

  vec3 normal = outNormal.xyz;
  vec4 csPosition = outPosition;
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
  outLighting = vec4(color, finalAlpha);
}
