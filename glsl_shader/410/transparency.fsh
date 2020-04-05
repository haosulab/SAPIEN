#version 410

// GBuffer Uniforms
uniform struct Material {
  vec4 kd;
  float ks;

  float roughness;
  float metallic;

  bool has_kd_map;
  bool has_ks_map;
  bool has_height_map;
  bool has_normal_map;

  sampler2D kd_map;
  sampler2D ks_map;
  sampler2D height_map;
  sampler2D normal_map;
} material;

uniform int segmentation;
uniform int segmentation2;
uniform vec3 segmentation_color;

layout (location=0) out vec4 GCOLOR;
layout (location=1) out vec4 GSPECULAR;
layout (location=2) out vec4 GNORMAL;
layout (location=3) out int GSEGMENTATION;
layout (location=4) out int GSEGMENTATION2;
layout (location=5) out vec4 GSEGMENTATIONCOLOR;
layout (location=6) out vec4 GUSER;
layout (location=7) out vec4 LIGHTING;

in vec2 texcoord;
in mat3 tbn;
in vec4 cameraSpacePosition;
in vec4 custom;

// Lighting uniforms
#define N_DIRECTION_LIGHTS 5
struct DirectionalLight
{
  vec3 direction;
  vec3 emission;
};

#define N_POINT_LIGHTS 3
struct PointLight
{
  vec3 position;
  vec3 emission;
};

uniform DirectionalLight directionalLights[N_DIRECTION_LIGHTS];
uniform PointLight pointLights[N_POINT_LIGHTS];
uniform vec3 ambientLight;

uniform sampler2D shadowtex;
uniform mat4 cameraToShadowMatrix;
uniform mat4 shadowProjectionMatrix;
uniform vec3 shadowLightDirection;
uniform vec3 shadowLightEmission;

uniform mat4 gbufferViewMatrix;
uniform mat4 gbufferViewMatrixInverse;
uniform mat4 gbufferProjectionMatrix;
uniform mat4 gbufferProjectionMatrixInverse;
uniform mat4 environmentViewMatrix;
uniform mat4 environmentViewMatrixInverse;

vec4 world2camera(vec4 pos) {
  return gbufferViewMatrix * pos;
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

float ggx(vec3 wi, vec3 wo, vec3 normal, float roughness,
          float ks) {
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

float eps = 0.1;

void main() {
  // Geometry processing
  vec4 COLOR = vec4(0.f, 0.f, 0.f, 0.f);
  if (material.has_kd_map) {
    COLOR = texture(material.kd_map, texcoord);
    if (COLOR.a == 0) {
      discard;
    }
  } else {
    COLOR = material.kd;
  }
  GCOLOR = vec4(COLOR.rgb, 1.f);

  float alpha = COLOR.a;

  if (material.has_ks_map) {
    GSPECULAR.r = texture(material.ks_map, texcoord).r;
  } else {
    GSPECULAR.r = material.ks;
  }
  GSPECULAR.g = material.roughness;
  GSPECULAR.b = material.metallic;

  GSEGMENTATION = segmentation;
  GSEGMENTATION2 = segmentation2;
  GSEGMENTATIONCOLOR = vec4(segmentation_color, 1);
  GUSER = custom;

  if (material.has_height_map) {
    const vec2 size = vec2(2.0,0.0);
    const ivec3 off = ivec3(-1,0,1);
    const float heightScale = 4.0;

    float s11 = texture(material.height_map, texcoord).x;
    float s01 = textureOffset(material.height_map, texcoord, off.xy).x;
    float s21 = textureOffset(material.height_map, texcoord, off.zy).x;
    float s10 = textureOffset(material.height_map, texcoord, off.yx).x;
    float s12 = textureOffset(material.height_map, texcoord, off.yz).x;
    vec3 va = normalize(vec3(size.xy, heightScale * (s21-s01)));
    vec3 vb = normalize(vec3(size.yx, heightScale * (s12-s10)));
    vec3 n = cross(va,vb);
    GNORMAL = vec4(normalize(tbn * n), 1);
  } else {
    GNORMAL = vec4(normalize(tbn * vec3(0,0,1)), 1);
  }

  // Lighting
  vec3 albedo = GCOLOR.rgb;
  vec3 srm = GSPECULAR.xyz;
  float F0 = srm.x;
  float roughness = srm.y;
  float metallic = srm.z;
  vec3 specular = mix(vec3(1.f, 1.f, 1.f), albedo, metallic);

  vec3 normal = GNORMAL.xyz;
  vec4 csPosition = cameraSpacePosition;

  vec4 ssPosition = cameraToShadowMatrix * vec4((csPosition.xyz + normal * eps), 1);
  vec4 shadowMapCoord = shadowProjectionMatrix * ssPosition;
  shadowMapCoord /= shadowMapCoord.w;
  shadowMapCoord = shadowMapCoord * 0.5 + 0.5;  // convert to 0-1
  float visibility = step(shadowMapCoord.z - texture(shadowtex, shadowMapCoord.xy).r, 0);
  if (shadowMapCoord.x <= 0 || shadowMapCoord.x >= 1 || shadowMapCoord.y <= 0 || shadowMapCoord.y >= 1) {
    visibility = 1;
  }

  vec3 camDir = -normalize(csPosition.xyz);

  vec3 color = vec3(0.f);
  for (int i = 0; i < N_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(pointLights[i].position, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    float d = max(length(l), 0.0001);
    vec3 lightDir = normalize(l);

    // diffuse
    color += (1 - metallic) * albedo * pointLights[i].emission *
             diffuse(lightDir, camDir, normal) / d / d;

    // metallic
    color += metallic * albedo * pointLights[i].emission *
             ggx(lightDir, camDir, normal, roughness, 1.f) / d / d;

    // specular
    color += pointLights[i].emission * ggx(lightDir, camDir, normal, roughness, F0) / d / d;
  }

  vec3 lightDir = -normalize((gbufferViewMatrix * vec4(shadowLightDirection, 0)).xyz);

  // diffuse
  color += (1.f - metallic) * albedo * shadowLightEmission
           * diffuse(lightDir, camDir, normal) * visibility;

  // metallic
  color += metallic * albedo * shadowLightEmission
           * ggx(lightDir, camDir, normal, roughness, 1.f) * visibility;

  // specular
  color += shadowLightEmission * ggx(lightDir, camDir, normal, roughness, F0) * visibility;

  // for (int i = 0; i < N_DIRECTION_LIGHTS; i++) {
  //   vec3 lightDir = -normalize((gbufferViewMatrix * vec4(directionalLights[i].direction, 0)).xyz);
  //   color += albedo * directionalLights[i].emission * max(0, dot(lightDir, normal));
  // }

  color += ambientLight * albedo;

  // LIGHTING = vec4(color, alpha);
  LIGHTING = vec4(color, alpha);
}

