#version 410

in vec2 texcoord;

uniform sampler2D shadowtex;
uniform mat4 cameraToShadowMatrix;
uniform mat4 shadowProjectionMatrix;
uniform vec3 shadowLightDirection;
uniform vec3 shadowLightEmission;

uniform sampler2D colortex0;  // albedo
uniform sampler2D colortex1;  // 
uniform sampler2D colortex2;  // normal
uniform sampler2D depthtex0;  // depth
uniform samplerCube skybox;

uniform mat4 gbufferViewMatrix;
uniform mat4 gbufferViewMatrixInverse;
uniform mat4 gbufferProjectionMatrix;
uniform mat4 gbufferProjectionMatrixInverse;
uniform mat4 environmentViewMatrix;
uniform mat4 environmentViewMatrixInverse;

uniform vec3 ambientLight;

out vec4 FragColor;

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

vec4 tex2camera(vec4 pos) {
  vec4 ndc = pos * 2.f - 1.f;
  vec4 cam = gbufferProjectionMatrixInverse * ndc;
  return cam / cam.w;
}

vec4 world2camera(vec4 pos) {
  return gbufferViewMatrix * pos;
}

vec4 getCameraSpacePosition(vec2 texcoord) {
  float depth = texture(depthtex0, texcoord).x;
  return tex2camera(vec4(texcoord, depth, 1.f));
}

vec3 getBackgroundColor(vec3 texcoord) {
  float r = sqrt(texcoord.x * texcoord.x + texcoord.y * texcoord.y);
  float angle = atan(texcoord.z / r) * 57.3;

  vec3 horizonColor = vec3(0.9, 0.9, 0.9);
  vec3 zenithColor = vec3(0.522, 0.757, 0.914);
  vec3 groundColor = vec3(0.5, 0.410, 0.271);
  
  return mix(mix(zenithColor, horizonColor, smoothstep(15.f, 5.f, angle)),
             groundColor,
             smoothstep(-5.f, -15.f, angle));
}

const float eps = 0.1;

// The Oren-Nayar shading model
float orenNayar(vec3 l, vec3 v, vec3 n, float r) {
  float a = r * r;
  float NoL = clamp(dot(n, l), 0.0, 1.0);
  float NoV = clamp(dot(n, v), 0.0, 1.0);
  float LoV = clamp(dot(l, v), 0.0, 1.0);
  float NLNV = NoL * NoV;

  if (NoL < 0.0 || NoV < 0.0) return 0.0;

  float A = 1.0 - 0.5 * (a / (a + 0.33));
  float B = 0.45 * (a / (a + 0.09));
  float C = max(0.0, LoV - NLNV) / max(NoL, NoV);

  return min(max(0.0, NoL) * (A + B * C), 1.0);
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

void main() {
  vec3 albedo = texture(colortex0, texcoord).xyz;
  vec3 srm = texture(colortex1, texcoord).xyz;
  float F0 = srm.x;
  float roughness = srm.y;
  float metallic = srm.z;
  // vec3 specular = mix(vec3(1.f, 1.f, 1.f), albedo, metallic);

  vec3 normal = texture(colortex2, texcoord).xyz;
  vec4 csPosition = getCameraSpacePosition(texcoord);

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

  float depth = texture(depthtex0, texcoord).x;
  if (depth == 1) {
    // FragColor = texture(skybox, (environmentViewMatrixInverse * csPosition).xyz);
    // FragColor = vec4(getBackgroundColor((gbufferViewMatrixInverse * csPosition).xyz), 1.f);
    FragColor = vec4(1,1,1,0);
  } else {
    FragColor = vec4(color, 1.f);
  }
  // FragColor.r = pow(FragColor.r, 1.f / 2.2f);
  // FragColor.g = pow(FragColor.g, 1.f / 2.2f);
  // FragColor.b = pow(FragColor.b, 1.f / 2.2f);
}
