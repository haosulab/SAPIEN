#version 450

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

uniform int debug;
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

const float eps = 0.01;

// The Oren-Nayar shading model
float orenNayar(vec3 l, vec3 v, vec3 n, float r) {
  float a = r * r;
  float NoL = clamp(dot(n, l), 0, 1);
  float NoV = clamp(dot(n, v), 0, 1);
  float LoV = clamp(dot(l, v), 0, 1);
  float NLNV = NoL * NoV;

  if (NoL < 0 || NoV <0) return 0;

  float A = 1 - 0.5 * (a / (a + 0.33));
  float B = 0.45 * (a / (a + 0.09));
  float C = max(0, LoV - NLNV) / max(NoL, NoV);

  return min(max(0, NoL) * (A + B * C), 1);
}

float ggx (vec3 L, vec3 V, vec3 N, float roughness, float F0) {
  float alpha = roughness*roughness;
  vec3 H = normalize(L + V);
  float dotLH = clamp(dot(L,H), 0, 1);
  float dotNH = clamp(dot(N,H), 0, 1);
  float dotNL = clamp(dot(N,L), 0, 1);

  float alphaSqr = alpha * alpha;
  float denom = dotNH * dotNH * (alphaSqr - 1.0) + 1.0;
  float D = alphaSqr / (3.141592653589793 * denom * denom);
  float F = F0 + (1.0 - F0) * pow(1.0 - dotLH, 5.0);
  float k = 0.5 * alpha;
  float k2 = k * k;
  return dotNL * D * F / (dotLH*dotLH*(1.0-k2)+k2);
}

void main() {
  vec3 albedo = texture(colortex0, texcoord).xyz;
  vec4 specular = texture(colortex1, texcoord);
  float roughness = 2 / (2 + specular.a);
  vec3 normal = texture(colortex2, texcoord).xyz;
  vec4 csPosition = getCameraSpacePosition(texcoord);

  vec4 ssPosition = cameraToShadowMatrix * vec4((csPosition.xyz + normal * eps), 1);
  vec4 shadowMapCoord = shadowProjectionMatrix * ssPosition;
  shadowMapCoord /= shadowMapCoord.w;
  shadowMapCoord = shadowMapCoord * 0.5 + 0.5;  // convert to 0-1
  float visibility = step(shadowMapCoord.z - texture(shadowtex, shadowMapCoord.xy).r, 0);

  vec3 camDir = -normalize(csPosition.xyz);

  vec3 color = vec3(0.f);
  for (int i = 0; i < N_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(pointLights[i].position, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    float d = max(length(l), 0.0001);
    vec3 lightDir = normalize(l);

    // point light diffuse
    color += albedo * pointLights[i].emission * orenNayar(lightDir, camDir, normal, 0.3f) / d / d;
    color += specular.rgb * pointLights[i].emission
             * ggx(lightDir, camDir, normal, roughness, 0.05) / d / d;
  }

  vec3 lightDir = -normalize((gbufferViewMatrix * vec4(shadowLightDirection, 0)).xyz);
  color += albedo * shadowLightEmission * orenNayar(lightDir, camDir, normal, 0.3f) * visibility;
  color += specular.rgb * shadowLightEmission * ggx(lightDir, camDir, normal, roughness, 0.05) * visibility;

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
  FragColor.r = pow(FragColor.r, 1.f / 2.2f);
  FragColor.g = pow(FragColor.g, 1.f / 2.2f);
  FragColor.b = pow(FragColor.b, 1.f / 2.2f);
}
