#version 450

in vec2 texcoord;

uniform sampler2D colortex0;  // albedo
uniform sampler2D colortex1;  // 
uniform sampler2D colortex2;  // normal
uniform sampler2D depthtex0;  // depth

uniform mat4 gbufferViewMatrix;
uniform mat4 gbufferViewMatrixInverse;
uniform mat4 gbufferProjectionMatrix;
uniform mat4 gbufferProjectionMatrixInverse;

uniform int debug;

out vec4 FragColor;

#define N_DIRECTION_LIGHTS 5
struct DirectionalLight
{
  vec3 direction;
  vec3 emission;
};

#define N_POINT_LIGHTS 5
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

void main() {
  vec3 albedo = texture(colortex0, texcoord).xyz;
  vec3 normal = texture(colortex2, texcoord).xyz;
  vec4 csPosition = getCameraSpacePosition(texcoord);

  vec3 camDir = -normalize(csPosition.xyz);

  vec3 color = vec3(0.f);

  for (int i = 0; i < N_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(pointLights[i].position, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    float d = length(l);
    vec3 lightDir = normalize(l);

    color += albedo * pointLights[i].emission * max(0, dot(lightDir, normal)) / d / d;
  }

  for (int i = 0; i < N_DIRECTION_LIGHTS; i++) {
    // TODO: need use inverse transpose?
    vec3 lightDir = -normalize((gbufferViewMatrix * vec4(directionalLights[i].direction, 0)).xyz);
    color += albedo * directionalLights[i].emission * max(0, dot(lightDir, normal));
  }

  FragColor = vec4(normal, 1.f);
  // FragColor = vec4(-csPosition.zzz / 10.f, 1.f);
}
