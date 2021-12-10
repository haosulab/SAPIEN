#version 450

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outPoint;

layout(set = 0, binding = 0) uniform sampler2D samplerSmoothedDepthLinear;
layout(set = 0, binding = 1) uniform sampler2D samplerPointColor;

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_SPOT_LIGHTS = 10;

layout(set = 1, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  mat4 prevViewMatrix;
  mat4 prevViewMatrixInverse;
  float width;
  float height;
} cameraBuffer;

#include "../common/lights.glsl"
layout(set = 2, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  DirectionalLight directionalLights[3];
  SpotLight spotLights[10];
  PointLight pointLights[10];
  SpotLight texturedLights[1];
} sceneBuffer;

vec3 getCameraSpacePosition(vec2 uv, float z) {
  float depth = -cameraBuffer.projectionMatrix[2][2] - cameraBuffer.projectionMatrix[3][2] / z;
  vec4 ndc = vec4(uv * 2.0 - 1.0, depth, 1.0);
  vec4 csPosition = cameraBuffer.projectionMatrixInverse * ndc;
  return vec3(csPosition.xy / csPosition.w, z);
}

vec4 world2camera(vec4 pos) {
  return cameraBuffer.viewMatrix * pos;
}

void main() {
  vec2 res = vec2(cameraBuffer.width, cameraBuffer.height);

  vec2 uv = inUV;
  float z = texture(samplerSmoothedDepthLinear, uv).x;
  if (z >= 0.0) {
    discard;
  }
  vec3 csp = getCameraSpacePosition(uv, z);

  uv = inUV + vec2(-1 / cameraBuffer.width, 0);
  vec3 cspnx = getCameraSpacePosition(uv, texture(samplerSmoothedDepthLinear, uv).x);

  uv = inUV + vec2(1 / cameraBuffer.width, 0);
  vec3 csppx = getCameraSpacePosition(uv, texture(samplerSmoothedDepthLinear, uv).x);

  uv = inUV + vec2(0, -1 / cameraBuffer.height);
  vec3 cspny = getCameraSpacePosition(uv, texture(samplerSmoothedDepthLinear, uv).x);

  uv = inUV + vec2(0, 1 / cameraBuffer.height);
  vec3 csppy = getCameraSpacePosition(uv, texture(samplerSmoothedDepthLinear, uv).x);

  vec3 pdx = csppx - csp;
  vec3 ndx = csp - cspnx;

  vec3 pdy = csppy - csp;
  vec3 ndy = csp - cspny;

  vec3 dx = abs(pdx.z) < abs(ndx.z) ? pdx: ndx;
  vec3 dy = abs(pdy.z) < abs(ndy.z) ? pdy: ndy;

  // vec3 dx;
  // if (abs(pdx.z) < abs(ndx.z)) {
  //   dx = pdx;
  // } else {
  //   dx = ndx;
  // }
  // vec3 dy = pdy;

  vec3 normal = normalize(cross(dy, dx));
  vec3 albedo = texture(samplerPointColor, inUV).rgb;

  // lighting
  vec4 csPosition = vec4(csp, 1.0);
  vec3 camDir = -normalize(csPosition.xyz);
  vec4 ndc = cameraBuffer.projectionMatrix * csPosition;
  ndc /= ndc.w;

  float roughness = 0.5;
  vec3 fresnel = vec3(0.2);

  vec3 color = vec3(0.0);
  for (int i = 0; i < NUM_POINT_LIGHTS; i++) {
    vec3 pos = world2camera(vec4(sceneBuffer.pointLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    color += computePointLight(sceneBuffer.pointLights[i].emission.rgb,
                               l, normal, camDir, albedo, roughness, fresnel);
  }

  for (int i = 0; i < NUM_DIRECTIONAL_LIGHTS; ++i) {
    color += computeDirectionalLight(mat3(cameraBuffer.viewMatrix) * sceneBuffer.directionalLights[i].direction.xyz, sceneBuffer.directionalLights[i].emission.rgb, normal, camDir, albedo, roughness, fresnel);
  }

  for (int i = 0; i < NUM_SPOT_LIGHTS; ++i) {
    vec3 pos = world2camera(vec4(sceneBuffer.spotLights[i].position.xyz, 1.f)).xyz;
    vec3 l = pos - csPosition.xyz;
    vec3 centerDir = mat3(cameraBuffer.viewMatrix) * sceneBuffer.spotLights[i].direction.xyz;
    color += computeSpotLight(sceneBuffer.spotLights[i].emission.a, sceneBuffer.spotLights[i].direction.a, centerDir, sceneBuffer.spotLights[i].emission.rgb, l, normal, camDir, albedo, roughness, fresnel);
  }

  color += sceneBuffer.ambientLight.rgb * albedo;

  outPoint.xyz = color;
}
