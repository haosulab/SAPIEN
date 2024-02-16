#version 450

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 3;
layout (constant_id = 2) const int NUM_DIRECTIONAL_LIGHT_SHADOWS = 1;
layout (constant_id = 3) const int NUM_POINT_LIGHT_SHADOWS = 1;
layout (constant_id = 4) const int NUM_TEXTURED_LIGHT_SHADOWS = 1;
layout (constant_id = 5) const int NUM_SPOT_LIGHT_SHADOWS = 1;
layout (constant_id = 6) const int NUM_SPOT_LIGHTS = 1;

#include "../common/lights.glsl"

layout(set = 0, binding = 0) uniform sampler2D samplerColorRaw;
layout(set = 0, binding = 1) uniform sampler2D samplerGbufferDepth;

layout(set = 1, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  float width;
  float height;
} cameraBuffer;

layout(set = 2, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  PointLight pointLights[10];
  DirectionalLight directionalLights[3];
  SpotLight spotLights[10];
  SpotLight texturedLights[1];
} sceneBuffer;

struct LightBuffer {
  mat4 viewMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrix;
  mat4 projectionMatrixInverse;
  int width;
  int height;
};

layout(set = 2, binding = 1) uniform ShadowBuffer {
  LightBuffer pointLightBuffers[60];
  LightBuffer directionalLightBuffers[3];
  LightBuffer spotLightBuffers[10];
  LightBuffer texturedLightBuffers[1];
} shadowBuffer;

layout(set = 2, binding = 2) uniform samplerCube samplerPointLightDepths[10];
layout(set = 2, binding = 3) uniform sampler2D samplerDirectionalLightDepths[3];
layout(set = 2, binding = 4) uniform sampler2D samplerSpotLightDepths[10];
layout(set = 2, binding = 5) uniform sampler2D samplerTexturedLightDepths[1];
layout(set = 2, binding = 6) uniform sampler2D samplerTexturedLightTextures[1];
layout(set = 2, binding = 7) uniform samplerCube samplerEnvironment;
layout(set = 2, binding = 8) uniform sampler2D samplerBRDFLUT;

layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 outColor;

vec3 getBackgroundColor(vec3 texcoord) {
  texcoord = texcoord.xzy;
  return textureLod(samplerEnvironment, texcoord, 0).rgb;
}

void main() {
  vec4 color = texture(samplerColorRaw, inUV);

  float depth = texture(samplerGbufferDepth, inUV).x;
  vec4 csPosition = cameraBuffer.projectionMatrixInverse * (vec4(inUV * 2 - 1, depth, 1));
  csPosition /= csPosition.w;
  vec3 bg = pow(getBackgroundColor((cameraBuffer.viewMatrixInverse * csPosition).xyz), vec3(1/2.2));

  outColor.rgb = mix(bg, color.rgb, color.a);
  outColor.a = color.a;
}
