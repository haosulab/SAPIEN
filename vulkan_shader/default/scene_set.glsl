// usage
// #define SET_NUM 1
// #include "scene_set.glsl"
// #undef SET_NUM

#include "../common/lights.glsl"
#include "../common/shadow.glsl"

layout(set = SET_NUM, binding = 0) uniform SceneBuffer {
  vec4 ambientLight;
  DirectionalLight directionalLights[3];
  SpotLight spotLights[10];
  PointLight pointLights[10];
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

layout(set = SET_NUM, binding = 1) uniform ShadowBuffer {
  LightBuffer directionalLightBuffers[1];
  LightBuffer spotLightBuffers[10];
  LightBuffer pointLightBuffers[18];
  LightBuffer texturedLightBuffers[1];
} shadowBuffer;

layout(set = SET_NUM, binding = 2) uniform samplerCube samplerPointLightDepths[3];
layout(set = SET_NUM, binding = 3) uniform sampler2D samplerDirectionalLightDepths[1];
layout(set = SET_NUM, binding = 4) uniform sampler2D samplerTexturedLightDepths[1];
layout(set = SET_NUM, binding = 5) uniform sampler2D samplerSpotLightDepths[10];
layout(set = SET_NUM, binding = 6) uniform sampler2D samplerTexturedLightTextures[1];

layout(set = SET_NUM, binding = 7) uniform samplerCube samplerEnvironment;
layout(set = SET_NUM, binding = 8) uniform sampler2D samplerBRDFLUT;
