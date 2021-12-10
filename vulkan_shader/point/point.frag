#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (constant_id = 0) const int NUM_DIRECTIONAL_LIGHTS = 3;
layout (constant_id = 1) const int NUM_POINT_LIGHTS = 10;
layout (constant_id = 2) const int NUM_SPOT_LIGHTS = 10;
layout (constant_id = 3) const float RESOLUTION_SCALE = 1.0;

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in flat vec4 inNdcRadius;

layout(location = 0) out vec4 outPointColor;
layout(location = 1) out vec4 outPointDepthLinear;

layout(set = 0, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  mat4 prevViewMatrix;
  mat4 prevViewMatrixInverse;
  float width;
  float height;
} cameraBuffer;

void main() {
  vec2 centerNdc = inNdcRadius.xy;
  vec2 res = vec2(cameraBuffer.width, cameraBuffer.height) * RESOLUTION_SCALE;
  vec2 pixelNdc = gl_FragCoord.xy / res * 2.0 - 1.0;
  vec2 offsetNdc = pixelNdc - centerNdc;
  vec2 offset = offsetNdc * (-inPosition.z) / vec2(cameraBuffer.projectionMatrix[0][0], cameraBuffer.projectionMatrix[1][1]);
  float radius = inNdcRadius.w;
  offset /= radius;
  if (offset.x * offset.x + offset.y * offset.y > 1) {
    discard;
  }
  vec3 normal = vec3(offset, sqrt(max(0.0, 1 - offset.x * offset.x - offset.y * offset.y)));
  vec4 csPosition = vec4(inPosition.xyz + normal * radius, 1.0);

  vec4 ndc = cameraBuffer.projectionMatrix * csPosition;
  ndc /= ndc.w;
  gl_FragDepth = ndc.z;

  outPointColor = inColor;
  outPointDepthLinear.x = csPosition.z;
  // outPoint1 = inColor;
  // outPoint1DepthLinear = vec4(csPosition.z);
}
