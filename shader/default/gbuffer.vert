#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable


layout(set = 1, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
} cameraBuffer;

layout(set = 2, binding = 0) uniform ObjectBuffer {
  mat4 modelMatrix;
  uvec4 segmentation;
  mat4 userData;  // optional
} objectBuffer;

layout(location = 0) in vec3 position;  // must be at location 0
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 tangent;
layout(location = 4) in vec3 bitangent;
// layout(location = 5) in vec4 color;

layout(location = 0) out vec4 outPosition;  // world space position
layout(location = 1) out vec2 outUV;
layout(location = 2) out flat uvec4 outSegmentation;
layout(location = 3) out vec3 outObjectCoord;
layout(location = 4) out mat3 outTbn;

void main() {
  outSegmentation = objectBuffer.segmentation;

  mat4 modelView = cameraBuffer.viewMatrix * objectBuffer.modelMatrix;
  mat3 normalMatrix = mat3(transpose(inverse(modelView)));

  outObjectCoord = position;

  outPosition = modelView * vec4(position, 1);
  outUV = uv;
  gl_Position = cameraBuffer.projectionMatrix * outPosition;

  vec3 outTangent = normalize(normalMatrix * tangent);
  vec3 outBitangent = normalize(normalMatrix * bitangent);
  vec3 outNormal = normalize(normalMatrix * normal);
  outTbn = mat3(outTangent, outBitangent, outNormal);
}
