#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(binding = 0, set = 1) uniform CameraUBO {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
} cameraUBO;

layout(binding = 0, set = 2) uniform ObjectUBO {
  mat4 modelMatrix;
  uvec4 segmentation;
  mat4 userData;
} objectUBO;


layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 tangent;
layout(location = 4) in vec3 bitangent;

layout(location = 0) out vec4 outPosition;
layout(location = 1) out vec2 outUV;
layout(location = 2) out flat uvec4 outSegmentation;
layout(location = 3) out vec3 objectCoord;
layout(location = 4) out mat3 outTbn;

void main() {
  outSegmentation = objectUBO.segmentation;

  mat4 modelView = cameraUBO.viewMatrix * objectUBO.modelMatrix;
  mat3 normalMatrix = mat3(transpose(inverse(modelView)));

  // objectCoord = pos;
  objectCoord = normalize(mat3(transpose(inverse(objectUBO.modelMatrix))) * normal);

  outPosition = modelView * vec4(pos, 1);
  outUV = uv;
  gl_Position = cameraUBO.projectionMatrix * outPosition;

  vec3 outTangent = normalize(normalMatrix * tangent);
  vec3 outBitangent = normalize(normalMatrix * bitangent);
  vec3 outNormal = normalize(normalMatrix * normal);
  outTbn = mat3(outTangent, outBitangent, outNormal);
}
