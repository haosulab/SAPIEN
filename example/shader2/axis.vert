#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (constant_id = 0) const uint NUM_INSTANCES = 16;

layout(binding = 0, set = 1) uniform CameraUBO {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
} cameraUBO;

layout(binding = 0, set = 0) uniform ObjectUBO {
  mat4 modelMatrix[NUM_INSTANCES];
} objectUBO;


layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 tangent;
layout(location = 4) in vec3 bitangent;

layout(location = 0) out vec3 outColor;

void main() {
  mat4 modelView = cameraUBO.viewMatrix * objectUBO.modelMatrix[gl_InstanceIndex];
  gl_Position = cameraUBO.projectionMatrix * modelView * vec4(pos, 1);
  outColor = normal;
}
