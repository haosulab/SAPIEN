#version 130
#extension GL_ARB_explicit_attrib_location : enable
#extension GL_ARB_gpu_shader5 : enable

uniform mat4 gbufferModelMatrix;
uniform mat4 gbufferViewMatrix;
uniform mat4 gbufferProjectionMatrix;

layout(location=0) in vec3 vpos;
layout(location=1) in vec3 vnormal;

out vec4 cameraSpacePosition;

void main() {
  mat3 normalMatrix = mat3(transpose(inverse(gbufferViewMatrix * gbufferModelMatrix)));
  cameraSpacePosition = gbufferViewMatrix * gbufferModelMatrix * vec4(vpos, 1.f);
  gl_Position    = gbufferProjectionMatrix * cameraSpacePosition;
}
