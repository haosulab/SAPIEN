#version 450
uniform mat4 gbufferModelMatrix;
uniform mat4 gbufferViewMatrix;
uniform mat4 gbufferProjectionMatrix;

layout(location=0) in vec3 vpos;
layout(location=1) in vec3 vnormal;

out vec4 cameraSpacePosition;

void main() {
  cameraSpacePosition = gbufferViewMatrix * gbufferModelMatrix * vec4(vpos, 1.f);
  gl_Position    = gbufferProjectionMatrix * cameraSpacePosition;
}
