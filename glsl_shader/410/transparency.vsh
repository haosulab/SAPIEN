#version 410

uniform mat4 gbufferModelMatrix;
uniform mat4 gbufferModelMatrixInverse;
uniform mat4 gbufferViewMatrix;
uniform mat4 gbufferViewMatrixInverse;
uniform mat4 gbufferProjectionMatrix;
uniform mat4 user_data;

layout(location=0) in vec3 vpos;
layout(location=1) in vec3 vnormal;
layout(location=2) in vec2 vtexcoord;
layout(location=3) in vec3 vtangent;
layout(location=4) in vec3 vbitangent;

out vec2 texcoord;
out mat3 tbn;
out vec4 cameraSpacePosition;
out vec4 custom;

void main() {
  mat3 normalMatrix = mat3(transpose(gbufferModelMatrixInverse * gbufferViewMatrixInverse));

  cameraSpacePosition = gbufferViewMatrix * gbufferModelMatrix * vec4(vpos, 1.f);
  gl_Position    = gbufferProjectionMatrix * cameraSpacePosition;
  texcoord       = vtexcoord;
  vec3 tangent   = normalize(normalMatrix * vtangent);
  vec3 bitangent = normalize(normalMatrix * vbitangent);
  vec3 normal    = normalize(normalMatrix * vnormal);
  tbn            = mat3(tangent, bitangent, normal);
  custom         = user_data * vec4(vpos, 1);
}
