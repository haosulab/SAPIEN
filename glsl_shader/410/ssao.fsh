#version 410

in vec2 texcoord;

uniform sampler2D colortex0;  // albedo
uniform sampler2D colortex1;  // 
uniform sampler2D colortex2;  // normal

uniform sampler2D depthtex0;  // depth

uniform sampler2D randomtex;
uniform int randomtexWidth;
uniform int randomtexHeight;

uniform int viewWidth;
uniform int viewHeight;

out vec4 FragColor;

uniform mat4 gbufferProjectionMatrix;
uniform mat4 gbufferProjectionMatrixInverse;

const int N_SAMPLE = 16;
const float BIAS = 0.01;
const float RADIUS = 0.3;

vec3 sampleDirection(vec2 coord, int index) {
  float z = texture(randomtex, coord * vec2(2 * N_SAMPLE * viewWidth / randomtexWidth + 2 * index,
                                                viewHeight / randomtexHeight)).x;
  float u = 6.283185307179586 *
            texture(randomtex, coord * vec2(2 * N_SAMPLE * viewWidth / randomtexWidth + 2 * index + 1,
                                                viewHeight / randomtexHeight)).x;
  float xy = sqrt(1-z*z);

  return vec3(xy * sin(u), xy * cos(u), z);
}

vec4 tex2camera(vec4 pos) {
  vec4 ndc = pos * 2.f - 1.f;
  vec4 cam = gbufferProjectionMatrixInverse * ndc;
  return cam / cam.w;
}

vec4 getCameraSpacePosition(vec2 texcoord) {
  float depth = texture(depthtex0, texcoord).x;
  return tex2camera(vec4(texcoord, depth, 1.f));
}

void main() {
  vec3 normal = texture(colortex2, texcoord).xyz;
  vec3 u1 = cross(normal, vec3(0,0,1));
  if (dot(u1, u1) < 0.1) {
    u1 = cross(normal, vec3(0,1,0));
  }
  u1 = normalize(u1);
  vec3 u2 = cross(normal, u1);
  mat3 tbn = mat3(u1, u2, normal);

  vec4 csPosition = getCameraSpacePosition(texcoord);

  float occlusion = 0;
  for (int i = 0; i < N_SAMPLE; ++i) {
    vec3 dir = sampleDirection(texcoord, i);
    dir = tbn * dir;

    vec3 position = csPosition.xyz + dir * RADIUS;
    vec4 offset = gbufferProjectionMatrix * vec4(position, 1.f);
    offset /= offset.w;
    offset = offset * 0.5 + 0.5;

    vec4 offsetPosition = getCameraSpacePosition(offset.xy);
    vec3 dist = offsetPosition.xyz - position.xyz;

    if (offset.x >= 0 && offset.x <= 1 && offset.y >= 0 && offset.y <= 1) {
      if (position.z - BIAS <= offsetPosition.z) {
        occlusion += smoothstep(0.f, 1.f, RADIUS / max(RADIUS, offsetPosition.z - position.z));
      }
    }
  }
  occlusion /= N_SAMPLE;

  FragColor = vec4(vec3(1-occlusion), 1);
}
