#version 450 

layout(set = 0, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
} cameraBuffer;

layout(set = 1, binding = 0) uniform sampler2D samplerNormal;
layout(set = 1, binding = 1) uniform sampler2D samplerGbufferDepth;
layout(set = 1, binding = 2) uniform sampler2D samplerRandom300x300_0;
layout(set = 1, binding = 3) uniform sampler2D samplerRandom300x300_1;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outAOMap;


const int N_SAMPLE = 16;
const float BIAS = 0.015;
const float RADIUS = 0.5;

vec3 sampleHemisphere(vec2 coord, int index, int width, int height) {
  float z = texture(samplerRandom300x300_0, coord * vec2((N_SAMPLE * width + index) / 300.f,
                                            height / 300.f)).x;
  float u = 6.283185307179586 *
            texture(samplerRandom300x300_1, coord * vec2((N_SAMPLE * width + index) / 300.f,
                                            height / 300.f)).x;
  float xy = sqrt(1-z*z);

  float scale = index / float(N_SAMPLE);
  scale = mix(0.1, 1.0, scale * scale);

  return scale * vec3(xy * sin(u), xy * cos(u), z);
}

mat3 onb(vec3 normal, vec2 coord, int width, int height) {
  float a = 6.283185307179586 * texture(samplerRandom300x300_0, coord * vec2(width / 300.f,
                                                         height / 300.f)).x;
  float c = cos(a);
  float s = sin(a);

  vec3 u1 = cross(normal, vec3(1,0,0));
  if (dot(u1, u1) < 0.01) {
    u1 = cross(normal, vec3(0,1,0));
  }
  u1 = normalize(u1);
  vec3 u2 = cross(normal, u1);
  mat3 tbn = mat3(c * u1 + s * u2, -s * u1 * c * u2, normal);
  return tbn;
}

void main() {
  ivec2 size = textureSize(samplerGbufferDepth, 0);

  vec3 normal = normalize(texture(samplerNormal, inUV).xyz * 2 - 1);
  mat3 tbn = onb(normal, inUV, size.x, size.y);

  float depth = texture(samplerGbufferDepth, inUV).x;
  vec4 csPosition = cameraBuffer.projectionMatrixInverse * (vec4(inUV * 2 - 1, depth, 1));
  csPosition /= csPosition.w;

  float occlusion = 0.f;
  for (int i = 0; i < N_SAMPLE; ++i) {
    vec3 dir = sampleHemisphere(inUV, i, size.x, size.y);
    dir = tbn * dir;
    vec3 position = csPosition.xyz + dir * RADIUS;
    vec4 offset = cameraBuffer.projectionMatrix * vec4(position, 1.f);
    offset /= offset.w;

    if (offset.x < -1) {
      offset.x = -2 - offset.x;
    }
    if (offset.x > 1) {
      offset.x =  2 - offset.x;
    }
    if (offset.y < -1) {
      offset.y = -2 - offset.y;
    }
    if (offset.y > 1) {
      offset.y =  2 - offset.y;
    }

    float offsetDepth = texture(samplerGbufferDepth, offset.xy * 0.5 + 0.5).x;
    vec4 tmp = cameraBuffer.projectionMatrixInverse * vec4(0, 0, offsetDepth, 1);
    offsetDepth = tmp.z / tmp.w;

    float rangeCheck = smoothstep(0.0, 1.0, RADIUS / abs(depth - offsetDepth));
    occlusion += (offsetDepth >= position.z + BIAS ? 1.0 : 0.0) * rangeCheck;
  }
  occlusion /= N_SAMPLE;
  outAOMap = vec4(vec3(1 - occlusion), 1);
}
