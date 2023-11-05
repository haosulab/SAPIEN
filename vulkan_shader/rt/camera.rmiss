#version 460
#extension GL_EXT_ray_tracing : require

#include "ray.glsl"
#include "push_constant.glsl"

layout(location=0) rayPayloadInEXT Ray ray;

layout(set = 1, binding = 10) uniform samplerCube samplerEnvironment;  // TODO: check this

void main() {
  vec3 dir = ray.direction;
  dir = vec3(-dir.y, dir.z, -dir.x);
  if (envmap != 0) {
    // FIXME: super high values seem to cause nan
    ray.radiance = clamp(texture(samplerEnvironment, dir).xyz, vec3(0,0,0), vec3(100, 100, 100));
  } else {
    ray.radiance = ambientLight;
  }

  ray.albedo = ray.radiance;
  ray.normal = vec3(0.0);
  ray.segmentation = uvec4(0);

  ray.attenuation = vec3(0.0);
  ray.depth = maxDepth + 1;
  ray.origin = vec3(0.0);
  ray.alpha = 0.0;
}
