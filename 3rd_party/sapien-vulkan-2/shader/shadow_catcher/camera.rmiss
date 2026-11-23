#version 460
#extension GL_EXT_ray_tracing : require

#include "ray.glsl"
#include "push_constant.glsl"

layout(location=0) rayPayloadInEXT Ray ray;

layout(set = 1, binding = 10) uniform samplerCube samplerEnvironment;  // TODO: check this

void main() {
  ray.radiance = vec3(0.0);
  ray.albedo = ray.radiance;
  ray.normal = vec3(0.0);
  ray.segmentation = uvec4(0);
  ray.attenuation = vec3(0.0);
  ray.depth = maxDepth + 1;
  ray.origin = vec3(0.0);
  ray.alpha = 0.0;
}
