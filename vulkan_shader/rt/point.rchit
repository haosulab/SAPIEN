#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_nonuniform_qualifier : enable

hitAttributeEXT vec3 attribs;

#include "random.glsl"
#include "ray.glsl"
#include "geometry.glsl"
#include "light.glsl"
#include "push_constant.glsl"

layout(location = 0) rayPayloadInEXT Ray ray;
layout(location = 1) rayPayloadInEXT ShadowRay shadowRay;

#define SET_NUM 1
#include "./scene_set.glsl"
#undef SET_NUM

#include "./shading.glsl"

void main() {
  vec3 worldPosition = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
  vec3 objectPos = gl_ObjectRayOriginEXT + gl_ObjectRayDirectionEXT * gl_HitTEXT;
  int id = pointInstances[nonuniformEXT(gl_InstanceID)];
  Point p = points[nonuniformEXT(id)].p[gl_PrimitiveID];
  vec3 normal = objectPos - p.position;
  vec3 worldShadingNormal = normalize(vec3(normal * gl_WorldToObjectEXT));

  vec3 baseColor = p.color.xyz;
  float alpha = p.color.a;

  float roughness = 0.1;
  vec3 emission = vec3(0);
  float metallic = 0.0;
  float transmission = 0.0;
  float specular = 0.5;
  float ior = 1.2;

  transmission = 1.0 - clamp(alpha, 0.0, 1.0) * (1.0 - transmission);

  float diffuseWeight = (1.0 - metallic) * (1.0 - transmission);
  transmission = (1.0 - metallic) * transmission;
  float specularWeight = 1.0 - transmission;

  vec3 specularColor = (specular * 0.08) * (1.0 - metallic) + baseColor * metallic;
  vec3 diffuseColor = baseColor * diffuseWeight;

  float diffuseProb = maxVec3(diffuseColor);
  float specularProb = maxVec3(specularColor);
  float transmissionProb = transmission;
  float total = diffuseProb + specularProb + transmissionProb;

  diffuseProb = diffuseProb / total;
  specularProb = specularProb / total;
  transmissionProb = transmissionProb / total;

  // HACK: avoid extreme values to reduce fireflies
  if (diffuseProb > 1e-6) {
    diffuseProb += 0.1;
  }
  if (specularProb > 1e-6) {
    specularProb += 0.1;
  }
  if (transmissionProb > 1e-6) {
    transmissionProb += 0.1;
  }
  total = diffuseProb + specularProb + transmissionProb;
  diffuseProb = diffuseProb / total;
  specularProb = specularProb / total;
  transmissionProb = transmissionProb / total;

  float rand = rnd(ray.seed);

  vec3 V = normalize(-ray.direction);

  bool isInside = false;  // TODO

  vec3 wx = vec3(1, 0, 0);
  if (abs(dot(worldShadingNormal, vec3(1, 0, 0))) > 0.95) {
    wx = vec3(0, 1, 0);
  }
  vec3 wy = normalize(cross(worldShadingNormal, wx));
  wx = cross(wy, worldShadingNormal);
  mat3 tbn = mat3(wx, wy, worldShadingNormal);

  if (isInside) {
    tbn *= -1.0;
  }
  float eta = isInside ? ior : (1.0 / ior);

  vec3 attenuation = vec3(0.0);
  vec3 L = vec3(0.0);
  if (diffuseProb > 1e-5 && rand <= diffuseProb) {
    sampleDiffuse(ray.seed, tbn, V, diffuseColor, L, attenuation);
    attenuation = attenuation / diffuseProb;
  } else if (specularProb > 1e-5 && rand <= diffuseProb + specularProb) {  // specular
    sampleGGXReflection(ray.seed, tbn, V, roughness, specularColor, L, attenuation);
    attenuation /= specularProb;
  } else if (transmissionProb > 1e-5){  // transmission
    sampleGGXTransmission(ray.seed, tbn, V, eta, roughness, L, attenuation);
    attenuation *= transmission / transmissionProb * baseColor;
  }

  ray.radiance = emission
    + tracePointLights(worldPosition, worldShadingNormal, diffuseColor, specularColor, roughness, transmission, eta)
    + traceDirectionalLights(worldPosition, worldShadingNormal, diffuseColor, specularColor, roughness, transmission, eta)
    + traceSpotLights(worldPosition, worldShadingNormal, diffuseColor, specularColor, roughness, transmission, eta)
    + traceParallelogramLights(worldPosition, worldShadingNormal, diffuseColor, specularColor, roughness, transmission, eta);

  ray.origin = worldPosition;
  ray.direction = L;
  ray.attenuation = attenuation;
  ray.normal = worldShadingNormal;
  ray.albedo = baseColor;
  ray.segmentation = uvec4(0);
}
