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

void unpackVertex(uint index, uint geometryIndex, out vec3 position, out vec3 normal, out vec2 uv, out vec3 tangent) {
  Vertex v = vertices[nonuniformEXT(geometryIndex)].v[index];
  position = vec3(v.x, v.y, v.z);
  normal = vec3(v.nx, v.ny, v.nz);
  uv = vec2(v.u, v.v);
  tangent = vec3(v.tx, v.ty, v.tz);
}

#include "./shading.glsl"


void main() {
  Object obj = objects.o[gl_InstanceID];

  int instanceId = gl_InstanceCustomIndexEXT + gl_GeometryIndexEXT;
  uint geometryIndex = geometryInstances.i[instanceId].geometryIndex;
  uint materialIndex = geometryInstances.i[instanceId].materialIndex;

  uint v0 = indices[nonuniformEXT(geometryIndex)].i[3 * gl_PrimitiveID + 0];
  uint v1 = indices[nonuniformEXT(geometryIndex)].i[3 * gl_PrimitiveID + 1];
  uint v2 = indices[nonuniformEXT(geometryIndex)].i[3 * gl_PrimitiveID + 2];

  vec3 p0, p1, p2;
  vec3 n0, n1, n2;
  vec2 uv0, uv1, uv2;
  vec3 t0, t1, t2;
  unpackVertex(v0, geometryIndex, p0, n0, uv0, t0);
  unpackVertex(v1, geometryIndex, p1, n1, uv1, t1);
  unpackVertex(v2, geometryIndex, p2, n2, uv2, t2);

  float b0 = 1.0 - attribs.x - attribs.y;
  float b1 = attribs.x;
  float b2 = attribs.y;

  vec3 geometricNormal = cross(p1 - p0, p2 - p0);
  vec3 shadingNormal = obj.shadeFlat != 0 ? geometricNormal : (n0 * b0 + n1 * b1 + n2 * b2);
  if (dot(shadingNormal, shadingNormal) < 1e-3) {
    shadingNormal = geometricNormal;
  }

  vec2 uv = uv0 * b0 + uv1 * b1 + uv2 * b2;
  vec3 tangent = t0 * b0 + t1 * b1 + t2 * b2;

  if (length(tangent) < 0.01) {
    tangent = normalize(p1 - p0);
  }

  // TODO: check inverse transpose
  vec3 worldGeometricNormal = normalize(vec3(geometricNormal * gl_WorldToObjectEXT));
  vec3 worldShadingNormal = normalize(vec3(shadingNormal * gl_WorldToObjectEXT));
  vec3 worldPosition = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
  vec3 worldTangent = normalize(vec3(tangent * gl_WorldToObjectEXT));

  Material mat = materials[nonuniformEXT(materialIndex)].m;
  TextureIndex ti = textureIndices.t[materialIndex];

  vec3 baseColor = mat.baseColor.rgb;
  float alpha = 1.0;
  if (ti.diffuse >= 0) {
    vec4 baseColorAlpha = texture(textures[nonuniformEXT(ti.diffuse)], uv * mat.textureTransforms[0].zw + mat.textureTransforms[0].xy);
    baseColor = baseColorAlpha.rgb;
    alpha = baseColorAlpha.a;
  }

  float metallic = mat.metallic;
  if (ti.metallic >= 0) {
    metallic = texture(textures[nonuniformEXT(ti.metallic)], uv * mat.textureTransforms[3].zw + mat.textureTransforms[3].xy).x;
  }

  float roughness = mat.roughness;
  if (ti.roughness >= 0) {
    roughness = texture(textures[nonuniformEXT(ti.roughness)], uv * mat.textureTransforms[1].zw + mat.textureTransforms[1].xy).x;
  }
  float a2 = roughness * roughness;

  vec3 texNormal = vec3(0.0, 0.0, 1.0);
  if (ti.normal >= 0) {
    texNormal = normalize(texture(textures[nonuniformEXT(ti.normal)], uv * mat.textureTransforms[2].zw + mat.textureTransforms[2].xy).xyz * 2.0 - 1.0);
  }

  vec3 emission = mat.emission.rgb;
  float strength = mat.emission.a;
  if (ti.emission >= 0) {
    emission *= texture(textures[nonuniformEXT(ti.emission)], uv * mat.textureTransforms[4].zw + mat.textureTransforms[4].xy).rgb;
  }
  emission *= strength;

  float transmission = mat.transmission;
  if (ti.transmission >= 0) {
    transmission = texture(textures[nonuniformEXT(ti.transmission)], uv * mat.textureTransforms[5].zw + mat.textureTransforms[5].xy).x;
  }


  if (rnd(ray.seed) < obj.transparency) {
    ray.origin = worldPosition;
    ray.attenuation = vec3(1.0);
    ray.normal = worldShadingNormal;
    ray.albedo = baseColor;
    ray.segmentation = obj.segmentation;
    ray.radiance = vec3(0.0);
    return;
  }

  float specular = max(mat.fresnel, 0.0);

  transmission = clamp(transmission, 0.0, 1.0);
  transmission = 1.0 - clamp(alpha, 0.0, 1.0) * (1.0 - transmission);

  metallic = clamp(metallic, 0.0, 1.0);
  float ior = clamp(mat.ior, 1.01, 4.0);

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

  bool isInside = gl_HitKindEXT == gl_HitKindBackFacingTriangleEXT;

  vec3 worldShadingBitangent = normalize(cross(worldShadingNormal, worldTangent));
  mat3 tbn = mat3(cross(worldShadingBitangent, worldShadingNormal), worldShadingBitangent, worldShadingNormal);

  // process normal map
  worldShadingNormal = tbn * texNormal;
  worldShadingBitangent = normalize(cross(worldShadingNormal, worldTangent));
  tbn = mat3(cross(worldShadingBitangent, worldShadingNormal), worldShadingBitangent, worldShadingNormal);

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
  ray.segmentation = obj.segmentation;
}
