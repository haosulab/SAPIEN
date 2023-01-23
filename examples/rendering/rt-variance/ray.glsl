struct Ray {
  vec3 origin;
  vec3 direction;
  vec3 albedo;
  vec3 normal;
  vec3 radiance;
  vec3 attenuation;
  uvec4 segmentation;
  uint depth;
  uint done;
  uint seed;
};

struct ShadowRay {
  bool shadowed;
};
