layout(push_constant) uniform Constants {
  vec3 ambientLight;
  int frameCount;
  int spp;
  int maxDepth;

  int russianRoulette;
  int russianRouletteMinBounces;

  int pointLightCount;
  int directionalLightCount;
  int spotLightCount;

  int envmap;
};
