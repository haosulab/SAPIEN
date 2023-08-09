layout(push_constant) uniform Constants {
  vec3 ambientLight;
  int frameCount;
  int spp;
  int maxDepth;

  float aperture;
  float focusPlane;

  int russianRoulette;
  int russianRouletteMinBounces;

  int pointLightCount;
  int directionalLightCount;
  int spotLightCount;
  int parallelogramLightCount;

  int envmap;

  // post processing
  float exposure;
  int toneMapper;
};
