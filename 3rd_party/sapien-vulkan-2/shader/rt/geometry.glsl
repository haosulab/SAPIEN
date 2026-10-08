struct GeometryInstance {
  uint geometryIndex;
  uint materialIndex;
  int padding0;
  int padding1;
};

struct Material {
  vec4 emission;
  vec4 baseColor;
  float fresnel;
  float roughness;
  float metallic;
  float transmission;
  float ior;
  float transmissionRoughness;
  int textureMask;
  int padding1;
  vec4 textureTransforms[6];
};

struct TextureIndex {
  int diffuse;
  int metallic;
  int roughness;
  int emission;
  int normal;
  int transmission;
  int occlusion;
  int padding0;
};
