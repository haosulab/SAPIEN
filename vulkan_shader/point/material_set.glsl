
layout(set = SET_NUM, binding = 0) uniform MaterialBuffer {
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
} materialBuffer;

layout(set = SET_NUM, binding = 1) uniform sampler2D colorTexture;
layout(set = SET_NUM, binding = 2) uniform sampler2D roughnessTexture;
layout(set = SET_NUM, binding = 3) uniform sampler2D normalTexture;
layout(set = SET_NUM, binding = 4) uniform sampler2D metallicTexture;
layout(set = SET_NUM, binding = 5) uniform sampler2D emissionTexture;
layout(set = SET_NUM, binding = 6) uniform sampler2D transmissionTexture;
