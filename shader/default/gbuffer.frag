#version 450

layout(set = 3, binding = 0) uniform MaterialBuffer {
  vec4 baseColor;
  float fresnel;
  float roughness;
  float metallic;
  float transparency;
  int textureMask;
} materialBuffer;

layout(set = 3, binding = 1) uniform sampler2D colorTexture;
layout(set = 3, binding = 2) uniform sampler2D roughnessTexture;
layout(set = 3, binding = 3) uniform sampler2D normalTexture;
layout(set = 3, binding = 4) uniform sampler2D metallicTexture;

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec2 inUV;
layout(location = 2) in flat uvec4 inSegmentation;
layout(location = 3) in vec3 inObjectCoord;
layout(location = 4) in mat3 inTbn;

// required output textures
layout(location = 0) out vec4 outAlbedo;
layout(location = 1) out vec4 outPosition;
layout(location = 2) out vec4 outSpecular;
layout(location = 3) out vec4 outNormal;
layout(location = 4) out uvec4 outSegmentation;

// custom output textures
layout(location = 5) out vec4 outObjectCoord;

void main() {
  outObjectCoord = vec4(inObjectCoord, 1);
  outSegmentation = inSegmentation;
  outPosition = inPosition;

  outAlbedo = mix(materialBuffer.baseColor, texture(colorTexture, inUV), float(materialBuffer.textureMask & 1));
  if (outAlbedo.a == 0) {
    discard;
  }

  outSpecular.r = materialBuffer.fresnel;
  outSpecular.g = mix(materialBuffer.roughness, texture(roughnessTexture, inUV).r,
                      float((materialBuffer.textureMask >> 1) & 1));
  outSpecular.b = mix(materialBuffer.metallic, texture(metallicTexture, inUV).r,
                      float((materialBuffer.textureMask >> 3) & 1));

  vec3 normal = mix(vec3(0,0,1), texture(normalTexture, inUV).xyz, float((materialBuffer.textureMask >> 2) & 1));
  outNormal = vec4(normalize(inTbn * normal), 1);
}
