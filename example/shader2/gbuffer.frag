#version 450

layout(set = 3, binding = 0) uniform MaterialUBO {
  vec4 baseColor;  // rgba
  float specular;
  float roughness;
  float metallic;
  float transparency;
  int hasColorTexture;
  int hasSpecularTexture;
  int hasNormalTexture;
  int hasHeightTexture;
} material;

layout(set = 3, binding = 1) uniform sampler2D colorTexture;
layout(set = 3, binding = 2) uniform sampler2D specularTexture;
layout(set = 3, binding = 3) uniform sampler2D normalTexture;
layout(set = 3, binding = 4) uniform sampler2D heightTexture;

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec2 inUV;
layout(location = 2) in flat uvec4 inSegmentation;
layout(location = 3) in float intensity;
layout(location = 4) in mat3 inTbn;

layout(location = 0) out vec4 outAlbedo;
layout(location = 1) out vec4 outPosition;
layout(location = 2) out vec4 outSpecular;
layout(location = 3) out vec4 outNormal;
layout(location = 4) out uvec4 outSegmentation;
layout(location = 5) out vec4 outCustom;

void main() {
  outCustom = vec4(0,0,0,1);
  outSegmentation = inSegmentation;

  outPosition = inPosition;
  if (material.hasColorTexture != 0) {
    outAlbedo = texture(colorTexture, inUV) * intensity;
  } else {
    outAlbedo = material.baseColor * intensity;
  }

  if (outAlbedo.a == 0) {
    discard;
  }

  if (material.hasSpecularTexture != 0) {
    outSpecular.r = texture(specularTexture, inUV).r;
  } else {
    outSpecular.r = material.specular;
  }
  outSpecular.g = material.roughness;
  outSpecular.b = material.metallic;

  if (material.hasNormalTexture != 0) {
    outNormal = vec4(normalize(inTbn * texture(normalTexture, inUV).xyz), 0);
  } else {
    outNormal = vec4(normalize(inTbn * vec3(0, 0, 1)), 0);
  }
}
