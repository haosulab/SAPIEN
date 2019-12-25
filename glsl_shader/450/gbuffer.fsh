#version 450

uniform struct Material {
  vec4 kd;
  vec3 ks;
  vec3 ka;
  float ke;

  bool has_kd_map;
  bool has_ks_map;
  bool has_height_map;
  bool has_normal_map;

  sampler2D kd_map;
  sampler2D ks_map;
  sampler2D height_map;
  sampler2D normal_map;
} material;

layout (location=0) out vec4 GCOLOR;
layout (location=1) out vec4 GSPECULAR;
layout (location=2) out vec4 GNORMAL;

in vec2 texcoord;
in mat3 tbn;
in vec4 cameraSpacePosition;

void main() {
  if (material.has_kd_map) {
    GCOLOR = texture(material.kd_map, texcoord);
    if (GCOLOR.a == 0) {
      discard;
    }
  } else {
    GCOLOR = vec4(material.kd.rgb, 1);
  }
  if (material.has_ks_map) {
    GSPECULAR = texture(material.ks_map, texcoord);
  } else {
    GSPECULAR = vec4(material.ks, material.ke);
  }

  if (material.has_height_map) {
    const vec2 size = vec2(2.0,0.0);
    const ivec3 off = ivec3(-1,0,1);
    const float heightScale = 4.0;

    float s11 = texture(material.height_map, texcoord).x;
    float s01 = textureOffset(material.height_map, texcoord, off.xy).x;
    float s21 = textureOffset(material.height_map, texcoord, off.zy).x;
    float s10 = textureOffset(material.height_map, texcoord, off.yx).x;
    float s12 = textureOffset(material.height_map, texcoord, off.yz).x;
    vec3 va = normalize(vec3(size.xy, heightScale * (s21-s01)));
    vec3 vb = normalize(vec3(size.yx, heightScale * (s12-s10)));
    vec3 n = cross(va,vb);
    GNORMAL = vec4(normalize(tbn * n), 1);
  } else {
    GNORMAL = vec4(normalize(tbn * vec3(0,0,1)), 1);
  }
}
