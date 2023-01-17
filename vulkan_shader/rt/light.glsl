struct DirectionalLight {
  vec3 direction;
  float softness;
  vec3 rgb;
  float padding;
};

struct PointLight {
  vec3 position;
  float radius;
  vec3 rgb;
  float padding;
};

struct SpotLight {
  mat4 viewMat;
  mat4 projMat;
  vec3 rgb;
  int padding0;
  vec3 position;  // FIXME: this is redundant?
  int padding1;
  float fovInner;
  float fovOuter;
  int textureId;
  int padding2;
};
