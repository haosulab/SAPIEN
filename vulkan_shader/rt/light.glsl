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
  vec3 position;
  int padding1;
  float fovInner;
  float fovOuter;
  int textureId;
  int padding2;
};

struct ParallelogramLight {
  vec3 rgb;
  float padding0;
  vec3 position;
  float padding1;
  vec3 edge0;
  float padding2;
  vec3 edge1;
  float padding3;
};
