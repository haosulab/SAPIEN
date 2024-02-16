
layout(set = SET_NUM, binding = 0) uniform ObjectTransformBuffer {
  mat4 modelMatrix;
} objectTransformBuffer;

layout(set = SET_NUM, binding = 1) uniform ObjectDataBuffer {
  uvec4 segmentation;
  float transparency;
  int shadeFlat;
} objectDataBuffer;
