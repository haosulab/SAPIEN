
struct CUstream_st;

namespace sapien {
namespace sapien_renderer {

struct RenderShapeData {
  float localRot[4];
  float localPos[3];
  float scale[3];
  int poseIndex;
  int sceneIndex;
  int transformIndex;
};

struct CameraData {
  void* buffer;
  float localRot[4];
  float localPos[3];
  int poseIndex;
};

void transform_sapien_to_render(float **scene_transform_buffers, RenderShapeData *render_shapes, float *poses, int pose_stride, int count, CUstream_st* stream);

}
}
