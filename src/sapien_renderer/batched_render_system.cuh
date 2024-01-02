#include "sapien/math/pose.h"

struct CUstream_st;

namespace sapien {
namespace sapien_renderer {

struct RenderShapeData {
  Pose localPose;
  Vec3 scale;
  int poseIndex;
  int sceneIndex;
  int transformIndex;
};

struct CameraData {
  void *buffer;
  Pose localPose;
  int poseIndex;
};

/**
 * Update render object transforms from  GPU poses in SAPIEN convention
 * scene_transform_buffers: 2D mat4 arrays representing model matrices of each object
 * render_shapes: render shape data corresponding to each RenderShape
 * poses: sapien poses in the format of p(xyz) q(wxyz)
 * pose_stride: number of floats (not bytes!) between consecutive poses
 * count: size of the render_shapes array
 * stream: cuda stream
 * */
void update_object_transforms(float **scene_transform_buffers, RenderShapeData *render_shapes,
                              float *poses, int pose_stride, int count, CUstream_st *stream);

/** The first 32 numbers must be are view matrix and inverse view matrix */
void update_camera_transforms(CameraData *cameras, float *poses, int pose_stride, int count,
                              CUstream_st *stream);

} // namespace sapien_renderer
} // namespace sapien
