#include "./batched_render_system.cuh"
#include "sapien/math/pose.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <cstdio>

using sapien::Pose;
using sapien::Quat;
using sapien::Vec3;

namespace {

inline CUDA_CALLABLE void PoseToMatrix(float *result, Pose const &pose, Vec3 const &scale) {
  Vec3 c0 = pose.q.rotate(Vec3(1, 0, 0));
  Vec3 c1 = pose.q.rotate(Vec3(0, 1, 0));
  Vec3 c2 = pose.q.rotate(Vec3(0, 0, 1));

  result[0] = c0.x * scale.x;
  result[1] = c0.y * scale.x;
  result[2] = c0.z * scale.x;
  result[3] = 0.f;

  result[4] = c1.x * scale.y;
  result[5] = c1.y * scale.y;
  result[6] = c1.z * scale.y;
  result[7] = 0.f;

  result[8] = c2.x * scale.z;
  result[9] = c2.y * scale.z;
  result[10] = c2.z * scale.z;
  result[11] = 0.f;

  result[12] = pose.p.x;
  result[13] = pose.p.y;
  result[14] = pose.p.z;
  result[15] = 1.f;
}

} // namespace

__global__ void transform_sapien_to_render(
    float *__restrict__ *__restrict__ scene_transform_buffers, // output buffers
    sapien::sapien_renderer::RenderShapeData *__restrict__ shapes,
    float *__restrict__ poses, // parent pose array
    int pose_stride, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  sapien::sapien_renderer::RenderShapeData shape = shapes[g];

  Vec3 scale(shape.scale[0], shape.scale[1], shape.scale[2]);
  Pose pose_s2b =
      Pose(Vec3(shape.localPos[0], shape.localPos[1], shape.localPos[2]),
           Quat(shape.localRot[0], shape.localRot[1], shape.localRot[2], shape.localRot[3]));

  int pose_index = shape.poseIndex;
  Pose pose_b2w =
      Pose(Vec3(poses[pose_stride * pose_index], poses[pose_stride * pose_index + 1],
                poses[pose_stride * pose_index + 2]),
           Quat(poses[pose_stride * pose_index + 3], poses[pose_stride * pose_index + 4],
                poses[pose_stride * pose_index + 5], poses[pose_stride * pose_index + 6]));
  Pose p = pose_b2w * pose_s2b;

  int scene_index = shape.sceneIndex;
  int transform_index = shape.transformIndex;

  printf("tid: %d, pose: %d, scene: %d, transform: %d\n", g, pose_index, scene_index, transform_index);

  PoseToMatrix(scene_transform_buffers[scene_index] + transform_index * 16, p, scale);
}

// __global__ void transform_sapien_to_camera_view_model(
//     float *__restrict__ *__restrict__ camera_buffers, float *__restrict__ local_transforms,
//     int *__restrict__ parent_indices, float *__restrict__ parent_transforms,
//     int parent_transform_stride, int count) {
//   int g = blockIdx.x * blockDim.x + threadIdx.x;
//   if (g >= count) {
//     return;
//   }

//   Pose pose_s2b =
//       Pose(Vec3(local_transforms[7 * g], local_transforms[7 * g + 1], local_transforms[7 * g + 2]),
//            Quat(local_transforms[7 * g + 3], local_transforms[7 * g + 4],
//                 local_transforms[7 * g + 5], local_transforms[7 * g + 6]));

//   int parent_index = parent_indices[g];
//   Pose pose_b2w = Pose(Vec3(parent_transforms[parent_transform_stride * parent_index],
//                             parent_transforms[parent_transform_stride * parent_index + 1],
//                             parent_transforms[parent_transform_stride * parent_index + 2]),
//                        Quat(parent_transforms[parent_transform_stride * parent_index + 3],
//                             parent_transforms[parent_transform_stride * parent_index + 4],
//                             parent_transforms[parent_transform_stride * parent_index + 5],
//                             parent_transforms[parent_transform_stride * parent_index + 6]));
//   Pose p = pose_b2w * pose_s2b;

//   p.getInverse().toMatrix(camera_buffers[g], Vec3(1));
//   p.toMatrix(camera_buffers[g] + 16, Vec3(1));
// }


namespace sapien {
namespace sapien_renderer {
constexpr int BLOCK_SIZE = 128;

void transform_sapien_to_render(float **scene_transform_buffers, RenderShapeData *render_shapes,
                                float *poses, int pose_stride, int count, CUstream_st *stream) {
  ::transform_sapien_to_render<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                 (cudaStream_t)stream>>>(scene_transform_buffers, render_shapes,
                                                         poses, pose_stride, count);
}

} // namespace sapien_renderer
} // namespace sapien
