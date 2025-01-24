/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "./batched_render_system.cuh"
#include "sapien/math/pose.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <cstdio>

namespace sapien {
namespace sapien_renderer {

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

__global__ void update_object_transforms_kernel(
    float *__restrict__ *__restrict__ scene_transform_buffers, // output buffers
    int transform_stride, RenderShapeData *__restrict__ shapes,
    float *__restrict__ poses, // parent pose array
    int pose_stride, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  RenderShapeData shape = shapes[g];

  Vec3 scale = shape.scale;
  Pose pose_s2b = shape.localPose;

  int pose_index = shape.poseIndex;
  Pose pose_b2w =
      Pose(Vec3(poses[pose_stride * pose_index], poses[pose_stride * pose_index + 1],
                poses[pose_stride * pose_index + 2]),
           Quat(poses[pose_stride * pose_index + 3], poses[pose_stride * pose_index + 4],
                poses[pose_stride * pose_index + 5], poses[pose_stride * pose_index + 6]));
  Pose p = pose_b2w * pose_s2b;

  int scene_index = shape.sceneIndex;
  int transform_index = shape.transformIndex;

  PoseToMatrix(scene_transform_buffers[scene_index] + transform_index * transform_stride, p, scale);
}

__global__ void update_camera_transforms_kernel(CameraData *cameras, float *poses, int pose_stride,
                                                int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }
  CameraData camera = cameras[g];
  int pose_index = camera.poseIndex;
  float *buffer = (float *)camera.buffer;

  Pose pose_b2w =
      Pose(Vec3(poses[pose_stride * pose_index], poses[pose_stride * pose_index + 1],
                poses[pose_stride * pose_index + 2]),
           Quat(poses[pose_stride * pose_index + 3], poses[pose_stride * pose_index + 4],
                poses[pose_stride * pose_index + 5], poses[pose_stride * pose_index + 6]));
  Pose pose_c2b = camera.localPose;

  Pose viewInverse = pose_b2w * pose_c2b;

  // viewInverse: 32-48
  PoseToMatrix(buffer + 32, viewInverse, Vec3(1.f));

  // view: 0-16
  PoseToMatrix(buffer, viewInverse.getInverse(), Vec3(1.f));
}

constexpr int BLOCK_SIZE = 128;

void update_object_transforms(float **scene_transform_buffers, int transform_stride,
                              RenderShapeData *render_shapes, float *poses, int pose_stride,
                              int count, CUstream_st *stream) {
  update_object_transforms_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                    (cudaStream_t)stream>>>(
      scene_transform_buffers, transform_stride, render_shapes, poses, pose_stride, count);
}

void update_camera_transforms(CameraData *cameras, float *poses, int pose_stride, int count,
                              CUstream_st *stream) {
  update_camera_transforms_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                    (cudaStream_t)stream>>>(cameras, poses, pose_stride, count);
}

} // namespace sapien_renderer
} // namespace sapien