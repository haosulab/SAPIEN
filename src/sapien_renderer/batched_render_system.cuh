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
void update_object_transforms(float **scene_transform_buffers, int transform_stride,
                              RenderShapeData *render_shapes, float *poses, int pose_stride,
                              int count, CUstream_st *stream);

/** The first 32 numbers must be are view matrix and inverse view matrix */
void update_camera_transforms(CameraData *cameras, float *poses, int pose_stride, int count,
                              CUstream_st *stream);

} // namespace sapien_renderer
} // namespace sapien