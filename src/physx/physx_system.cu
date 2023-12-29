#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace {

struct PxBodyData {
  float4 q;
  float3 p;
  float padding0;
  float3 v;
  float padding1;
  float3 w;
  float padding2;
};
static_assert(sizeof(PxBodyData) == 64);

struct SapienBodyData {
  float pq[7];
  float3 v;
  float3 w;
};
static_assert(sizeof(SapienBodyData) == 52);

struct PxTransform {
  float q[4];
  float p[3];
};
static_assert(sizeof(PxTransform) == 28);

struct PxVelocity {
  float3 v;
  float3 w;
};
static_assert(sizeof(PxVelocity) == 24);

struct SapienTransform {
  float pq[7];
};
static_assert(sizeof(SapienTransform) == 28);

} // namespace

__global__ void body_data_physx_to_sapien(SapienBodyData *__restrict__ sapien_data,
                                          PxBodyData *__restrict__ physx_data,
                                          float3 *__restrict__ offset, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  SapienBodyData sd{};

  sd.pq[0] = physx_data[g].p.x - offset[g].x;
  sd.pq[1] = physx_data[g].p.y - offset[g].y;
  sd.pq[2] = physx_data[g].p.z - offset[g].z;

  sd.pq[3] = physx_data[g].q.w;
  sd.pq[4] = physx_data[g].q.x;
  sd.pq[5] = physx_data[g].q.y;
  sd.pq[6] = physx_data[g].q.z;

  sd.v = physx_data[g].v;
  sd.w = physx_data[g].w;

  sapien_data[g] = sd;
}

__global__ void body_data_sapien_to_physx(PxBodyData *__restrict__ physx_data,
                                          SapienBodyData *__restrict__ sapien_data,
                                          float3 *__restrict__ offset, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  SapienBodyData sd = sapien_data[g];

  PxBodyData pd{};
  pd.q = make_float4(sd.pq[4], sd.pq[5], sd.pq[6], sd.pq[3]);
  pd.p = make_float3(sd.pq[0] + offset[g].x, sd.pq[1] + offset[g].y, sd.pq[2] + offset[g].z);
  pd.v = sd.v;
  pd.w = sd.w;

  physx_data[g] = pd;
}

__global__ void body_data_sapien_to_physx(PxBodyData *__restrict__ physx_data,
                                          int4 *__restrict__ physx_index,
                                          SapienBodyData *__restrict__ sapien_data,
                                          int4 *__restrict__ sapien_index,
                                          int *__restrict__ apply_index,
                                          float3 *__restrict__ offset, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int i = apply_index[g];

  SapienBodyData sd = sapien_data[i];

  PxBodyData pd{};
  pd.q = make_float4(sd.pq[4], sd.pq[5], sd.pq[6], sd.pq[3]);
  pd.p = make_float3(sd.pq[0] + offset[i].x, sd.pq[1] + offset[i].y, sd.pq[2] + offset[i].z);
  pd.v = sd.v;
  pd.w = sd.w;

  physx_data[g] = pd;
  physx_index[g] = sapien_index[i];
}

__global__ void link_pose_physx_to_sapien(SapienBodyData *__restrict__ sapien_data,
                                          PxTransform *__restrict__ physx_pose,
                                          float3 *__restrict__ offset, int link_count, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = g / link_count;

  sapien_data[g].pq[0] = physx_pose[g].p[0] - offset[ai].x;
  sapien_data[g].pq[1] = physx_pose[g].p[1] - offset[ai].y;
  sapien_data[g].pq[2] = physx_pose[g].p[2] - offset[ai].z;

  sapien_data[g].pq[3] = physx_pose[g].q[3];
  sapien_data[g].pq[4] = physx_pose[g].q[0];
  sapien_data[g].pq[5] = physx_pose[g].q[1];
  sapien_data[g].pq[6] = physx_pose[g].q[2];
}

__global__ void root_pose_sapien_to_physx(PxTransform *__restrict__ physx_pose,
                                          SapienBodyData *__restrict__ sapien_data,
                                          int *__restrict__ index, float3 *__restrict__ offset,
                                          int link_count, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = index[g]; // ith articulation

  SapienBodyData sd = sapien_data[ai * link_count];

  physx_pose[ai].q[0] = sd.pq[4];
  physx_pose[ai].q[1] = sd.pq[5];
  physx_pose[ai].q[2] = sd.pq[6];
  physx_pose[ai].q[3] = sd.pq[3];

  physx_pose[ai].p[0] = sd.pq[0] + offset[ai].x;
  physx_pose[ai].p[1] = sd.pq[1] + offset[ai].y;
  physx_pose[ai].p[2] = sd.pq[2] + offset[ai].z;
}

__global__ void link_vel_physx_to_sapien(SapienBodyData *__restrict__ sapien_data,
                                         PxVelocity *__restrict__ physx_vel, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  sapien_data[g].v = physx_vel[g].v;
  sapien_data[g].w = physx_vel[g].w;
}

__global__ void root_vel_sapien_to_physx(PxVelocity *__restrict__ physx_vel,
                                         SapienBodyData *__restrict__ sapien_data,
                                         int *__restrict__ index, int link_count, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = index[g];
  SapienBodyData sd = sapien_data[ai * link_count];

  physx_vel[ai].v = sd.v;
  physx_vel[ai].w = sd.w;
}

namespace sapien {

constexpr int BLOCK_SIZE = 128;

void body_data_physx_to_sapien(void *sapien_data, void *physx_data, void *offset, int count,
                               cudaStream_t stream) {
  ::body_data_physx_to_sapien<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (SapienBodyData *)sapien_data, (PxBodyData *)physx_data, (float3 *)offset, count);
}

void body_data_sapien_to_physx(void *physx_data, void *sapien_data, void *offset, int count,
                               cudaStream_t stream) {
  ::body_data_sapien_to_physx<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (PxBodyData *)physx_data, (SapienBodyData *)sapien_data, (float3 *)offset, count);
}

void body_data_sapien_to_physx(void *physx_data, void *physx_index, void *sapien_data,
                               void *sapien_index, void *apply_index, void *offset, int count,
                               cudaStream_t stream) {
  ::body_data_sapien_to_physx<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (PxBodyData *)physx_data, (int4 *)physx_index, (SapienBodyData *)sapien_data,
      (int4 *)sapien_index, (int *)apply_index, (float3 *)offset, count);
}

void link_pose_physx_to_sapien(void *sapien_data, void *physx_pose, void *offset, int link_count,
                               int count, cudaStream_t stream) {
  ::link_pose_physx_to_sapien<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (SapienBodyData *)sapien_data, (PxTransform *)physx_pose, (float3 *)offset, link_count,
      count);
}

void root_pose_sapien_to_physx(void *physx_pose, void *sapien_data, void *index, void *offset,
                               int link_count, int count, cudaStream_t stream) {
  ::root_pose_sapien_to_physx<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (PxTransform *)physx_pose, (SapienBodyData *)sapien_data, (int *)index, (float3 *)offset,
      link_count, count);
}

void link_vel_physx_to_sapien(void *sapien_data, void *physx_vel, int count, cudaStream_t stream) {
  ::link_vel_physx_to_sapien<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (SapienBodyData *)sapien_data, (PxVelocity *)physx_vel, count);
}

void root_vel_sapien_to_physx(void *physx_vel, void *sapien_data, void *index, int link_count,
                              int count, cudaStream_t stream) {
  ::root_vel_sapien_to_physx<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      (PxVelocity *)physx_vel, (SapienBodyData *)sapien_data, (int *)index, link_count, count);
}

// void body_data_physx_to_sapien_subtract_offset(void *output, void *data, void *index, void
// *offset,
//                                                int count, void *stream) {
//   body_data_physx_to_sapien_subtract_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE,
//   BLOCK_SIZE, 0,
//                                               (cudaStream_t)stream>>>(
//       (SapienBodyData *)output, (PxBodyData *)data, (int4 *)index, (float4 *)offset, count);
// }

// void body_data_sapien_to_physx_add_offset(void *output, void *data, void *index, void *offset,
//                                           int count, void *stream) {
//   body_data_sapien_to_physx_add_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
//                                          (cudaStream_t)stream>>>(
//       (PxBodyData *)output, (SapienBodyData *)data, (int4 *)index, (float4 *)offset, count);
// }

// void transform_sapien_to_physx_add_offset(void *output, void *data, int link_count, void
// *index,
//                                           void *offset, int count, void *stream) {
//   transform_sapien_to_physx_add_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
//                                          (cudaStream_t)stream>>>(
//       (PxTransform *)output, (SapienTransform *)data, link_count, (int *)index, (float4
//       *)offset, count);
// }

// void transform_physx_to_sapien_subtract_offset(void *output, void *data, int link_count,
//                                                void *index, void *offset, int count,
//                                                void *stream) {
//   transform_physx_to_sapien_subtract_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE,
//   BLOCK_SIZE, 0,
//                                               (cudaStream_t)stream>>>(
//       (SapienTransform *)output, (PxTransform *)data, link_count, (int *)index, (float4
//       *)offset, count);
// }

} // namespace sapien
