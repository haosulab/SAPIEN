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
  float padding0;
  float padding1;
  float padding2;
};
static_assert(sizeof(SapienBodyData) == 64);

struct PxTransformWrong {
  float4 q;
  float3 p;
};
static_assert(sizeof(PxTransformWrong) == 32);

struct PxTransform {
  float q[4];
  float p[3];
};
static_assert(sizeof(PxTransform) == 28);

struct SapienTransform {
  float pq[7];
};
static_assert(sizeof(SapienTransform) == 28);

} // namespace

__global__ void transform_sapien_to_physx_add_offset(SapienTransform *__restrict__ data,
                                                     int const *__restrict__ index,
                                                     float4 const *__restrict__ offset,
                                                     int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int i = index[g];

  union {
    PxTransform pd{};
    SapienTransform sd;
  };

  pd.q[0] = data[i].pq[4];
  pd.q[1] = data[i].pq[5];
  pd.q[2] = data[i].pq[6];
  pd.q[3] = data[i].pq[3];

  pd.p[0] = data[i].pq[0] + offset[i].x;
  pd.p[1] = data[i].pq[1] + offset[i].y;
  pd.p[2] = data[i].pq[2] + offset[i].z;

  data[i] = sd;
}

__global__ void transform_physx_to_sapien_subtract_offset(PxTransform *__restrict__ data,
                                                          int const *__restrict__ index,
                                                          float4 const *__restrict__ offset,
                                                          int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }
  int i = index[g];

  union {
    SapienBodyData sd{};
    PxTransform pd;
  };

  sd.pq[0] = data[i].p[0] - offset[i].x;
  sd.pq[1] = data[i].p[1] - offset[i].y;
  sd.pq[2] = data[i].p[2] - offset[i].z;

  sd.pq[3] = data[i].q[3];
  sd.pq[4] = data[i].q[0];
  sd.pq[5] = data[i].q[1];
  sd.pq[6] = data[i].q[2];

  data[i] = pd;
}

__global__ void body_data_sapien_to_physx_add_offset(SapienBodyData *__restrict__ data,
                                                     int4 const *__restrict__ index,
                                                     float4 const *__restrict__ offset,
                                                     int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }
  int i = index[g].x;

  union {
    PxBodyData pd{};
    SapienBodyData sd;
  };

  pd.q = make_float4(data[i].pq[4], data[i].pq[5], data[i].pq[6], data[i].pq[3]);
  pd.p = make_float3(data[i].pq[0] + offset[i].x, data[i].pq[1] + offset[i].y,
                     data[i].pq[2] + offset[i].z);
  pd.v = data[i].v;
  pd.w = data[i].w;

  data[i] = sd;
}

__global__ void body_data_physx_to_sapien_subtract_offset(PxBodyData *__restrict__ data,
                                                          int4 const *__restrict__ index,
                                                          float4 const *__restrict__ offset,
                                                          int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }
  int i = index[g].x;

  union {
    SapienBodyData sd{};
    PxBodyData pd;
  };

  sd.pq[0] = data[i].p.x - offset[i].x;
  sd.pq[1] = data[i].p.y - offset[i].y;
  sd.pq[2] = data[i].p.z - offset[i].z;

  sd.pq[3] = data[i].q.w;
  sd.pq[4] = data[i].q.x;
  sd.pq[5] = data[i].q.y;
  sd.pq[6] = data[i].q.z;

  sd.v = data[i].v;
  sd.w = data[i].w;

  data[i] = pd;
}

namespace sapien {

constexpr int BLOCK_SIZE = 128;

void body_data_physx_to_sapien_subtract_offset(void *data, void *index, void *offset, int count,
                                               void *stream) {
  body_data_physx_to_sapien_subtract_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                              (cudaStream_t)stream>>>(
      (PxBodyData *)data, (int4 *)index, (float4 *)offset, count);
}

void body_data_sapien_to_physx_add_offset(void *data, void *index, void *offset, int count,
                                          void *stream) {
  body_data_sapien_to_physx_add_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                         (cudaStream_t)stream>>>(
      (SapienBodyData *)data, (int4 *)index, (float4 *)offset, count);
}

void transform_sapien_to_physx_add_offset(void *data, void *index, void *offset, int count,
                                          void *stream) {
  transform_sapien_to_physx_add_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                         (cudaStream_t)stream>>>(
      (SapienTransform *)data, (int *)index, (float4 *)offset, count);
}

void transform_physx_to_sapien_subtract_offset(void *data, void *index, void *offset, int count,
                                               void *stream) {
  transform_physx_to_sapien_subtract_offset<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                              (cudaStream_t)stream>>>(
      (PxTransform *)data, (int *)index, (float4 *)offset, count);
}

}; // namespace sapien
