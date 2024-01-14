#include "./physx_system.cuh"

namespace sapien {
namespace physx {

__global__ void body_data_physx_to_sapien_kernel(SapienBodyData *__restrict__ sapien_data,
                                                 PhysxBodyData *__restrict__ physx_data,
                                                 Vec3 *__restrict__ offset, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  sapien_data[g] = {
      .p = physx_data[g].pose.p - offset[g],
      .q = Quat(physx_data[g].pose.q.w, physx_data[g].pose.q.x, physx_data[g].pose.q.y,
                physx_data[g].pose.q.z),
      .v = physx_data[g].v,
      .w = physx_data[g].w,
  };
}

__global__ void body_data_sapien_to_physx_kernel(PhysxBodyData *__restrict__ physx_data,
                                                 SapienBodyData *__restrict__ sapien_data,
                                                 Vec3 *__restrict__ offset, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  SapienBodyData sd = sapien_data[g];

  PhysxBodyData pd;
  pd.pose.q = {sd.q.x, sd.q.y, sd.q.z, sd.q.w};
  pd.pose.p = sd.p + offset[g];
  pd.v = sd.v;
  pd.w = sd.w;

  physx_data[g] = pd;
}

__global__ void body_data_sapien_to_physx_kernel(PhysxBodyData *__restrict__ physx_data,
                                                 int4 *__restrict__ physx_index,
                                                 SapienBodyData *__restrict__ sapien_data,
                                                 int4 *__restrict__ sapien_index,
                                                 int *__restrict__ apply_index,
                                                 Vec3 *__restrict__ offset, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int i = apply_index[g];

  SapienBodyData sd = sapien_data[i];

  PhysxBodyData pd;
  pd.pose.q = {sd.q.x, sd.q.y, sd.q.z, sd.q.w};
  pd.pose.p = sd.p + offset[i];
  pd.v = sd.v;
  pd.w = sd.w;

  physx_data[g] = pd;
  physx_index[g] = sapien_index[i];
}

__global__ void link_pose_physx_to_sapien_kernel(SapienBodyData *__restrict__ sapien_data,
                                                 PhysxPose *__restrict__ physx_pose,
                                                 Vec3 *__restrict__ offset, int link_count,
                                                 int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = g / link_count;

  sapien_data[g].p = physx_pose[g].p - offset[ai];
  sapien_data[g].q =
      Quat(physx_pose[g].q.w, physx_pose[g].q.x, physx_pose[g].q.y, physx_pose[g].q.z);
}

__global__ void root_pose_sapien_to_physx_kernel(PhysxPose *__restrict__ physx_pose,
                                                 SapienBodyData *__restrict__ sapien_data,
                                                 int *__restrict__ index,
                                                 Vec3 *__restrict__ offset, int link_count,
                                                 int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = index[g]; // ith articulation

  SapienBodyData sd = sapien_data[ai * link_count];

  physx_pose[ai] = {.q = {sd.q.x, sd.q.y, sd.q.z, sd.q.w}, .p = sd.p + offset[ai]};
}

__global__ void link_vel_physx_to_sapien_kernel(SapienBodyData *__restrict__ sapien_data,
                                                PhysxVelocity *__restrict__ physx_vel, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  sapien_data[g].v = physx_vel[g].v;
  sapien_data[g].w = physx_vel[g].w;
}

__global__ void root_vel_sapien_to_physx_kernel(PhysxVelocity *__restrict__ physx_vel,
                                                SapienBodyData *__restrict__ sapien_data,
                                                int *__restrict__ index, int link_count,
                                                int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = index[g];
  SapienBodyData sd = sapien_data[ai * link_count];

  physx_vel[ai].v = sd.v;
  physx_vel[ai].w = sd.w;
}

constexpr int BLOCK_SIZE = 128;

void body_data_physx_to_sapien(void *sapien_data, void *physx_data, void *offset, int count,
                               cudaStream_t stream) {
  body_data_physx_to_sapien_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>((SapienBodyData *)sapien_data,
                                               (PhysxBodyData *)physx_data, (Vec3 *)offset, count);
}

void body_data_sapien_to_physx(void *physx_data, void *sapien_data, void *offset, int count,
                               cudaStream_t stream) {
  body_data_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>(
      (PhysxBodyData *)physx_data, (SapienBodyData *)sapien_data, (Vec3 *)offset, count);
}

void body_data_sapien_to_physx(void *physx_data, void *physx_index, void *sapien_data,
                               void *sapien_index, void *apply_index, void *offset, int count,
                               cudaStream_t stream) {
  body_data_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>((PhysxBodyData *)physx_data, (int4 *)physx_index,
                                               (SapienBodyData *)sapien_data, (int4 *)sapien_index,
                                               (int *)apply_index, (Vec3 *)offset, count);
}

void link_pose_physx_to_sapien(void *sapien_data, void *physx_pose, void *offset, int link_count,
                               int count, cudaStream_t stream) {
  link_pose_physx_to_sapien_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>(
      (SapienBodyData *)sapien_data, (PhysxPose *)physx_pose, (Vec3 *)offset, link_count, count);
}

void root_pose_sapien_to_physx(void *physx_pose, void *sapien_data, void *index, void *offset,
                               int link_count, int count, cudaStream_t stream) {
  root_pose_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>((PhysxPose *)physx_pose,
                                               (SapienBodyData *)sapien_data, (int *)index,
                                               (Vec3 *)offset, link_count, count);
}

void link_vel_physx_to_sapien(void *sapien_data, void *physx_vel, int count, cudaStream_t stream) {
  link_vel_physx_to_sapien_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                    stream>>>((SapienBodyData *)sapien_data,
                                              (PhysxVelocity *)physx_vel, count);
}

void root_vel_sapien_to_physx(void *physx_vel, void *sapien_data, void *index, int link_count,
                              int count, cudaStream_t stream) {
  root_vel_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                    stream>>>(
      (PhysxVelocity *)physx_vel, (SapienBodyData *)sapien_data, (int *)index, link_count, count);
}

} // namespace physx
} // namespace sapien
