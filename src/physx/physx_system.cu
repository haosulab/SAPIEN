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
#include "./physx_system.cuh"

#include <cstdio>

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
      physx_data[g].pose.p - offset[g],
      Quat(physx_data[g].pose.q.w, physx_data[g].pose.q.x, physx_data[g].pose.q.y,
           physx_data[g].pose.q.z),
      physx_data[g].v,
      physx_data[g].w,
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

  physx_pose[ai] = {{sd.q.x, sd.q.y, sd.q.z, sd.q.w}, sd.p + offset[ai]};
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

__device__ int binary_search(ActorPairQuery const *__restrict__ arr, int count, ActorPair x) {
  int low = 0;
  int high = count - 1;
  while (low <= high) {
    int mid = low + (high - low) / 2;
    if (arr[mid].pair == x)
      return mid;
    if (arr[mid].pair < x)
      low = mid + 1;
    else
      high = mid - 1;
  }
  return -1;
}

__device__ int binary_search(ActorQuery const *__restrict__ arr, int count, ::physx::PxActor *x) {
  int low = 0;
  int high = count - 1;
  while (low <= high) {
    int mid = low + (high - low) / 2;
    if (arr[mid].actor == x)
      return mid;
    if (arr[mid].actor < x)
      low = mid + 1;
    else
      high = mid - 1;
  }
  return -1;
}

__global__ void handle_contacts_kernel(::physx::PxGpuContactPair *__restrict__ contacts,
                                       int contact_count, ActorPairQuery *__restrict__ query,
                                       int query_count, Vec3 *__restrict__ out_forces) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= contact_count) {
    return;
  }

  int order = 0;
  ActorPair pair = makeActorPair(contacts[g].actor0, contacts[g].actor1, order);

  int index = binary_search(query, query_count, pair);
  if (index < 0) {
    return;
  }
  uint32_t id = query[index].id;

  order *= query[index].order;

  ::physx::PxContactPatch *patches = (::physx::PxContactPatch *)contacts[g].contactPatches;
  ::physx::PxContact *points = (::physx::PxContact *)contacts[g].contactPoints;

  float *forces = contacts[g].contactForces;

  Vec3 force = Vec3(0.f);
  for (int pi = 0; pi < contacts[g].nbPatches; ++pi) {
    Vec3 normal(patches[pi].normal.x, patches[pi].normal.y, patches[pi].normal.z);
    for (int i = 0; i < patches[pi].nbContacts; ++i) {
      int ci = patches[pi].startContactIndex + i;
      float f = forces[ci];
      force += normal * (f * order);
      // printf("normal = %f %f %f, normal length2 = %f, separation = %f, force = %f\n", normal.x,
      //        normal.y, normal.z, normal.dot(normal), points[ci].separation, f);
    }
  }
  atomicAdd(&out_forces[id].x, force.x);
  atomicAdd(&out_forces[id].y, force.y);
  atomicAdd(&out_forces[id].z, force.z);
}

__global__ void handle_net_contact_force_kernel(::physx::PxGpuContactPair *__restrict__ contacts,
                                                int contact_count, ActorQuery *__restrict__ query,
                                                int query_count, Vec3 *__restrict__ out_forces) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= contact_count) {
    return;
  }

  ::physx::PxActor *actor0 = contacts[g].actor0;
  ::physx::PxActor *actor1 = contacts[g].actor1;

  int index0 = binary_search(query, query_count, actor0);
  int index1 = binary_search(query, query_count, actor1);

  if (index0 < 0 && index1 < 0) {
    return;
  }

  ::physx::PxContactPatch *patches = (::physx::PxContactPatch *)contacts[g].contactPatches;
  ::physx::PxContact *points = (::physx::PxContact *)contacts[g].contactPoints;

  float *forces = contacts[g].contactForces;

  Vec3 force = Vec3(0.f);
  for (int pi = 0; pi < contacts[g].nbPatches; ++pi) {
    Vec3 normal(patches[pi].normal.x, patches[pi].normal.y, patches[pi].normal.z);
    for (int i = 0; i < patches[pi].nbContacts; ++i) {
      int ci = patches[pi].startContactIndex + i;
      float f = forces[ci];
      force += normal * f;
    }
  }

  if (index0 >= 0) {
    int id = query[index0].id;
    atomicAdd(&out_forces[id].x, force.x);
    atomicAdd(&out_forces[id].y, force.y);
    atomicAdd(&out_forces[id].z, force.z);
  }
  if (index1 >= 0) {
    int id = query[index1].id;
    atomicAdd(&out_forces[id].x, -force.x);
    atomicAdd(&out_forces[id].y, -force.y);
    atomicAdd(&out_forces[id].z, -force.z);
  }
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

void handle_contacts(::physx::PxGpuContactPair *contacts, int contact_count, ActorPairQuery *query,
                     int query_count, Vec3 *out_forces, cudaStream_t stream) {
  handle_contacts_kernel<<<(contact_count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      contacts, contact_count, query, query_count, out_forces);
}

void handle_net_contact_force(::physx::PxGpuContactPair *contacts, int contact_count,
                              ActorQuery *query, int query_count, Vec3 *out_forces,
                              cudaStream_t stream) {
  handle_net_contact_force_kernel<<<(contact_count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                    stream>>>(contacts, contact_count, query, query_count,
                                              out_forces);
}

} // namespace physx
} // namespace sapien