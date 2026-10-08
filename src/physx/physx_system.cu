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

__global__ void body_data_physx_to_sapien_kernel(SapienBodyData *__restrict__ s_data,
                                                 PhysxPose *__restrict__ p_pose, Vec3 *__restrict__ v,
                                                 Vec3 *__restrict__ w, Vec3 *__restrict__ offset,
                                                 int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  s_data[g] = {
      p_pose[g].p - offset[g],
      Quat(p_pose[g].q.w, p_pose[g].q.x, p_pose[g].q.y, p_pose[g].q.z),
      v[g],
      w[g],
  };
}

__global__ void body_data_sapien_to_physx_kernel(SapienBodyData *__restrict__ s_data,
                                                 PhysxPose *__restrict__ p_pose, Vec3 *__restrict__ v,
                                                 Vec3 *__restrict__ w, Vec3 *__restrict__ offset,
                                                 int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  p_pose[g].p = s_data[g].p + offset[g];
  p_pose[g].q = {s_data[g].q.x, s_data[g].q.y, s_data[g].q.z, s_data[g].q.w};
  v[g] = s_data[g].v;
  w[g] = s_data[g].w;
}

__global__ void link_data_physx_to_sapien_kernel(SapienBodyData *__restrict__ s_data,
                                                 PhysxPose *__restrict__ p_pose, Vec3 *__restrict__ v,
                                                 Vec3 *__restrict__ w, Vec3 *__restrict__ offset,
                                                 int max_links, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  int ai = g / max_links;

  s_data[g].p = p_pose[g].p - offset[ai];
  s_data[g].q = Quat(p_pose[g].q.w, p_pose[g].q.x, p_pose[g].q.y, p_pose[g].q.z);
  s_data[g].v = v[g];
  s_data[g].w = w[g];
}

__global__ void root_data_sapien_to_physx_kernel(SapienBodyData *__restrict__ s_data,
                                                 PhysxPose *__restrict__ p_pose, Vec3 *__restrict__ v,
                                                 Vec3 *__restrict__ w, Vec3 *__restrict__ offset,
                                                 int max_links, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  SapienBodyData sd = s_data[g * max_links];

  p_pose[g] = {{sd.q.x, sd.q.y, sd.q.z, sd.q.w}, sd.p + offset[g]};
  v[g] = sd.v;
  w[g] = sd.w;
}

__global__ void body_wrench_physx_to_sapien_kernel(SapienWrench *__restrict__ s_wrench,
                                                    Vec3 *__restrict__ f, Vec3 *__restrict__ t,
                                                    int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }
  s_wrench[g].f = f[g];
  s_wrench[g].t = t[g];
}

__global__ void body_wrench_sapien_to_physx_kernel(SapienWrench *__restrict__ s_wrench,
                                                   Vec3 *__restrict__ f, Vec3 *__restrict__ t,
                                                   int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }
  f[g] = s_wrench[g].f;
  t[g] = s_wrench[g].t;
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

void body_data_physx_to_sapien(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int count, CUstream_st *stream) {
  body_data_physx_to_sapien_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      s_data, p_pose, v, w, offset, count);
}
void body_data_sapien_to_physx(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int count, CUstream_st *stream) {
  body_data_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>(s_data, p_pose, v, w, offset, count);
}

void link_data_physx_to_sapien(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int max_links, int count, CUstream_st *stream) {
  link_data_physx_to_sapien_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>(s_data, p_pose, v, w, offset, max_links, count);
}
void root_data_sapien_to_physx(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int max_links, int count, CUstream_st *stream) {
  root_data_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                     stream>>>(s_data, p_pose, v, w, offset, max_links, count);
}

void body_wrench_physx_to_sapien(SapienWrench *s_wrench, Vec3 *f, Vec3 *t, int count,
                                 CUstream_st *stream) {
  body_wrench_physx_to_sapien_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                        stream>>>(s_wrench, f, t, count);
}
void body_wrench_sapien_to_physx(SapienWrench *s_wrench, Vec3 *f, Vec3 *t, int count,
                                 CUstream_st *stream) {
  body_wrench_sapien_to_physx_kernel<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                        stream>>>(s_wrench, f, t, count);
}

void handle_contacts(::physx::PxGpuContactPair *contacts, int contact_count, ActorPairQuery *query,
                     int query_count, Vec3 *out_forces, cudaStream_t stream) {
  if (contact_count == 0) {
    return;
  }
  handle_contacts_kernel<<<(contact_count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0, stream>>>(
      contacts, contact_count, query, query_count, out_forces);
}

void handle_net_contact_force(::physx::PxGpuContactPair *contacts, int contact_count,
                              ActorQuery *query, int query_count, Vec3 *out_forces,
                              cudaStream_t stream) {
  if (contact_count == 0) {
    return;
  }
  handle_net_contact_force_kernel<<<(contact_count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                    stream>>>(contacts, contact_count, query, query_count,
                                             out_forces);
}

} // namespace physx
} // namespace sapien