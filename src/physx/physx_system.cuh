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
#include <PxContact.h>
#include <geomutils/PxContactPoint.h>

struct CUstream_st;

namespace sapien {
namespace physx {

struct ActorPair {
  ::physx::PxActor *actor0;
  ::physx::PxActor *actor1;
};

CUDA_CALLABLE inline ActorPair makeActorPair(::physx::PxActor *a0, ::physx::PxActor *a1,
                                             int &order) {
  if (a0 < a1) {
    order = 1;
    return {a0, a1};
  } else {
    order = -1;
    return {a1, a0};
  }
}

struct ActorPairQuery {
  ActorPair pair;
  uint32_t id;
  int order;
};

struct ActorQuery {
  ::physx::PxActor *actor;
  uint32_t id;
};

CUDA_CALLABLE inline bool operator==(ActorPair const &a, ActorPair const &b) {
  return a.actor0 == b.actor0 && a.actor1 == b.actor1;
}

CUDA_CALLABLE inline bool operator<(ActorPair const &a, ActorPair const &b) {
  return a.actor0 < b.actor0 || (a.actor0 == b.actor0 && a.actor1 < b.actor1);
}

struct PhysxQuat {
  float x, y, z, w;
};

struct PhysxPose {
  PhysxQuat q{};
  Vec3 p{};
};
static_assert(sizeof(PhysxPose) == 28);

struct SapienWrench {
  Vec3 f;
  Vec3 t;
};
static_assert(sizeof(SapienWrench) == 24);

struct SapienBodyData {
  Vec3 p;
  Quat q;
  Vec3 v{};
  Vec3 w{};
};
static_assert(sizeof(SapienBodyData) == 52);

void body_data_physx_to_sapien(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int count, CUstream_st *);
void body_data_sapien_to_physx(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int count, CUstream_st *);

void link_data_physx_to_sapien(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int max_links, int link_count, CUstream_st *);
void root_data_sapien_to_physx(SapienBodyData *s_data, PhysxPose *p_pose, Vec3 *v, Vec3 *w,
                               Vec3 *offset, int max_links, int articulation_count,
                               CUstream_st *);

void body_wrench_physx_to_sapien(SapienWrench *s_wrench, Vec3 *f, Vec3 *t, int count,
                                 CUstream_st *);
void body_wrench_sapien_to_physx(SapienWrench *s_wrench, Vec3 *f, Vec3 *t, int count,
                                 CUstream_st *);

// fill out_forces with net contact forces per body pair. query stores sorted pairs of
// interested actors and their indices corresponding to the out_forces array
void handle_contacts(::physx::PxGpuContactPair *contacts, int contact_count, ActorPairQuery *query,
                     int query_count, Vec3 *out_forces, cudaStream_t stream);

// fill out_forces with net contact forces per body. query stores sorted actors
// and their indices corresponding to the out_forces array
void handle_net_contact_force(::physx::PxGpuContactPair *contacts, int contact_count,
                              ActorQuery *query, int query_count, Vec3 *out_forces,
                              cudaStream_t stream);

} // namespace physx
} // namespace sapien