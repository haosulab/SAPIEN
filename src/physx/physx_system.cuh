#include "sapien/math/pose.h"

struct CUstream_st;

namespace sapien {
namespace physx {

struct PhysxQuat {
  float x, y, z, w;
};

struct PhysxPose {
  PhysxQuat q{};
  Vec3 p{};
};
static_assert(sizeof(PhysxPose) == 28);

struct PhysxVelocity {
  Vec3 v{};
  Vec3 w{};
};
static_assert(sizeof(PhysxVelocity) == 24);

struct PhysxBodyData {
  PhysxPose pose{};
  float padding0;
  Vec3 v{};
  float padding1;
  Vec3 w{};
  float padding2;
};
static_assert(sizeof(PhysxBodyData) == 64);

struct SapienBodyData {
  Pose pose{};
  Vec3 v{};
  Vec3 w{};
};
static_assert(sizeof(SapienBodyData) == 52);

void body_data_physx_to_sapien(void *sapien_data, void *physx_data, void *offset, int count,
                               CUstream_st *);
void link_pose_physx_to_sapien(void *sapien_data, void *physx_pose, void *offset, int link_count,
                               int count, CUstream_st *);
void link_vel_physx_to_sapien(void *sapien_data, void *physx_vel, int count, CUstream_st *);

void body_data_sapien_to_physx(void *physx_data, void *sapien_data, void *offset, int count,
                               CUstream_st *);
void body_data_sapien_to_physx(void *physx_data, void *physx_index, void *sapien_data,
                               void *sapien_index, void *apply_index, void *offset, int count,
                               CUstream_st *);

void root_pose_sapien_to_physx(void *physx_pose, void *sapien_data, void *index, void *offset,
                               int link_count, int count, CUstream_st *);
void root_vel_sapien_to_physx(void *physx_vel, void *sapien_data, void *index, int link_count,
                              int count, CUstream_st *);

} // namespace physx
} // namespace sapien
