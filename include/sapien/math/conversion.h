#pragma once
#include "pose.h"
#include <Eigen/Dense>
#include <PxPhysicsAPI.h>
#include <numbers>

namespace sapien {

inline Vec3 PxVec3ToVec3(::physx::PxVec3 const &v) { return {v.x, v.y, v.z}; }
inline Quat PxQuatToQuat(::physx::PxQuat const &q) { return {q.w, q.x, q.y, q.z}; }

inline ::physx::PxVec3 Vec3ToPxVec3(Vec3 const &v) { return {v.x, v.y, v.z}; }
inline ::physx::PxQuat QuatToPxQuat(Quat const &q) { return {q.x, q.y, q.z, q.w}; }

inline Pose PxTransformToPose(::physx::PxTransform const &transform) {
  return {PxVec3ToVec3(transform.p), PxQuatToQuat(transform.q)};
}
inline ::physx::PxTransform PoseToPxTransform(Pose const &pose) {
  return {Vec3ToPxVec3(pose.p), QuatToPxQuat(pose.q)};
}

inline Pose EigenMat4ToPose(Eigen::Matrix<float, 4, 4, Eigen::RowMajor> const &mat) {
  Eigen::Quaternionf q = Eigen::Quaternionf(mat.block<3, 3>(0, 0)).normalized();
  Eigen::Vector3f p = mat.block<3, 1>(0, 3);
  return {{p.x(), p.y(), p.z()}, {q.w(), q.x(), q.y(), q.z()}};
}

inline Eigen::Matrix<float, 4, 4, Eigen::RowMajor> PoseToEigenMat4(Pose const &pose) {
  Eigen::Quaternionf q(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> mat =
      Eigen::Matrix<float, 4, 4, Eigen::RowMajor>::Identity();
  mat.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  mat.block<3, 1>(0, 3) = Eigen::Vector3f(pose.p.x, pose.p.y, pose.p.z);
  return mat;
}

inline Vec3 QuatToRPY(Quat const &quat) {
  Eigen::Quaternionf q(quat.w, quat.x, quat.y, quat.z);
  auto rpy = q.toRotationMatrix().eulerAngles(2, 1, 0);
  return Vec3(rpy[2], rpy[1], rpy[0]);
}

inline Quat RPYToQuat(Vec3 const &rpy) {
  return Quat(Vec3(0, 0, 1), rpy.z) * Quat(Vec3(0, 1, 0), rpy.y) * Quat(Vec3(1, 0, 0), rpy.x);
}

inline Quat ShortestRotation(Vec3 const &v0_, Vec3 const &v1_) {
  Vec3 v0 = v0_.getNormalized();
  Vec3 v1 = v1_.getNormalized();

  float d = v0.dot(v1);

  if (d > 0.999999) {
    return Quat{1.f, 0.f, 0.f, 0.f};
  }

  if (d < -0.999999) {
    Vec3 tmp = Vec3(1.f, 0.f, 0.f).cross(v0_);
    if (tmp.length() < 0.000001f) {
      tmp = Vec3(0.f, 1.f, 0.f).cross(v0_);
    }
    tmp.normalize();
    return Quat(tmp, std::numbers::pi_v<float>);
  }

  Vec3 c = v0.cross(v1);
  Quat q = Quat(1.f + d, c.x, c.y, c.z);
  q.normalize();
  return q;
}

inline Eigen::Matrix<float, 4, 4, Eigen::RowMajor> ros2opencv() {
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> mat;
  mat << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  return mat;
}

inline std::string poseRepr(Pose const &pose) {
  std::ostringstream oss;
  oss << "Pose([" << pose.p.x << ", " << pose.p.y << ", " << pose.p.z << "], [" << pose.q.w << ", "
      << pose.q.x << ", " << pose.q.y << ", " << pose.q.z << "])";
  return oss.str();
}

} // namespace sapien
