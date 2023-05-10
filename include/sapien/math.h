#pragma once

#include <Eigen/Dense>
#include <PxPhysicsAPI.h>

namespace sapien {

using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
using Mat3 = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;
using Vec3 = Eigen::Vector3f;
using Vec4 = Eigen::Vector4f;
using Quat = Eigen::Quaternionf;

inline physx::PxTransform fromTransFormationMatrix(const Mat4 &mat) {
  Quat q = Quat(mat.block<3, 3>(0, 0)).normalized();
  Vec3 p = mat.block<3, 1>(0, 3);
  return physx::PxTransform({p.x(), p.y(), p.z()}, {q.x(), q.y(), q.z(), q.w()});
}

inline Mat4 toTransformationMatrix(physx::PxTransform const &t) {
  Quat q(t.q.w, t.q.x, t.q.y, t.q.z);
  Mat4 mat = Mat4::Identity();
  mat.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  mat.block<3, 1>(0, 3) = Vec3(t.p.x, t.p.y, t.p.z);
  return mat;
}

inline std::string poseRepr(const physx::PxTransform &pose) {
  std::ostringstream oss;
  oss << "Pose([" << pose.p.x << ", " << pose.p.y << ", " << pose.p.z << "], [" << pose.q.w << ", "
      << pose.q.x << ", " << pose.q.y << ", " << pose.q.z << "])";
  return oss.str();
}

inline Mat4 ros2opencv() {
  Mat4 mat;
  mat << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  return mat;
}

}; // namespace sapien
