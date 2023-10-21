#pragma once

#include "pose.h"
#include <Eigen/Dense>

namespace sapien {

struct AABB;
struct OBB;

struct AABB {
  Vec3 lower{0.f};
  Vec3 upper{0.f};

  inline OBB getOBB() const;
  inline Vec3 getCenter() const { return (lower + upper) * 0.5f; }

  AABB operator+(AABB const &other) {
    return {{std::min(other.lower.x, lower.x), std::min(other.lower.y, lower.y),
             std::min(other.lower.z, lower.z)},
            {std::max(other.upper.x, upper.x), std::max(other.upper.y, upper.y),
             std::max(other.upper.z, upper.z)}};
  }
};

struct OBB {
  Vec3 halfExtents;
  Pose pose;

  inline AABB getAABB() const;
  inline Vec3 getCenter() const { return pose.p; }
};

inline AABB computeAABB(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
                        Vec3 const &scale, Pose const &pose) {
  auto R =
      Eigen::Quaternionf(pose.q.w, pose.q.x, pose.q.y, pose.q.z).normalized().toRotationMatrix();
  auto points = vertices * R.transpose();

  auto lower = points.colwise().minCoeff();
  auto upper = points.colwise().maxCoeff();
  return {.lower = Vec3(lower(0), lower(1), lower(2)) * scale + pose.p,
          .upper = Vec3(upper(0), upper(1), upper(2)) * scale + pose.p};
}

inline AABB computeAABB(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
                        Pose const &pose) {
  return computeAABB(vertices, Vec3(1.f), pose);
}

inline AABB computeAABB(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices) {
  return computeAABB(vertices, Vec3(1.f), Pose());
}

inline AABB getTransformedAABB(AABB const &b, Pose const &transform) {
  auto obb = b.getOBB();
  obb.pose = transform * obb.pose;
  return obb.getAABB();
}

OBB AABB::getOBB() const {
  return {.halfExtents = (upper - lower) * 0.5, .pose = Pose((lower + upper) * 0.5)};
}

AABB OBB::getAABB() const {
  Eigen::Matrix<float, 8, 3, Eigen::RowMajor> points;
  points << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1;
  points = points * Eigen::Vector3f(halfExtents.x, halfExtents.y, halfExtents.z).asDiagonal();
  return computeAABB(points, pose);
}

} // namespace sapien
