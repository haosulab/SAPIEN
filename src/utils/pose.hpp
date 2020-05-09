#pragma once
#include <PxPhysicsAPI.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace sapien::utils {
using namespace physx;
namespace py = pybind11;

PxTransform *fromTransFormationMatrix(const py::array_t<PxReal> &mat) {
  assert(mat.size() == 16 && mat.shape()[0] == 4);
  auto um = mat.unchecked<2>();
  float w = 0.5 * std::sqrt(1.0 + um(0, 0) + um(1, 1) + um(2, 2));
  float over_w = 0.25 / w;
  float x = (um(2, 1) - um(1, 2)) * over_w;
  float y = (um(0, 2) - um(2, 0)) * over_w;
  float z = (um(1, 0) - um(0, 1)) * over_w;
  return new PxTransform({um(0, 3), um(1, 3), um(2, 3)}, {x, y, z, w});
}

Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor> toTransformationMatrix(PxTransform &t) {
  t.q.normalize();
  Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor> mat44;
  Eigen::Quaternionf q(t.q.w, t.q.x, t.q.y, t.q.z);
  mat44.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  mat44.block<3, 1>(0, 3) = Eigen::Matrix<PxReal, 3, 1>(t.p.x, t.p.y, t.p.z);
  return mat44;
}

std::string poseRepresentation(const PxTransform &pose) {
  std::ostringstream oss;
  oss << "Pose([" << pose.p.x << ", " << pose.p.y << ", " << pose.p.z << "], [" << pose.q.w << ", "
      << pose.q.x << ", " << pose.q.y << ", " << pose.q.z << "])";
  return oss.str();
}
} // namespace sapien::utils
