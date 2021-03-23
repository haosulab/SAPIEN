#pragma once
#include <PxPhysicsAPI.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace sapien::utils {
using namespace physx;
namespace py = pybind11;

/**
 * Note that the conversion from quaternion to rotation matrix requires additional attention.
 * Thus, here we directly use Eigen implementation.
 * https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Geometry/Quaternion.h
 */
PxTransform *fromTransFormationMatrix(const py::array_t<PxReal> &mat) {
  assert(mat.size() == 16 && mat.shape()[0] == 4);
  auto um = mat.unchecked<2>();
  Eigen::Matrix3f rotMat;  // TODO(jigu): Maybe there exists an easier way to write it
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
        rotMat(i, j) = um(i, j);
    }
  }
  Eigen::Quaternionf q(rotMat);
  return new PxTransform({um(0, 3), um(1, 3), um(2, 3)}, {q.x(), q.y(), q.z(), q.w()});
}

Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor> toTransformationMatrix(PxTransform &t) {
  t.q.normalize();
  Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor> mat44 = Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor>::Identity(4,4);
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
