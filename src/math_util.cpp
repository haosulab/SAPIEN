#include "math_util.h"

std::tuple<physx::PxVec3, physx::PxTransform> decomposeInertia(const physx::PxVec3 &inOrigin,
                                                               const physx::PxMat33 inInertia) {
  Eigen::Matrix3f inertia;
  inertia(0, 0) = inInertia(0, 0);
  inertia(1, 1) = inInertia(1, 1);
  inertia(2, 2) = inInertia(2, 2);
  inertia(1, 0) = inertia(0, 1) = inInertia(0, 1);
  inertia(2, 0) = inertia(0, 2) = inInertia(0, 2);
  inertia(2, 1) = inertia(1, 2) = inInertia(1, 2);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
  es.compute(inertia);
  auto eigs = es.eigenvalues();
  auto vecs = es.eigenvectors();

  auto col0 = es.eigenvectors().col(0);
  auto col1 = es.eigenvectors().col(1);
  auto col2 = es.eigenvectors().col(2);

  physx::PxMat33 rot;
  rot.column0 = {col0[0], col0[1], col0[2]};
  rot.column1 = {col1[0], col1[1], col1[2]};
  rot.column2 = {col2[0], col2[1], col2[2]};

  physx::PxTransform outTransform = physx::PxTransform(inOrigin, physx::PxQuat(rot));
  physx::PxVec3 outInertia = {eigs[0], eigs[1], eigs[2]};

  return {outInertia, outTransform};
}
