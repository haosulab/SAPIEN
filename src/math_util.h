#pragma once

#include <PxPhysicsAPI.h>
#include <eigen3/Eigen/Eigenvalues>

std::tuple<physx::PxVec3, physx::PxTransform> decomposeInertia(const physx::PxVec3 &inOrigin,
                                                               const physx::PxMat33 inInertia);
