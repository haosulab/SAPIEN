#pragma once
#include <PxPhysicsAPI.h>
#include <memory>
#include <string>
#include <vector>

namespace sapien {

struct SGeometry {};

struct SBoxGeometry : public SGeometry {
  physx::PxVec3 halfLengths;
};

struct SCapsuleGeometry : public SGeometry {
  physx::PxReal radius;
  physx::PxReal halfLength;
};

struct SSphereGeometry : public SGeometry {
  physx::PxReal radius;
};

struct SConvexMeshGeometry : public SGeometry {
  physx::PxVec3 scale;
  physx::PxQuat rotation;
  std::vector<physx::PxReal> vertices;
  std::vector<uint32_t> indices;
};

struct SPlaneGeometry : public SGeometry {};

struct SShape {
  std::string type = "invalid";
  physx::PxTransform pose = physx::PxTransform(physx::PxIdentity);
  std::unique_ptr<SGeometry> geometry = nullptr;

  inline SShape() {};
  SShape(SShape const &) = delete;
  SShape(SShape &&) = default;
  SShape &operator=(SShape const &) = delete;
  SShape &operator=(SShape &&) = default;
  ~SShape() = default;
};

} // namespace sapien
