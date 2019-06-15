#pragma once
#include <PxPhysicsAPI.h>
#include <foundation/PxTransform.h>
#include <string>

class IRenderer {
 public:
  /* This function is called when a rigid body is added to a scene */
  virtual void addRigidbody(uint64_t uniqueId, const std::string& meshFile) = 0;
  virtual void addRigidbody(uint64_t uniqueId, physx::PxGeometryType::Enum type, physx::PxVec3 scale) = 0;

  /* This function is called when a rigid body is removed from a scene */
  virtual void removeRigidbody(uint64_t uniqueId) = 0;

  /* This function is called when a rigid body is updated */
  virtual void updateRigidbody(uint64_t uniqueId, const physx::PxTransform& transform) = 0;
};

