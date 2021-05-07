#pragma once
#include <PxPhysicsAPI.h>
#include <memory>
#include <string>
#include <vector>

namespace sapien {

class SActorBase;
class SPhysicalMaterial;
struct SGeometry {
  virtual std::string getType() const = 0;
  virtual ~SGeometry() = default;
};

struct SBoxGeometry : public SGeometry {
  physx::PxVec3 halfLengths;
  inline virtual std::string getType() const { return "box"; };
};

struct SCapsuleGeometry : public SGeometry {
  physx::PxReal radius;
  physx::PxReal halfLength;
  inline virtual std::string getType() const { return "capsule"; };
};

struct SSphereGeometry : public SGeometry {
  physx::PxReal radius;
  inline virtual std::string getType() const { return "sphere"; };
};

struct SConvexMeshGeometry : public SGeometry {
  physx::PxVec3 scale;
  physx::PxQuat rotation;
  std::vector<physx::PxReal> vertices;
  std::vector<uint32_t> indices;
  inline virtual std::string getType() const { return "convex_mesh"; };
};

struct SNonconvexMeshGeometry : public SGeometry {
  physx::PxVec3 scale;
  physx::PxQuat rotation;
  std::vector<physx::PxReal> vertices;
  std::vector<uint32_t> indices;
  inline virtual std::string getType() const { return "nonconvex_mesh"; };
};

struct SPlaneGeometry : public SGeometry {
  inline virtual std::string getType() const { return "plane"; };
};

class SCollisionShape {

public:
  SCollisionShape(physx::PxShape *shape);

  inline physx::PxShape *getPxShape() const { return mPxShape; }

  /** called by SActorBase::AttachShape */
  void setActor(SActorBase *actor);

  SActorBase *getActor() const;

  void setCollisionGroups(uint32_t group0, uint32_t group1, uint32_t group2, uint32_t group3);
  std::array<uint32_t, 4> getCollisionGroups() const;

  void setRestOffset(physx::PxReal offset);
  physx::PxReal getRestOffset() const;

  void setContactOffset(physx::PxReal offset);
  physx::PxReal getContactOffset() const;

  void setTorsionalPatchRadius(physx::PxReal radius);
  physx::PxReal getTorsionalPatchRadius() const;

  void setMinTorsionalPatchRadius(physx::PxReal radius);
  physx::PxReal getMinTorsionalPatchRadius() const;

  physx::PxTransform getLocalPose() const;
  void setLocalPose(physx::PxTransform const &pose);

  void setIsTrigger(bool trigger);
  bool isTrigger() const;

  void setPhysicalMaterial(std::shared_ptr<SPhysicalMaterial> material);
  std::shared_ptr<SPhysicalMaterial> getPhysicalMaterial() const;

  std::string getType() const;
  std::unique_ptr<SGeometry> getGeometry() const;

  SCollisionShape(SCollisionShape const &) = delete;
  SCollisionShape(SCollisionShape &&) = default;
  SCollisionShape &operator=(SCollisionShape const &) = delete;
  SCollisionShape &operator=(SCollisionShape &&) = default;
  ~SCollisionShape();

private:
  physx::PxShape *mPxShape{};
  SActorBase *mActor{};
  std::shared_ptr<SPhysicalMaterial> mPhysicalMaterial{};
};

} // namespace sapien
