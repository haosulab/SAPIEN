#pragma once
#include <PxPhysicsAPI.h>
#include <foundation/PxTransform.h>
#include <string>
#include <vector>
#include <array>
#include <functional>

struct JointGuiInfo {
  std::string name;
  std::array<float, 2> limits;
  float value;
};

struct ArticulationGuiInfo {
  std::string name = "";
  std::vector<JointGuiInfo> jointInfo;
};

struct LinkGuiInfo {
  std::string name = "";
  physx::PxTransform transform;
};

struct GuiInfo {
  ArticulationGuiInfo articulationInfo;
  LinkGuiInfo linkInfo;
};

class IRenderer {
public:
  /* This function is called when a rigid body is added to a scene */
  virtual void addRigidbody(uint32_t uniqueId, const std::string &meshFile,
                            const physx::PxVec3 &scale) = 0;
  virtual void addRigidbody(uint32_t uniqueId, physx::PxGeometryType::Enum type,
                            const physx::PxVec3 &scale) = 0;

  /* This function is called when a rigid body is removed from a scene */
  virtual void removeRigidbody(uint32_t uniqueId) = 0;

  /* This function is called when a rigid body is updated */
  virtual void updateRigidbody(uint32_t uniqueId, const physx::PxTransform &transform) = 0;

  virtual void bindQueryCallback(std::function<GuiInfo(uint32_t)>) = 0;
  virtual void bindSyncCallback(std::function<void(uint32_t, const GuiInfo &info)>) = 0;
};
