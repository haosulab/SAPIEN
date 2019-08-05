#pragma once
#include "render_interface.h"
#include <memory>
#include <camera_spec.h>
#include <optifuser.h>

class OptifuserRenderer : public IRenderer {
public:
  std::map<uint64_t, std::vector<Optifuser::Object*>> mObjectRegistry;
  std::shared_ptr<Optifuser::Scene> mScene;
  Optifuser::GLFWRenderContext *mContext = nullptr;
  Optifuser::FPSCameraSpec cam;

public:
  virtual void addRigidbody(uint64_t uniqueId, const std::string &meshFile,
                            const physx::PxVec3 &scale) override;
  virtual void addRigidbody(uint64_t uniqueId, physx::PxGeometryType::Enum type,
                            const physx::PxVec3 &scale) override;
  virtual void removeRigidbody(uint64_t uniqueId) override;
  virtual void updateRigidbody(uint64_t uniqueId, const physx::PxTransform &transform) override;

public:
  void init();
  void destroy();
  void render();
};
