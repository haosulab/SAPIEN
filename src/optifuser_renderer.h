#pragma once
#include "render_interface.h"
#include <memory>
#include <camera_spec.h>
#include <optifuser.h>

class OptifuserRenderer : public IRenderer {
public:
  std::map<uint32_t, std::vector<Optifuser::Object*>> mObjectRegistry;
  std::shared_ptr<Optifuser::Scene> mScene;
  Optifuser::GLFWRenderContext *mContext = nullptr;
  Optifuser::FPSCameraSpec cam;
  std::function<GuiInfo(uint32_t)> queryCallback = {};
  std::function<void(uint32_t, const GuiInfo &info)> syncCallback = {};

  std::vector<std::unique_ptr<Optifuser::OffscreenRenderContext>> mOffscreenContexts;

public:
  virtual void addRigidbody(uint32_t uniqueId, const std::string &meshFile,
                            const physx::PxVec3 &scale) override;
  virtual void addRigidbody(uint32_t uniqueId, physx::PxGeometryType::Enum type,
                            const physx::PxVec3 &scale) override;
  virtual void removeRigidbody(uint32_t uniqueId) override;
  virtual void updateRigidbody(uint32_t uniqueId, const physx::PxTransform &transform) override;

  virtual void bindQueryCallback(std::function<GuiInfo(uint32_t)>) override;
  virtual void bindSyncCallback(std::function<void(uint32_t, const GuiInfo &info)>) override;


public:
  void init();
  void destroy();
  void render();

  void addOffscreenContext(int width, int height);
  Optifuser::OffscreenRenderContext &getOffscreenContext(int i);
};
