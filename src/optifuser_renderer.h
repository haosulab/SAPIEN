#pragma once
#include "render_interface.h"
#include <optifuser.h>
#include <memory>

class OptifuserRenderer : public IRenderer {
  std::map<uint64_t, std::vector<std::shared_ptr<Optifuser::Object>>> mObjectRegistry;
  std::shared_ptr<Optifuser::Scene> mScene;
  Optifuser::GLFWRenderContext *mContext = nullptr;

 public :
  virtual void addRigidbody(uint64_t uniqueId, const std::string &meshFile) override;
  virtual void removeRigidbody(uint64_t uniqueId) override;
  virtual void updateRigidbody(uint64_t uniqueId, const physx::PxTransform &transform) override;

 public:
  void init(); 
  void destroy();
  void render();
};
