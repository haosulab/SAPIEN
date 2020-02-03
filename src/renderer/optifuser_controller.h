#pragma once
#include "camera_controller.h"
#include "event_system/events.h"
#include "optifuser_renderer.h"

namespace sapien {
class Simulation;
class SScene;
class SActorBase;

namespace Renderer {

struct JointGuiModel {
  std::string name;
  std::array<float, 2> limits;
  float value;
};

struct ArticulationGuiModel {
  std::string name = "";
  std::vector<JointGuiModel> jointModel;
};

struct LinkGuiModel {
  std::string name = "";
  physx::PxTransform transform = {{0, 0, 0}, physx::PxIdentity};
  physx::PxTransform cmassPose = {{0, 0, 0}, physx::PxIdentity};
  uint32_t col1 = 0;
  uint32_t col2 = 0;
  uint32_t col3 = 0;
  bool renderCollision = false;
  bool showCenterOfMass = false;
};

struct GuiModel {
  int linkId = 0;
  LinkGuiModel linkModel;
  int articulationId = 0;
  ArticulationGuiModel articulationModel;
};

class OptifuserController : public IEventListener<ActorPreDestroyEvent> {
  OptifuserRenderer *mRenderer = nullptr;
  SScene *mScene = nullptr;

  GuiModel mGuiModel = {};
  SActorBase *mCurrentSelection = nullptr;

  bool mShouldQuit = false;

  Optifuser::CameraSpec mCamera;
  SActorBase *mCurrentFocus = nullptr;

public:
  FPSCameraController mFreeCameraController;
  ArcRotateCameraController mArcCameraController;

  explicit OptifuserController(OptifuserRenderer *renderer);

  bool shouldQuit();

  void showWindow();
  void hideWindow();
  void setCurrentScene(SScene *scene);
  void focus(SActorBase *actor);
  void setCameraPosition(float x, float y, float z);
  void setCameraRotation(float yaw, float pitch);
  physx::PxTransform getCameraPose() const;

  void render();

  void onEvent(ActorPreDestroyEvent &e) override;
};
} // namespace Renderer
} // namespace sapien
