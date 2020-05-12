#pragma once
#include "camera_controller.h"
#include "event_system/event_system.h"
#include "optifuser_renderer.h"

namespace sapien {
class Simulation;
class SScene;
class SActorBase;

namespace Renderer {

class OptifuserController : public IEventListener<EventActorPreDestroy> {
  OptifuserRenderer *mRenderer = nullptr;
  SScene *mScene = nullptr;

  // GuiModel mGuiModel = {};
  // int mLinkId = 0;
  // int mArticulationId = 0;
  bool showCM = false;

  SActorBase *mCurrentSelection = nullptr;

  bool mShouldQuit = false;

  int mCameraMode;
  std::unique_ptr<Optifuser::CameraSpec> mCamera;
  SActorBase *mCurrentFocus = nullptr;
  bool paused = false;
  bool flipX = false;
  bool flipY = false;
  bool transparentSelection = false;
  bool gizmo = false;
  glm::mat4 gizmoTransform = glm::mat4(1);

  void editTransform();

  std::vector<IPxrRigidbody *> gizmoBody;
  void createGizmoVisual(SActorBase *actor);

public:
  FPSCameraController mFreeCameraController;
  ArcRotateCameraController mArcCameraController;

  explicit OptifuserController(OptifuserRenderer *renderer);

  bool shouldQuit();

  void showWindow();
  void hideWindow();
  void setCurrentScene(SScene *scene);
  void focus(SActorBase *actor);
  void select(SActorBase *actor);
  void setCameraPosition(float x, float y, float z);
  void setCameraRotation(float yaw, float pitch);
  void setCameraOrthographic(bool ortho = true);
  physx::PxTransform getCameraPose() const;

  void render();

  void onEvent(EventActorPreDestroy &e) override;

  inline SActorBase *getSelectedActor() const { return mCurrentSelection; }
};
} // namespace Renderer
} // namespace sapien
