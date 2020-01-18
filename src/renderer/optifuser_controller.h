#pragma once
#include "optifuser_renderer.h"

namespace sapien {
class Simulation;
class SScene;

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
  physx::PxTransform transform;
  uint32_t col1 = 0;
  uint32_t col2 = 0;
  uint32_t col3 = 0;
  bool renderCollision = false;
};

struct GuiModel {
  int linkId = 0;
  LinkGuiModel linkModel;
  int articulationId = 0;
  ArticulationGuiModel articulationModel;
};

class OptifuserController {
  OptifuserRenderer *mRenderer = nullptr;
  SScene *mScene = nullptr;

  GuiModel mGuiModel = {};

  bool mShouldQuit = false;

public:
  Optifuser::FPSCameraSpec mCamera;
  explicit OptifuserController(OptifuserRenderer *renderer);

  bool shouldQuit();

  void showWindow();
  void hideWindow();
  void setCurrentScene(SScene *scene);
  void render();
};
} // namespace Renderer
} // namespace sapien
