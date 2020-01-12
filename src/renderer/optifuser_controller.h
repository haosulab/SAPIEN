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
};

struct GuiModel {
  int linkId;
  LinkGuiModel linkModel;
  int articulationId;
  ArticulationGuiModel articulationModel;
};

class OptifuserController {
  OptifuserRenderer *mRenderer = nullptr;
  SScene *mScene = nullptr;

  GuiModel mGuiModel = {};

public:
  Optifuser::FPSCameraSpec mCamera;
  explicit OptifuserController(OptifuserRenderer *renderer);

  /* called by the renderer */
  // void selectObject(int linkId);

  void showWindow();
  void hideWindow();
  void setCurrentScene(SScene *scene);
  void render();
};
} // namespace Renderer
} // namespace sapien
