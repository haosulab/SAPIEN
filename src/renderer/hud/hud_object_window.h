#pragma once
#include <imgui.h>
#include "id_generator.h"

namespace sapien {
class SScene;
class SActorBase;
class SArticulationBase;

namespace Renderer {

struct HudActor {
  bool mShowCenterOfMass;
  void draw(SActorBase *actor);
};

struct HudArticulation {
  bool mSelect = false;
  physx_id_t mSelectedId;
  void draw(SArticulationBase *articulation, SActorBase *actor);
};

struct HudWorld {
  bool mSelect = false;
  physx_id_t mSelectedId;

  void draw(SScene *scene, physx_id_t selectedId); 
};

struct HudObjectWindow {
  HudActor mHudActor {};
  HudArticulation mHudArticulation {};
  HudWorld mHudWorld {};

  void draw(SScene *scene, physx_id_t selectedId);
};

}
} // namespace sapien
