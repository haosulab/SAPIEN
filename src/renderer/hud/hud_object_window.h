#pragma once
#include <imgui.h>
#include "id_generator.h"

namespace sapien {
class SScene;
class SActorBase;

namespace Renderer {

struct HudActor {
  void draw(SActorBase *actor);
};

struct HudWorld {
  bool mSelect = false;
  physx_id_t mSelectedId;

  void draw(SScene *scene, physx_id_t selectedId); 
};

struct HudObjectWindow {
  HudActor mHudActor = {};
  HudWorld mHudWorld = {};

  void draw(SScene *scene, physx_id_t selectedId);
};

}
} // namespace sapien
