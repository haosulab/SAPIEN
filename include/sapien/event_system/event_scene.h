#pragma once
#include "event.h"

namespace sapien {

struct EventSceneStep : public Event {
  class SScene* scene;
  float timeStep;
};
} // namespace sapien
