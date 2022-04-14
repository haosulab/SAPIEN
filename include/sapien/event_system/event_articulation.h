#pragma once
#include "event.h"

namespace sapien {

class SArticulationBase;

struct EventArticulationPreDestroy : public Event {
  SArticulationBase *articulation;
};

struct EventArticulationStep : public Event {
  SArticulationBase *articulation;
  float time;
};

} // namespace sapien
