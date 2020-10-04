#pragma once
#include "event.h"

namespace sapien {

class SArticulationBase;

class EventArticulationPreDestroy : public Event {
public:
  SArticulationBase *articulation;
};

class EventArticulationStep : public Event {
public:
  SArticulationBase *articulation;
  float time;
};

} // namespace sapien
