#pragma once
#include "event.h"

namespace sapien {

class SActorBase;

class EventActorPreDestroy : public Event {
public:
  SActorBase *actor;
};

class EventActorStep : public Event {
public:
  SActorBase *actor;
  float time;
};

} // namespace sapien
