#pragma once
#include "event.h"

namespace sapien {

class SActorBase;

struct EventActorPreDestroy : public Event {
  SActorBase *actor;
};

struct EventActorStep : public Event {
  SActorBase *actor;
  float time;
};

struct EventActorContact : public Event {
public:
  SActorBase *self;
  SActorBase *other;
  struct SContact const *contact;
};

struct EventActorTrigger : public Event {
public:
  SActorBase *triggerActor;
  SActorBase *otherActor;
  struct STrigger const *trigger;
};

} // namespace sapien
