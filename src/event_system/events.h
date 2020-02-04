#pragma once
#include <algorithm>
#include <spdlog/spdlog.h>
#include <vector>

namespace sapien {
class SActorBase;

struct Event {};

struct ActorPreDestroyEvent : public Event {
  SActorBase *actor;
};

struct ArticulationPreDestroyEvent : public Event {};

template <typename T> class IEventListener {
public:
  virtual void onEvent(T &event) = 0;
};


}; // namespace sapien
