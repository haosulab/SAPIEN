#pragma once
#include "event.h"

namespace sapien {

class EventStep : public Event {
public:
  float timeStep;
};
} // namespace sapien
