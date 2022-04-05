#pragma once

#include "simulation.h"
#include <easy/profiler.h>

namespace sapien {
inline void AddProfilerEvent(char const *name) {
  if (!profiler::isListening()) {
    profiler::startListen();
  }
  EASY_EVENT(name);
}
}; // namespace sapien
