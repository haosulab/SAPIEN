#include "sapien/profiler.h"
#include <easy/profiler.h>

namespace sapien {

void AddProfilerEvent(char const *name) {
  if (!profiler::isListening()) {
    profiler::startListen();
  }
  EASY_EVENT(name);
}
void StartProfilerBlock(char const *name) { EASY_NONSCOPED_BLOCK(name); }
void EndProfilerBlock() { EASY_END_BLOCK; }

} // namespace sapien
