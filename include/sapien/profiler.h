#pragma once

#include "simulation.h"

namespace sapien {
void AddProfilerEvent(char const *name);
void StartProfilerBlock(char const *name);
void EndProfilerBlock();

}; // namespace sapien
