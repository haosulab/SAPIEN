#pragma once

namespace sapien {
void AddProfilerEvent(char const *name);
void StartProfilerBlock(char const *name);
void EndProfilerBlock();

}; // namespace sapien
