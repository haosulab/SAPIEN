#include "sapien/profiler.h"
#ifdef SAPIEN_CUDA
#include <nvtx3/nvToolsExt.h>
#endif

namespace sapien {
#ifdef SAPIEN_CUDA
void ProfilerEvent(char const *name) { nvtxMarkA(name); }
void ProfilerBlockBegin(char const *name) { nvtxRangePushA(name); }
void ProfilerBlockEnd() { nvtxRangePop(); }
#else
void ProfilerEvent(char const *name) { }
void ProfilerBlockBegin(char const *name) { }
void ProfilerBlockEnd() { }
#endif
} // namespace sapien
