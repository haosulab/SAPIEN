#include "sapien/profiler.h"
#include <nvtx3/nvToolsExt.h>

namespace sapien {

void ProfilerEvent(char const *name) { nvtxMarkA(name); }
void ProfilerBlockBegin(char const *name) { nvtxRangePushA(name); }
void ProfilerBlockEnd() { nvtxRangePop(); }

} // namespace sapien
