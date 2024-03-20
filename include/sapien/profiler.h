#pragma once

namespace sapien {

void ProfilerEvent(char const *name);
void ProfilerBlockBegin(char const *name);
void ProfilerBlockEnd();

class ProfilerBlock {
public:
  ProfilerBlock(char const *name) { ProfilerBlockBegin(name); }
  ~ProfilerBlock() { ProfilerBlockEnd(); }
};

}; // namespace sapien

#define SAPIEN_PROFILE_CONCAT_(prefix, suffix) prefix##suffix
#define SAPIEN_PROFILE_CONCAT(prefix, suffix) SAPIEN_PROFILE_CONCAT_(prefix, suffix)
#define SAPIEN_PROFILE_FUNCTION ::sapien::ProfilerBlock SAPIEN_PROFILE_CONCAT(sapien_profiler_block_, __LINE__)(__func__);
#define SAPIEN_PROFILE_BLOCK(name) ::sapien::ProfilerBlock SAPIEN_PROFILE_CONCAT(sapien_profiler_block_,__LINE__)(#name);
#define SAPIEN_PROFILE_BLOCK_BEGIN(name) ::sapien::ProfilerBlockBegin(#name);
#define SAPIEN_PROFILE_BLOCK_END ::sapien::ProfilerBlockEnd();
