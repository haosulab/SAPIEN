/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
