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
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>
#include <vulkan/vulkan.hpp>

namespace svulkan2 {
namespace fs = std::filesystem;
class GLSLCompiler {
public:
  static std::vector<std::uint32_t> const &
  compileGlslFileCached(vk::ShaderStageFlagBits stage,
                        fs::path const &filepath);

  static std::tuple<std::string, std::vector<std::tuple<std::string, int>>>
  loadGlslCodeWithDebugInfo(fs::path const &filepath);
  // static std::string loadGlslCode(fs::path const &filepath);
  static std::vector<std::uint32_t>
  compileToSpirv(vk::ShaderStageFlagBits shaderStage,
                 std::string const &glslCode,
                 std::vector<std::tuple<std::string, int>> const &debugInfo = {});
  static void InitializeProcess();
  static void FinalizeProcess();

private:
};

} // namespace svulkan2