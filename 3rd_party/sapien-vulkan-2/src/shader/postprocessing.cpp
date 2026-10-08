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
#include "svulkan2/shader/postprocessing.h"
#include "reflect.h"
#include "svulkan2/common/launch_policy.h"
#include "../common/logger.h"

namespace svulkan2 {
namespace shader {

std::future<void>
PostprocessingShaderParser::loadFileAsync(std::string const &filepath) {
  return std::async(LAUNCH_ASYNC, [=, this]() {
    logger::info("Compiling: " + filepath);
    mSPVCode = GLSLCompiler::compileGlslFileCached(
        vk::ShaderStageFlagBits::eCompute, filepath);
    logger::info("Compiled: " + filepath);
    reflectSPV();
  });
}

void PostprocessingShaderParser::reflectSPV() {
  spirv_cross::Compiler compiler(mSPVCode);
  auto resources = compiler.get_shader_resources();
  for (auto &r : resources.storage_images) {
    uint32_t setNumber =
        compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet);
    uint32_t bindingNumber =
        compiler.get_decoration(r.id, spv::Decoration::DecorationBinding);
    vk::Format format =
        get_image_format(compiler.get_type(r.type_id).image.format);
    int dim = compiler.get_type(r.type_id).array.size();
    int arraySize = dim == 1 ? compiler.get_type(r.type_id).array[0] : 0;

    mResources[setNumber].images.push_back(r.name);
    mResources[setNumber].bindings[bindingNumber] = {
        .name = r.name,
        .type = vk::DescriptorType::eStorageImage,
        .dim = dim,
        .arraySize = arraySize,
        .arrayIndex =
            static_cast<uint32_t>(mResources[setNumber].images.size() - 1),
        .format = format};
  }

  // TODO: validate only 1 set is used and everything is storage image
}

} // namespace shader
} // namespace svulkan2