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
#include "glsl_compiler.h"
#include "svulkan2/common/err.h"
#include "svulkan2/common/fs.h"
#include "svulkan2/common/layout.h"
#include <future>
#include <map>
#include <optional>

namespace spirv_cross {
class Compiler;
struct SPIRType;
struct Resource;
}; // namespace spirv_cross

namespace svulkan2 {
namespace shader {

enum UniformBindingType {
  eScene,
  eObject,
  eCamera,
  eMaterial,
  eTextures,
  eLight,

  eRTScene,
  eRTCamera,
  eRTOutput,

  eNone,
  eUnknown
};

std::vector<uint32_t> getDescriptorSetIds(spirv_cross::Compiler &compiler);

struct DescriptorSetDescription {
  struct Binding {
    std::string name;
    vk::DescriptorType type;
    int dim;
    int arraySize;
    int imageDim{0};     // e.g. sampler1D
    uint32_t arrayIndex; // index in the buffers/samplers vector
    vk::Format format;   // only used for storage image
  };
  UniformBindingType type{eUnknown};
  std::vector<std::shared_ptr<StructDataLayout>> buffers;
  std::vector<std::string> samplers;
  std::vector<std::string> images;
  std::map<uint32_t, Binding> bindings;

  DescriptorSetDescription merge(DescriptorSetDescription const &other) const;
};

// TODO rename
/** Options configured by the shaders  */
struct ShaderConfig {
  std::shared_ptr<InputDataLayout> vertexLayout;
  std::shared_ptr<InputDataLayout> primitiveVertexLayout;
  std::shared_ptr<StructDataLayout> objectDataBufferLayout;
  std::shared_ptr<StructDataLayout> sceneBufferLayout;
  std::shared_ptr<StructDataLayout> cameraBufferLayout;
  std::shared_ptr<StructDataLayout> lightBufferLayout;
  std::shared_ptr<StructDataLayout> shadowBufferLayout;

  DescriptorSetDescription sceneSetDescription;
  DescriptorSetDescription cameraSetDescription;
  DescriptorSetDescription objectSetDescription;
  DescriptorSetDescription lightSetDescription;
};

DescriptorSetDescription getDescriptorSetDescription(spirv_cross::Compiler &compiler,
                                                     uint32_t setNumber);

inline std::string getOutTextureName(std::string variableName) { // remove "out" prefix
  if (variableName.substr(0, 3) != "out") {
    throw std::runtime_error("Output texture must start with \"out\"");
  }
  return variableName.substr(3, std::string::npos);
}

inline static std::string getInTextureName(std::string variableName) { // remove "sampler" prefix
  if (variableName.substr(0, 7) != "sampler") {
    throw std::runtime_error("Input texture must start with \"sampler\"");
  }
  return variableName.substr(7, std::string::npos);
}

std::shared_ptr<InputDataLayout> parseInputData(spirv_cross::Compiler &compiler);
std::shared_ptr<InputDataLayout> parseVertexInput(spirv_cross::Compiler &compiler);

std::shared_ptr<OutputDataLayout> parseOutputData(spirv_cross::Compiler &compiler);
std::shared_ptr<OutputDataLayout> parseTextureOutput(spirv_cross::Compiler &compiler);

bool hasUniformBuffer(spirv_cross::Compiler &compiler, uint32_t bindingNumber, uint32_t setNumber);

std::shared_ptr<StructDataLayout> parseBuffer(spirv_cross::Compiler &compiler,
                                              uint32_t bindingNumber, uint32_t setNumber);
std::shared_ptr<StructDataLayout> parseBuffer(spirv_cross::Compiler &compiler,
                                              spirv_cross::SPIRType const &type);
std::shared_ptr<StructDataLayout> parseBuffer(spirv_cross::Compiler &compiler,
                                              spirv_cross::Resource &resource);

std::shared_ptr<SpecializationConstantLayout>
parseSpecializationConstant(spirv_cross::Compiler &compiler);

class BaseParser {

protected:
  std::string mName;
  std::vector<uint32_t> mVertSPVCode;
  std::vector<uint32_t> mFragSPVCode;
  std::vector<uint32_t> mGeomSPVCode;

  int mIndex{-1};

public:
  void loadGLSLFiles(std::string const &vertFile, std::string const &fragFile,
                     std::string const &geomFile = "");

  std::future<void> loadGLSLFilesAsync(std::string const &vertFile, std::string const &fragFile,
                                       std::string const &geomFile = "");

  virtual std::shared_ptr<OutputDataLayout> getTextureOutputLayout() const = 0;
  virtual std::vector<std::string> getColorRenderTargetNames() const = 0;
  virtual std::optional<std::string> getDepthRenderTargetName() const { return {}; };

  virtual std::vector<UniformBindingType> getUniformBindingTypes() const;

  /** name of textures used in the texture descriptor set (only in
   * deferred-like passes) */
  virtual std::vector<std::string> getInputTextureNames() const;

  inline void setName(std::string const &name) { mName = name; }
  inline std::string getName() const { return mName; }

  // TODO: push constant
  virtual vk::UniquePipelineLayout
  createPipelineLayout(vk::Device device, std::vector<vk::DescriptorSetLayout> layouts) const;

  virtual vk::UniqueRenderPass createRenderPass(
      vk::Device device, std::vector<vk::Format> const &colorFormats, vk::Format depthFormat,
      std::vector<std::pair<vk::ImageLayout, vk::ImageLayout>> const &colorTargetLayouts,
      std::pair<vk::ImageLayout, vk::ImageLayout> const &depthLayout,
      vk::SampleCountFlagBits sampleCount) const = 0;

  virtual vk::UniquePipeline
  createPipeline(vk::Device device, vk::PipelineLayout layout, vk::RenderPass renderPass,
                 vk::CullModeFlags cullMode, vk::FrontFace frontFace, bool alphaBlend,
                 vk::SampleCountFlagBits sampleCount,
                 std::map<std::string, SpecializationConstantValue> const
                     &specializationConstantInfo) const = 0;

  virtual std::vector<DescriptorSetDescription> getDescriptorSetDescriptions() const = 0;

  // inline float getResolutionScale() const { return mResolutionScale; }

  inline void setIndex(int index) { mIndex = index; }
  inline int getIndex() const { return mIndex; }

  virtual ~BaseParser() = default;

protected:
  virtual void reflectSPV() = 0;
};
} // namespace shader
} // namespace svulkan2