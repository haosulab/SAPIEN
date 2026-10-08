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
#include "svulkan2/shader/rt.h"
#include "../common/logger.h"
#include "reflect.h"
#include "svulkan2/common/launch_policy.h"
#include "svulkan2/core/buffer.h"
#include "svulkan2/core/context.h"

namespace svulkan2 {
namespace shader {

static std::string
summarizeResources(std::unordered_map<uint32_t, DescriptorSetDescription> const &resources) {
  std::stringstream ss;
  for (auto &[sid, set] : resources) {
    ss << "\nSet " << std::setw(2) << sid;
    if (set.type != UniformBindingType::eUnknown) {
      if (set.type == UniformBindingType::eRTCamera) {
        ss << "    Camera";
      } else if (set.type == UniformBindingType::eRTScene) {
        ss << "     Scene";
      } else if (set.type == UniformBindingType::eRTOutput) {
        ss << "    Output";
      }
    }
    ss << "\n";
    for (auto &[bid, b] : set.bindings) {
      ss << "  Binding " << std::setw(2) << bid << std::setw(20) << b.name;
      switch (b.type) {
      case vk::DescriptorType::eUniformBuffer:
        ss << " UniformBuffer\n";
        ss << "    Dim  " << b.dim << "\n";
        if (b.dim > 0) {
          ss << "    Size " << b.arraySize << "\n";
        }
        break;
      case vk::DescriptorType::eStorageBuffer:
        ss << " StorageBuffer\n";
        ss << "    Dim  " << b.dim << "\n";
        if (b.dim > 0) {
          ss << "    Size " << b.arraySize << "\n";
        }
        ss << "    " << std::setw(10) << "Field" << std::setw(10) << "offset" << std::setw(10)
           << "size\n";
        for (auto &elem : set.buffers[b.arrayIndex]->getElementsSorted()) {
          ss << "    " << std::setw(10) << elem->name << std::setw(10) << elem->offset
             << std::setw(10) << elem->size << "\n";
        }
        break;
      case vk::DescriptorType::eCombinedImageSampler:
        ss << " CombinedImageSampler\n";
        break;
      case vk::DescriptorType::eStorageImage:
        ss << " StorageImage\n";
        break;
      case vk::DescriptorType::eAccelerationStructureKHR:
        ss << " AccelerationStructure\n";
        break;
      default:
        ss << " Unknown\n";
        break;
      }
    }
  }
  return ss.str();
}

std::future<void> RayTracingStageParser::loadFileAsync(std::string const &filepath,
                                                       vk::ShaderStageFlagBits stage) {
  return std::async(LAUNCH_ASYNC, [=, this]() {
    logger::info("Compiling: " + filepath);
    mSPVCode = GLSLCompiler::compileGlslFileCached(stage, filepath);
    logger::info("Compiled: " + filepath);
    reflectSPV();
  });
}

void RayTracingStageParser::reflectSPV() {
  mResources.clear();

  spirv_cross::Compiler compiler(mSPVCode);
  auto resources = compiler.get_shader_resources();

  for (auto &r : resources.uniform_buffers) {
    uint32_t setNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet);
    uint32_t bindingNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationBinding);
    int dim = compiler.get_type(r.type_id).array.size();
    int arraySize = dim == 1 ? compiler.get_type(r.type_id).array[0] : 0;

    if (mResources[setNumber].bindings.contains(bindingNumber)) {
      logger::critical("duplicated set {} binding {}", setNumber, bindingNumber);
      throw std::runtime_error("shader compilation failed");
    }

    mResources[setNumber].buffers.push_back(parseBuffer(compiler, r));
    mResources[setNumber].bindings[bindingNumber] = {
        .name = r.name,
        .type = vk::DescriptorType::eUniformBuffer,
        .dim = dim,
        .arraySize = arraySize,
        .arrayIndex = static_cast<uint32_t>(mResources[setNumber].buffers.size() - 1)};
  }

  for (auto &r : resources.storage_buffers) {
    uint32_t setNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet);
    uint32_t bindingNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationBinding);
    int dim = compiler.get_type(r.type_id).array.size();
    int arraySize = dim == 1 ? compiler.get_type(r.type_id).array[0] : 0;

    if (mResources[setNumber].bindings.contains(bindingNumber)) {
      logger::critical("duplicated set {} binding {}", setNumber, bindingNumber);
      throw std::runtime_error("shader compilation failed");
    }

    mResources[setNumber].buffers.push_back(parseBuffer(compiler, r));
    mResources[setNumber].bindings[bindingNumber] = {
        .name = r.name,
        .type = vk::DescriptorType::eStorageBuffer,
        .dim = dim,
        .arraySize = arraySize,
        .arrayIndex = static_cast<uint32_t>(mResources[setNumber].buffers.size() - 1)};
  }

  for (auto &r : resources.acceleration_structures) {
    uint32_t setNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet);
    uint32_t bindingNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationBinding);
    int dim = compiler.get_type(r.type_id).array.size();
    int arraySize = dim == 1 ? compiler.get_type(r.type_id).array[0] : 0;

    mResources[setNumber].bindings[bindingNumber] = {
        .name = r.name,
        .type = vk::DescriptorType::eAccelerationStructureKHR,
        .dim = dim,
        .arraySize = arraySize,
        .arrayIndex = 0};
  }

  for (auto &r : resources.storage_images) {
    uint32_t setNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet);
    uint32_t bindingNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationBinding);
    vk::Format format = get_image_format(compiler.get_type(r.type_id).image.format);
    int dim = compiler.get_type(r.type_id).array.size();
    int arraySize = dim == 1 ? compiler.get_type(r.type_id).array[0] : 0;

    mResources[setNumber].images.push_back(r.name);
    mResources[setNumber].bindings[bindingNumber] = {
        .name = r.name,
        .type = vk::DescriptorType::eStorageImage,
        .dim = dim,
        .arraySize = arraySize,
        .arrayIndex = static_cast<uint32_t>(mResources[setNumber].images.size() - 1),
        .format = format};
  }

  for (auto &r : resources.sampled_images) {
    uint32_t setNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet);
    uint32_t bindingNumber = compiler.get_decoration(r.id, spv::Decoration::DecorationBinding);
    // log::info("name {} format {}", r.name,
    // compiler.get_type(r.type_id).image.format);
    int dim = compiler.get_type(r.type_id).array.size();
    int arraySize = dim == 1 ? compiler.get_type(r.type_id).array[0] : 0;

    mResources[setNumber].samplers.push_back(r.name);
    mResources[setNumber].bindings[bindingNumber] = {
        .name = r.name,
        .type = vk::DescriptorType::eCombinedImageSampler,
        .dim = dim,
        .arraySize = arraySize,
        .arrayIndex = static_cast<uint32_t>(mResources[setNumber].samplers.size() - 1)};
  }

  for (auto &r : resources.push_constant_buffers) {
    auto const &type = compiler.get_type(r.type_id);
    if (type.basetype != spirv_cross::SPIRType::Struct) {
      throw std::runtime_error("push constant buffer must be a struct");
    }
    mPushConstantLayout = parseBuffer(compiler, type);
  }

  logger::info("\n" + summary());

  // TODO: analyze rayPayload
  // TODO: enforce light buffer layout
}

std::string RayTracingStageParser::summary() const { return summarizeResources(mResources); }

RayTracingShaderPack::RayTracingShaderPack(std::string const &shaderDir) {
  fs::path path(shaderDir);

  std::vector<std::future<void>> futures;
  std::vector<RayTracingStageParser *> allParsers;

  // ray gen stage
  {
    auto raygenFile = path / "camera.rgen";
    if (!fs::exists(raygenFile)) {
      throw std::runtime_error("ray tracing shader directory must contain camera.rgen");
    }
    mRaygenStageParser = std::make_unique<RayTracingStageParser>();
    allParsers.push_back(mRaygenStageParser.get());
    futures.push_back(mRaygenStageParser->loadFileAsync(raygenFile.string(),
                                                        vk::ShaderStageFlagBits::eRaygenKHR));
  }

  // miss stages
  {
    auto cameraRmissFile = path / "camera.rmiss";
    if (!fs::exists(cameraRmissFile)) {
      throw std::runtime_error("ray tracing shader directory must contain camera.rmiss");
    }
    mMissGroupParsers.push_back({"camera", std::make_unique<RayTracingStageParser>()});
    allParsers.push_back(mMissGroupParsers.back().miss.get());
    futures.push_back(mMissGroupParsers.back().miss->loadFileAsync(
        cameraRmissFile.string(), vk::ShaderStageFlagBits::eMissKHR));
  }

  {
    auto shadowRmissFile = path / "shadow.rmiss";
    if (!fs::exists(shadowRmissFile)) {
      throw std::runtime_error("ray tracing shader directory must contain shadow.rmiss");
    }
    mMissGroupParsers.push_back({"shadow", std::make_unique<RayTracingStageParser>()});
    allParsers.push_back(mMissGroupParsers.back().miss.get());
    futures.push_back(mMissGroupParsers.back().miss->loadFileAsync(
        shadowRmissFile.string(), vk::ShaderStageFlagBits::eMissKHR));
  }

  // mesh hit
  {
    auto camRahitFile = path / "camera.rahit";
    auto camRchitFile = path / "camera.rchit";
    if (!fs::exists(camRchitFile) || !fs::exists(camRahitFile)) {
      throw std::runtime_error(
          "ray tracing shader directory must contain camera.rchit and camera.rahit ");
    }
    mHitGroupParsers.push_back({.name = "camera",
                                .any = std::make_unique<RayTracingStageParser>(),
                                .closest = std::make_unique<RayTracingStageParser>()});
    allParsers.push_back(mHitGroupParsers.back().any.get());
    allParsers.push_back(mHitGroupParsers.back().closest.get());
    futures.push_back(mHitGroupParsers.back().any->loadFileAsync(
        camRahitFile.string(), vk::ShaderStageFlagBits::eAnyHitKHR));
    futures.push_back(mHitGroupParsers.back().closest->loadFileAsync(
        camRchitFile.string(), vk::ShaderStageFlagBits::eClosestHitKHR));
  }

  // TODO: point rahit
  // point hit
  {
    auto pointRchitFile = path / "point.rchit";
    auto pointIntersectFile = path / "point.rint";
    if (fs::exists(pointIntersectFile) || fs::exists(pointRchitFile)) {
      if (!(fs::exists(pointIntersectFile) && fs::exists(pointRchitFile))) {
        throw std::runtime_error(
            "ray tracing shader directory should contain both point.rint and point.rchit");
      }
      mHitGroupParsers.push_back({.name = "point",
                                  .closest = std::make_unique<RayTracingStageParser>(),
                                  .intersect = std::make_unique<RayTracingStageParser>()});
      allParsers.push_back(mHitGroupParsers.back().closest.get());
      allParsers.push_back(mHitGroupParsers.back().intersect.get());
      futures.push_back(mHitGroupParsers.back().intersect->loadFileAsync(
          pointIntersectFile.string(), vk::ShaderStageFlagBits::eIntersectionKHR));
      futures.push_back(mHitGroupParsers.back().closest->loadFileAsync(
          pointRchitFile.string(), vk::ShaderStageFlagBits::eClosestHitKHR));
    }
  }

  // wait for glsl loading to finish
  for (auto &f : futures) {
    f.get();
  }

  // reflect set layouts
  mResources = mRaygenStageParser->getResources();
  std::shared_ptr<StructDataLayout> pushConstantLayout;
  for (auto &parser : allParsers) {
    for (auto &[sid, set] : parser->getResources()) {
      if (mResources.contains(sid)) {
        mResources[sid] = mResources.at(sid).merge(set);

        logger::info("Merging...");

        {
          logger::info("Left");
          auto set = mResources.at(sid);
          std::stringstream ss;
          ss << "\nSet " << std::setw(2) << sid;
          if (set.type != UniformBindingType::eUnknown) {
            if (set.type == UniformBindingType::eRTCamera) {
              ss << "    Camera";
            } else if (set.type == UniformBindingType::eRTScene) {
              ss << "     Scene";
            } else if (set.type == UniformBindingType::eRTOutput) {
              ss << "    Output";
            }
          }
          ss << "\n";
          for (auto &[bid, b] : set.bindings) {
            ss << "  Binding " << std::setw(2) << bid << std::setw(20) << b.name;
            switch (b.type) {
            case vk::DescriptorType::eUniformBuffer:
              ss << " UniformBuffer\n";
              ss << "    Dim  " << b.dim << "\n";
              if (b.dim > 0) {
                ss << "    Size " << b.arraySize << "\n";
              }
              break;
            case vk::DescriptorType::eStorageBuffer:
              ss << " StorageBuffer\n";
              ss << "    Dim  " << b.dim << "\n";
              if (b.dim > 0) {
                ss << "    Size " << b.arraySize << "\n";
              }
              ss << "    " << std::setw(10) << "Field" << std::setw(10) << "offset"
                 << std::setw(10) << "size\n";
              for (auto &elem : set.buffers[b.arrayIndex]->getElementsSorted()) {
                ss << "    " << std::setw(10) << elem->name << std::setw(10) << elem->offset
                   << std::setw(10) << elem->size << "\n";
              }
              break;
            case vk::DescriptorType::eCombinedImageSampler:
              ss << " CombinedImageSampler\n";
              break;
            case vk::DescriptorType::eStorageImage:
              ss << " StorageImage\n";
              break;
            case vk::DescriptorType::eAccelerationStructureKHR:
              ss << " AccelerationStructure\n";
              break;
            default:
              ss << " Unknown\n";
              break;
            }
          }
          logger::info(ss.str());
        }

        {
          logger::info("Right");
          std::stringstream ss;
          ss << "\nSet " << std::setw(2) << sid;
          if (set.type != UniformBindingType::eUnknown) {
            if (set.type == UniformBindingType::eRTCamera) {
              ss << "    Camera";
            } else if (set.type == UniformBindingType::eRTScene) {
              ss << "     Scene";
            } else if (set.type == UniformBindingType::eRTOutput) {
              ss << "    Output";
            }
          }
          ss << "\n";
          for (auto &[bid, b] : set.bindings) {
            ss << "  Binding " << std::setw(2) << bid << std::setw(20) << b.name;
            switch (b.type) {
            case vk::DescriptorType::eUniformBuffer:
              ss << " UniformBuffer\n";
              ss << "    Dim  " << b.dim << "\n";
              if (b.dim > 0) {
                ss << "    Size " << b.arraySize << "\n";
              }
              break;
            case vk::DescriptorType::eStorageBuffer:
              ss << " StorageBuffer\n";
              ss << "    Dim  " << b.dim << "\n";
              if (b.dim > 0) {
                ss << "    Size " << b.arraySize << "\n";
              }
              ss << "    " << std::setw(10) << "Field" << std::setw(10) << "offset"
                 << std::setw(10) << "size\n";
              for (auto &elem : set.buffers[b.arrayIndex]->getElementsSorted()) {
                ss << "    " << std::setw(10) << elem->name << std::setw(10) << elem->offset
                   << std::setw(10) << elem->size << "\n";
              }
              break;
            case vk::DescriptorType::eCombinedImageSampler:
              ss << " CombinedImageSampler\n";
              break;
            case vk::DescriptorType::eStorageImage:
              ss << " StorageImage\n";
              break;
            case vk::DescriptorType::eAccelerationStructureKHR:
              ss << " AccelerationStructure\n";
              break;
            default:
              ss << " Unknown\n";
              break;
            }
          }
          logger::info(ss.str());
        }

      } else {
        mResources[sid] = set;
      }
      if (auto layout = parser->getPushConstantLayout()) {
        if (!pushConstantLayout) {
          pushConstantLayout = layout;
        } else {
          if (*pushConstantLayout != *layout) {
            throw std::runtime_error("push_constant must be the same for all shader stages");
          }
        }
      }
    }
  }
  mPushConstantLayout = pushConstantLayout;

  // determine set types
  for (uint32_t setId = 0;; setId++) {
    if (!mResources.contains(setId)) {
      if (mResources.size() != setId) {
        throw std::runtime_error("descriptor sets numbers must be "
                                 "consecurive integers starting from 0");
      }
      break;
    }
    for (auto &[bid, b] : mResources.at(setId).bindings) {
      if (b.type == vk::DescriptorType::eAccelerationStructureKHR) {
        mResources.at(setId).type = UniformBindingType::eRTScene;
        break;
      } else if (b.name == "CameraBuffer") {
        mResources.at(setId).type = UniformBindingType::eRTCamera;
        break;
      } else if (b.name.starts_with("out") && b.type == vk::DescriptorType::eStorageImage) {
        mResources.at(setId).type = UniformBindingType::eRTOutput;
        break;
      }
    }
    if (mResources.at(setId).type == UniformBindingType::eUnknown) {
      throw std::runtime_error("cannot identify descriptor set " + std::to_string(setId));
    }
  }

  // load post processing shaders
  // TODO: handle more postprocessing files
  auto postprocessingFile = path / "postprocessing.comp";
  if (fs::exists(postprocessingFile)) {
    auto pp = std::make_unique<PostprocessingShaderParser>();
    pp->loadFileAsync(postprocessingFile.string()).get();
    mPostprocessingParsers.push_back(std::move(pp));
  }
}

StructDataLayout const &RayTracingShaderPack::getMaterialBufferLayout() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTScene) {
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "Materials") {
          return *set.buffers.at(binding.arrayIndex)->elements.begin()->second.member;
        }
      }
    }
  }
  throw std::runtime_error("failed to retrieve material buffer in shader");
}

StructDataLayout const &RayTracingShaderPack::getObjectBufferLayout() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTScene) {
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "Objects") {
          return *set.buffers.at(binding.arrayIndex)->elements.begin()->second.member;
        }
      }
    }
  }
  throw std::runtime_error("failed to retrieve object buffer in shader");
}

StructDataLayout const &RayTracingShaderPack::getTextureIndexBufferLayout() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTScene) {
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "TextureIndices") {
          return *set.buffers.at(binding.arrayIndex)->elements.begin()->second.member;
        }
      }
    }
  }
  throw std::runtime_error("failed to retrieve texture index buffer in shader");
}

StructDataLayout const &RayTracingShaderPack::getGeometryInstanceBufferLayout() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTScene) {
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "GeometryInstances") {
          return *set.buffers.at(binding.arrayIndex)->elements.begin()->second.member;
        }
      }
    }
  }
  throw std::runtime_error("failed to retrieve texture index buffer in shader");
}

StructDataLayout const &RayTracingShaderPack::getCameraBufferLayout() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTCamera) {
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "CameraBuffer") {
          return *set.buffers.at(binding.arrayIndex);
        }
      }
    }
  }
  throw std::runtime_error("failed to retrieve texture index buffer in shader");
}

DescriptorSetDescription const &RayTracingShaderPack::getOutputDescription() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTOutput) {
      return set;
    }
  }
  throw std::runtime_error("failed to retrieve output descriptor set");
}

DescriptorSetDescription const &RayTracingShaderPack::getCameraDescription() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTCamera) {
      return set;
    }
  }
  throw std::runtime_error("failed to retrieve output descriptor set");
}

DescriptorSetDescription const &RayTracingShaderPack::getSceneDescription() const {
  for (auto &[sid, set] : getResources()) {
    if (set.type == UniformBindingType::eRTScene) {
      return set;
    }
  }
  throw std::runtime_error("failed to retrieve output descriptor set");
}

std::shared_ptr<InputDataLayout> RayTracingShaderPack::computeCompatibleInputVertexLayout() const {
  for (auto &[sid, set] : mResources) {
    if (set.type == UniformBindingType::eRTScene) {
      auto inputLayout = std::make_shared<InputDataLayout>();

      uint32_t structOffset{};
      uint32_t location{0};
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "Vertices" && binding.type == vk::DescriptorType::eStorageBuffer) {
          if (set.buffers.at(binding.arrayIndex)->elements.size() != 1) {
            throw std::runtime_error("Vertices buffer must contain a single "
                                     "array of structs (Vertex)");
          }
          auto elem = set.buffers.at(binding.arrayIndex)->elements.begin()->second;
          if (elem.array.size() != 1 || elem.dtype != DataType::STRUCT()) {
            throw std::runtime_error("Vertices buffer must contain a single "
                                     "array of structs (Vertex)");
          }
          auto vertex = elem.member;
          if (!vertex->elements.contains("x") ||
              vertex->elements.at("x").dtype != DataType::FLOAT() ||
              vertex->elements.at("x").offset != 0 || !vertex->elements.contains("y") ||
              vertex->elements.at("y").dtype != DataType::FLOAT() ||
              vertex->elements.at("y").offset != 4 || !vertex->elements.contains("z") ||
              vertex->elements.at("z").dtype != DataType::FLOAT() ||
              vertex->elements.at("z").offset != 8) {
            throw std::runtime_error("Vertex struct must contain float variables named "
                                     "x, y, and z at the beginning");
          }
          structOffset = 12;
          inputLayout->elements["position"] = {
              .name = "position", .location = location++, .dtype = DataType::FLOAT3()};

          if (vertex->elements.contains("nx")) {
            if (vertex->elements.at("nx").offset != structOffset) {
              throw std::runtime_error("Vertex struct members must follow the order of position "
                                       "normal uv tangent bitangent color without padding");
            }
            if (!vertex->elements.contains("ny") || !vertex->elements.contains("nz")) {
              throw std::runtime_error("if a Vertex struct contains float "
                                       "nx, it must contain ny, nz");
            }
            if (vertex->elements.at("nx").dtype != DataType::FLOAT() ||
                vertex->elements.at("ny").dtype != DataType::FLOAT() ||
                vertex->elements.at("nz").dtype != DataType::FLOAT()) {
              throw std::runtime_error("Vertex struct member nx, ny, nz must be float");
            }
            uint32_t offset = vertex->elements.at("nx").offset;
            if (vertex->elements.at("ny").offset != offset + 4 ||
                vertex->elements.at("nz").offset != offset + 8) {
              throw std::runtime_error("Vertex struct member nx, ny, nz must be consecutive");
            }
            structOffset += 12;
            inputLayout->elements["normal"] = {
                .name = "normal", .location = location++, .dtype = DataType::FLOAT3()};
          }

          if (vertex->elements.contains("u")) {
            if (vertex->elements.at("u").offset != structOffset) {
              throw std::runtime_error("Vertex struct members must follow the order of position "
                                       "normal uv tangent bitangent color without padding");
            }
            if (!vertex->elements.contains("v")) {
              throw std::runtime_error("if a Vertex struct contains float u, it must contain v");
            }
            if (vertex->elements.at("u").dtype != DataType::FLOAT() ||
                vertex->elements.at("v").dtype != DataType::FLOAT()) {
              throw std::runtime_error("Vertex struct member u, v must be float");
            }
            uint32_t offset = vertex->elements.at("u").offset;
            if (vertex->elements.at("v").offset != offset + 4) {
              throw std::runtime_error("Vertex struct member must be consecutive");
            }
            structOffset += 8;
            inputLayout->elements["uv"] = {
                .name = "uv", .location = location++, .dtype = DataType::FLOAT2()};
          }

          if (vertex->elements.contains("tx")) {
            if (vertex->elements.at("tx").offset != structOffset) {
              throw std::runtime_error("Vertex struct members must follow the order of position "
                                       "normal uv tangent bitangent color without padding");
            }
            if (!vertex->elements.contains("ty") || !vertex->elements.contains("tz")) {
              throw std::runtime_error("if a Vertex struct contains float "
                                       "tx, it must contain ty, tz");
            }
            if (vertex->elements.at("tx").dtype != DataType::FLOAT() ||
                vertex->elements.at("ty").dtype != DataType::FLOAT() ||
                vertex->elements.at("tz").dtype != DataType::FLOAT()) {
              throw std::runtime_error("Vertex struct member tx, ty, tz must be float");
            }
            uint32_t offset = vertex->elements.at("tx").offset;
            if (vertex->elements.at("ty").offset != offset + 4 ||
                vertex->elements.at("tz").offset != offset + 8) {
              throw std::runtime_error("Vertex struct member tx, ty, tz must be consecutive");
            }
            structOffset += 12;
            inputLayout->elements["tangent"] = {
                .name = "tangent", .location = location++, .dtype = DataType::FLOAT3()};
          }

          if (vertex->elements.contains("bx")) {
            if (vertex->elements.at("bx").offset != structOffset) {
              throw std::runtime_error("Vertex struct members must follow the order of position "
                                       "normal uv tangent bitangent color without padding");
            }
            if (!vertex->elements.contains("by") || !vertex->elements.contains("bz")) {
              throw std::runtime_error("if a Vertex struct contains float "
                                       "bx, it must contain by, bz");
            }
            if (vertex->elements.at("bx").dtype != DataType::FLOAT() ||
                vertex->elements.at("by").dtype != DataType::FLOAT() ||
                vertex->elements.at("bz").dtype != DataType::FLOAT()) {
              throw std::runtime_error("Vertex struct member bx, by, bz must be float");
            }
            uint32_t offset = vertex->elements.at("bx").offset;
            if (vertex->elements.at("by").offset != offset + 4 ||
                vertex->elements.at("bz").offset != offset + 8) {
              throw std::runtime_error("Vertex struct member bx, by, bz must be consecutive");
            }
            structOffset += 12;
            inputLayout->elements["bitangent"] = {
                .name = "bitangent", .location = location++, .dtype = DataType::FLOAT3()};
          }

          if (vertex->elements.contains("r")) {
            if (vertex->elements.at("r").offset != structOffset) {
              throw std::runtime_error("Vertex struct members must follow the order of position "
                                       "normal uv tangent bitangent color without padding");
            }
            if (!vertex->elements.contains("g") || !vertex->elements.contains("b") ||
                !vertex->elements.contains("a")) {
              throw std::runtime_error("if a Vertex struct contains float r, "
                                       "it must contain g, b, a");
            }
            if (vertex->elements.at("r").dtype != DataType::FLOAT() ||
                vertex->elements.at("g").dtype != DataType::FLOAT() ||
                vertex->elements.at("b").dtype != DataType::FLOAT() ||
                vertex->elements.at("a").dtype != DataType::FLOAT()) {
              throw std::runtime_error("Vertex struct member r, g, b, a must be float");
            }
            uint32_t offset = vertex->elements.at("r").offset;
            if (vertex->elements.at("g").offset != offset + 4 ||
                vertex->elements.at("b").offset != offset + 8 ||
                vertex->elements.at("a").offset != offset + 12) {
              throw std::runtime_error("Vertex struct member r, g, b, a must be consecutive");
            }
            structOffset += 16;
            inputLayout->elements["color"] = {
                .name = "color", .location = location++, .dtype = DataType::FLOAT4()};
          }
        }
      }
      return inputLayout;
    }
  }
  throw std::runtime_error("failed to find the vertex buffer in the shader");
}

std::shared_ptr<InputDataLayout> RayTracingShaderPack::computePrimitiveLayout() const {
  for (auto &[sid, set] : mResources) {
    if (set.type == UniformBindingType::eRTScene) {
      auto inputLayout = std::make_shared<InputDataLayout>();

      uint32_t location{0};
      for (auto &[bid, binding] : set.bindings) {
        if (binding.name == "Points" && binding.type == vk::DescriptorType::eStorageBuffer) {
          if (set.buffers.at(binding.arrayIndex)->elements.size() != 1) {
            throw std::runtime_error(
                "Point buffer must contain a single array of structs (Point)");
          }
          auto elem = set.buffers.at(binding.arrayIndex)->elements.begin()->second;
          if (elem.array.size() != 1 || elem.dtype != DataType::STRUCT()) {
            throw std::runtime_error(
                "Point buffer must contain a single array of structs (Point)");
          }
          auto vertex = elem.member;

          for (auto &e : vertex->getElementsSorted()) {
            inputLayout->elements[e->name] = {
                .name = e->name, .location = location++, .dtype = e->dtype};
          }
        }
      }
      return inputLayout;
    }
  }
  return nullptr;
}

std::string RayTracingShaderPack::summary() const { return summarizeResources(mResources); }

RayTracingShaderPackInstance::RayTracingShaderPackInstance(RayTracingShaderPackInstanceDesc desc)
    : mDesc(desc) {
  mShaderPack = core::Context::Get()->getResourceManager()->CreateRTShaderPack(desc.shaderDir);
}

static vk::UniqueDescriptorSetLayout
createSceneDescriptorSetLayout(vk::Device device, DescriptorSetDescription const &description,
                               uint32_t maxMeshes, uint32_t maxMaterials, uint32_t maxTextures,
                               uint32_t maxPointSets) {
  std::vector<vk::DescriptorSetLayoutBinding> bindings;
  std::vector<vk::DescriptorBindingFlags> bindingFlags;
  for (uint32_t bid = 0; bid < description.bindings.size(); ++bid) {
    switch (description.bindings.at(bid).type) {
    case vk::DescriptorType::eAccelerationStructureKHR:
      bindings.push_back({
          bid,
          vk::DescriptorType::eAccelerationStructureKHR,
          1,
          vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eClosestHitKHR,
      });
      bindingFlags.push_back({});
      break;

    case vk::DescriptorType::eStorageBuffer:
      if (description.bindings.at(bid).name == "Materials") {
        bindings.push_back(
            {bid,
             vk::DescriptorType::eStorageBuffer,
             maxMaterials,
             vk::ShaderStageFlagBits::eAnyHitKHR | vk::ShaderStageFlagBits::eClosestHitKHR,
             {}});
        bindingFlags.push_back({});
      } else if (description.bindings.at(bid).name == "Vertices" ||
                 description.bindings.at(bid).name == "Indices") {
        bindings.push_back(
            {bid,
             vk::DescriptorType::eStorageBuffer,
             maxMeshes,
             vk::ShaderStageFlagBits::eAnyHitKHR | vk::ShaderStageFlagBits::eClosestHitKHR,
             {}});
        bindingFlags.push_back({});
      }

      else if (description.bindings.at(bid).name == "Points") {
        bindings.push_back({bid,
                            vk::DescriptorType::eStorageBuffer,
                            maxPointSets,
                            vk::ShaderStageFlagBits::eAnyHitKHR |
                                vk::ShaderStageFlagBits::eClosestHitKHR |
                                vk::ShaderStageFlagBits::eIntersectionKHR,
                            {}});
        bindingFlags.push_back({});

      } else if (description.bindings.at(bid).dim == 0) {
        bindings.push_back({bid,
                            vk::DescriptorType::eStorageBuffer,
                            1,
                            vk::ShaderStageFlagBits::eAnyHitKHR |
                                vk::ShaderStageFlagBits::eClosestHitKHR |
                                vk::ShaderStageFlagBits::eIntersectionKHR,
                            {}});
        bindingFlags.push_back({});
      } else {
        throw std::runtime_error("unrecognized storage buffer " +
                                 description.bindings.at(bid).name);
      }
      break;

    case vk::DescriptorType::eCombinedImageSampler:
      if (description.bindings.at(bid).name == "textures") {
        bindings.push_back(
            {bid,
             vk::DescriptorType::eCombinedImageSampler,
             maxTextures,
             vk::ShaderStageFlagBits::eAnyHitKHR | vk::ShaderStageFlagBits::eClosestHitKHR,
             {}});
        bindingFlags.push_back({});
      } else {
        bindings.push_back({
            bid,
            vk::DescriptorType::eCombinedImageSampler,
            1,
            vk::ShaderStageFlagBits::eAnyHitKHR | vk::ShaderStageFlagBits::eClosestHitKHR |
                vk::ShaderStageFlagBits::eMissKHR,
        });
        bindingFlags.push_back({});
      }
      break;
    default:
      throw std::runtime_error("only storage buffer, sampler2d, and acceleration structure are "
                               "allowed in the scene descriptor set");
    }
  }
  vk::DescriptorSetLayoutBindingFlagsCreateInfo flagInfo(bindingFlags);
  vk::DescriptorSetLayoutCreateInfo layoutInfo({}, bindings);
  layoutInfo.setPNext(&flagInfo);
  return device.createDescriptorSetLayoutUnique(layoutInfo);
}

static vk::UniqueDescriptorSetLayout
createCameraDescriptorSetLayout(vk::Device device, DescriptorSetDescription const &description) {
  if (description.bindings.size() > 1) {
    throw std::runtime_error("the camera set should contain a single CameraBuffer");
  }
  if (description.bindings.at(0).type != vk::DescriptorType::eUniformBuffer) {
    throw std::runtime_error("CameraBuffer must be uniform buffer");
  }
  vk::DescriptorSetLayoutBinding binding(0, vk::DescriptorType::eUniformBuffer, 1,
                                         vk::ShaderStageFlagBits::eRaygenKHR);
  return device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding));
}

static vk::UniqueDescriptorSetLayout
createOutputDescriptorSetLayout(vk::Device device, DescriptorSetDescription const &description) {
  std::vector<vk::DescriptorSetLayoutBinding> bindings;
  for (uint32_t bid = 0; bid < description.bindings.size(); ++bid) {
    if (description.bindings.at(bid).type != vk::DescriptorType::eStorageImage) {
      throw std::runtime_error("Only storage images are allowed in the output descriptor set");
    }
    bindings.push_back(
        {bid, vk::DescriptorType::eStorageImage, 1, vk::ShaderStageFlagBits::eRaygenKHR});
  }
  return device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, bindings));
}

static uint32_t alignUp(uint32_t size, uint32_t alignment) {
  return (size + (alignment - 1)) & ~(alignment - 1);
}

void RayTracingShaderPackInstance::initPipeline() {
  if (mPipeline) {
    return;
  }

  auto context = core::Context::Get();
  vk::Device device = context->getDevice();

  std::vector<vk::PushConstantRange> pushConstantRanges;
  if (mShaderPack->getPushConstantLayout()) {
    pushConstantRanges.push_back(vk::PushConstantRange(
        vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eMissKHR |
            vk::ShaderStageFlagBits::eClosestHitKHR | vk::ShaderStageFlagBits::eAnyHitKHR |
            vk::ShaderStageFlagBits::eCompute,
        0, mShaderPack->getPushConstantLayout()->size));
  }

  std::vector<vk::DescriptorSetLayout> descriptorSetLayouts;
  for (uint32_t sid = 0; sid < mShaderPack->getResources().size(); ++sid) {
    auto &set = mShaderPack->getResources().at(sid);
    switch (set.type) {
    case UniformBindingType::eRTScene:
      mSceneSetLayout = createSceneDescriptorSetLayout(
          device, set, mDesc.maxMeshes, mDesc.maxMaterials, mDesc.maxTextures, mDesc.maxPointSets);
      descriptorSetLayouts.push_back(mSceneSetLayout.get());
      break;
    case UniformBindingType::eRTOutput:
      mOutputSetLayout = createOutputDescriptorSetLayout(device, set);
      descriptorSetLayouts.push_back(mOutputSetLayout.get());
      break;
    case UniformBindingType::eRTCamera:
      mCameraSetLayout = createCameraDescriptorSetLayout(device, set);
      descriptorSetLayouts.push_back(mCameraSetLayout.get());
      break;
    default:
      throw std::runtime_error("unrecognized descriptor set");
    }
  }

  vk::PipelineLayoutCreateInfo layoutInfo({}, descriptorSetLayouts, pushConstantRanges);
  mPipelineLayout = device.createPipelineLayoutUnique(layoutInfo);

  std::vector<vk::PipelineShaderStageCreateInfo> shaderStages;
  std::vector<vk::RayTracingShaderGroupCreateInfoKHR> groups;

  vk::UniqueShaderModule rgenModule;
  std::vector<vk::UniqueShaderModule> missModules;
  std::vector<vk::UniqueShaderModule> rcHitModules;
  std::vector<vk::UniqueShaderModule> raHitModules;
  std::vector<vk::UniqueShaderModule> intersectModules;

  int count = 0;
  std::map<RayTracingStageParser *, int> parserIndex;

  parserIndex[mShaderPack->getRaygenStageParser()] = count++;
  rgenModule = device.createShaderModuleUnique(
      vk::ShaderModuleCreateInfo({}, mShaderPack->getRaygenStageParser()->getCode()));
  shaderStages.push_back(vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eRaygenKHR,
                                                           rgenModule.get(), "main"));
  for (auto &p : mShaderPack->getMissGroupParsers()) {
    parserIndex[p.miss.get()] = count++;
    missModules.push_back(
        device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, p.miss->getCode())));
    shaderStages.push_back(vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eMissKHR,
                                                             missModules.back().get(), "main"));
  }
  for (auto &p : mShaderPack->getHitGroupParsers()) {
    if (p.closest) {
      parserIndex[p.closest.get()] = count++;
      rcHitModules.push_back(
          device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, p.closest->getCode())));
      shaderStages.push_back(vk::PipelineShaderStageCreateInfo(
          {}, vk::ShaderStageFlagBits::eClosestHitKHR, rcHitModules.back().get(), "main"));
    }
  }
  for (auto &p : mShaderPack->getHitGroupParsers()) {
    if (p.any) {
      parserIndex[p.any.get()] = count++;
      raHitModules.push_back(
          device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, p.any->getCode())));
      shaderStages.push_back(vk::PipelineShaderStageCreateInfo(
          {}, vk::ShaderStageFlagBits::eAnyHitKHR, raHitModules.back().get(), "main"));
    }
  }
  for (auto &p : mShaderPack->getHitGroupParsers()) {
    if (p.intersect) {
      parserIndex[p.intersect.get()] = count++;
      intersectModules.push_back(
          device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, p.intersect->getCode())));
      shaderStages.push_back(vk::PipelineShaderStageCreateInfo(
          {}, vk::ShaderStageFlagBits::eIntersectionKHR, intersectModules.back().get(), "main"));
    }
  }

  // gen group
  groups.push_back(
      vk::RayTracingShaderGroupCreateInfoKHR(vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                             parserIndex.at(mShaderPack->getRaygenStageParser()),
                                             VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                             VK_SHADER_UNUSED_KHR)); // raygen

  // miss group
  for (auto &parser : mShaderPack->getMissGroupParsers()) {
    groups.push_back(vk::RayTracingShaderGroupCreateInfoKHR(
        vk::RayTracingShaderGroupTypeKHR::eGeneral, parserIndex.at(parser.miss.get()),
        VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
        VK_SHADER_UNUSED_KHR)); // camera miss
  }

  // hit group
  for (auto &parser : mShaderPack->getHitGroupParsers()) {
    uint32_t closest = VK_SHADER_UNUSED_KHR;
    uint32_t any = VK_SHADER_UNUSED_KHR;
    uint32_t intersect = VK_SHADER_UNUSED_KHR;

    auto groupType = parser.name == "camera"
                         ? vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup
                         : vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup;

    if (parser.closest) {
      closest = parserIndex.at(parser.closest.get());
    }

    if (parser.any) {
      any = parserIndex.at(parser.any.get());
    }

    if (parser.intersect) {
      intersect = parserIndex.at(parser.intersect.get());
    }

    groups.push_back(vk::RayTracingShaderGroupCreateInfoKHR(groupType, VK_SHADER_UNUSED_KHR,
                                                            closest, any, intersect));
  }

  vk::RayTracingPipelineCreateInfoKHR createInfo({}, shaderStages, groups, 30, nullptr, nullptr,
                                                 nullptr, mPipelineLayout.get(), {}, 0);

  auto result = device.createRayTracingPipelineKHRUnique({}, nullptr, createInfo);
  if (result.result != vk::Result::eSuccess) {
    throw std::runtime_error("failed to create ray tracing pipeline");
  }
  mPipeline = std::move(result.value);

  // post processing piepline
  for (auto &parser : mShaderPack->getPostprocessingParsers()) {
    if (parser->getResources().size() != 1) {
      throw std::runtime_error("invalid postprocessing shader");
    }
    auto &[sid, set] = *parser->getResources().begin();

    std::vector<vk::DescriptorSetLayoutBinding> bindings;
    for (uint32_t bid = 0; bid < set.bindings.size(); ++bid) {
      bindings.push_back(vk::DescriptorSetLayoutBinding(bid, set.bindings.at(bid).type, 1,
                                                        vk::ShaderStageFlagBits::eCompute));
    }
    auto descriptorSetLayout =
        device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, bindings));
    auto pipelineLayout = device.createPipelineLayoutUnique(
        vk::PipelineLayoutCreateInfo({}, descriptorSetLayout.get(), pushConstantRanges));

    auto shaderModule =
        device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, parser->getCode()));
    auto shaderStageInfo = vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eCompute,
                                                             shaderModule.get(), "main");

    auto pipeline =
        device
            .createComputePipelineUnique(
                {}, vk::ComputePipelineCreateInfo({}, shaderStageInfo, pipelineLayout.get()))
            .value;

    mPostprocessingSetLayouts.push_back(std::move(descriptorSetLayout));
    mPostprocessingPipelineLayouts.push_back(std::move(pipelineLayout));
    mPostprocessingPipelines.push_back(std::move(pipeline));
  }
}

void RayTracingShaderPackInstance::initSBT() {
  auto context = core::Context::Get();

  initPipeline();

  // create shader binding table
  auto properties = context->getPhysicalDevice()
                        .getProperties2<vk::PhysicalDeviceProperties2,
                                        vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
  auto pipelineProperties = properties.get<vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();

  uint32_t genCount = 1;
  uint32_t missCount = mShaderPack->getMissGroupParsers().size();
  uint32_t hitCount = mShaderPack->getHitGroupParsers().size();

  uint32_t handleCount = genCount + missCount + hitCount;
  uint32_t handleSize = pipelineProperties.shaderGroupHandleSize;
  uint32_t dataSize = handleCount * handleSize;
  std::vector<uint8_t> shaderHandleStorage(dataSize);
  if (context->getDevice().getRayTracingShaderGroupHandlesKHR(
          mPipeline.get(), 0, handleCount, dataSize, shaderHandleStorage.data()) !=
      vk::Result::eSuccess) {
    throw std::runtime_error("failed to get ray tracing shader group handles");
  }

  uint32_t handleSizeAligned = alignUp(pipelineProperties.shaderGroupHandleSize,
                                       pipelineProperties.shaderGroupHandleAlignment);

  uint32_t rgenStride = alignUp(handleSizeAligned, pipelineProperties.shaderGroupBaseAlignment);
  uint32_t rgenSize = rgenStride;
  uint32_t missStride = handleSizeAligned;
  uint32_t missSize =
      alignUp(missCount * handleSizeAligned, pipelineProperties.shaderGroupBaseAlignment);
  uint32_t hitStride = handleSizeAligned;
  uint32_t hitSize =
      alignUp(hitCount * handleSizeAligned, pipelineProperties.shaderGroupBaseAlignment);

  vk::DeviceSize sbtSize = rgenSize + missSize + hitSize; // there is no call region
  mSBTBuffer = core::Buffer::Create(
      sbtSize,
      vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eShaderDeviceAddress |
          vk::BufferUsageFlagBits::eShaderBindingTableKHR | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_CPU_TO_GPU, VmaAllocationCreateFlags{}, false,
      context->getAllocator().getRTPool());

  vk::DeviceSize rgenAddress = mSBTBuffer->getAddress();
  vk::DeviceSize missAddress = rgenAddress + rgenSize;
  vk::DeviceSize hitAddress = missAddress + missSize;

  mRgenRegion = vk::StridedDeviceAddressRegionKHR(rgenAddress, rgenStride, rgenSize);
  mMissRegion = vk::StridedDeviceAddressRegionKHR(missAddress, missStride, missSize);
  mHitRegion = vk::StridedDeviceAddressRegionKHR(hitAddress, hitStride, hitSize);

  uint32_t handleOffset = 0;
  uint32_t bufferOffset = 0;
  mSBTBuffer->upload(shaderHandleStorage.data() + handleOffset, rgenSize, bufferOffset);
  handleOffset += handleSize;
  bufferOffset = rgenSize;

  for (uint32_t i = 0; i < missCount; ++i) {
    mSBTBuffer->upload(shaderHandleStorage.data() + handleOffset, handleSize, bufferOffset);
    handleOffset += handleSize;
    bufferOffset += missStride;
  }

  for (uint32_t i = 0; i < hitCount; ++i) {
    mSBTBuffer->upload(shaderHandleStorage.data() + handleOffset, handleSize, bufferOffset);
    handleOffset += handleSize;
    bufferOffset += hitStride;
  }
}

vk::PipelineLayout RayTracingShaderPackInstance::getPipelineLayout() {
  if (mPipelineLayout) {
    return mPipelineLayout.get();
  }
  initPipeline();
  return mPipelineLayout.get();
}

vk::Pipeline RayTracingShaderPackInstance::getPipeline() {
  if (mPipeline) {
    return mPipeline.get();
  }
  initPipeline();
  return mPipeline.get();
}

core::Buffer &RayTracingShaderPackInstance::getShaderBindingTable() {
  if (mSBTBuffer) {
    return *mSBTBuffer;
  }
  initSBT();
  return *mSBTBuffer;
}

vk::StridedDeviceAddressRegionKHR const &RayTracingShaderPackInstance::getRgenRegion() {
  if (!mSBTBuffer) {
    initSBT();
  }
  return mRgenRegion;
}
vk::StridedDeviceAddressRegionKHR const &RayTracingShaderPackInstance::getMissRegion() {
  if (!mSBTBuffer) {
    initSBT();
  }
  return mMissRegion;
}
vk::StridedDeviceAddressRegionKHR const &RayTracingShaderPackInstance::getHitRegion() {
  if (!mSBTBuffer) {
    initSBT();
  }
  return mHitRegion;
}
vk::StridedDeviceAddressRegionKHR const &RayTracingShaderPackInstance::getCallRegion() {
  if (!mSBTBuffer) {
    initSBT();
  }
  return mCallRegion;
}

vk::DescriptorSetLayout RayTracingShaderPackInstance::getOutputSetLayout() {
  if (!mOutputSetLayout) {
    initPipeline();
  }
  return mOutputSetLayout.get();
}

vk::DescriptorSetLayout RayTracingShaderPackInstance::getSceneSetLayout() {
  if (!mSceneSetLayout) {
    initPipeline();
  }
  return mSceneSetLayout.get();
}

vk::DescriptorSetLayout RayTracingShaderPackInstance::getCameraSetLayout() {
  if (!mCameraSetLayout) {
    initPipeline();
  }
  return mCameraSetLayout.get();
}

} // namespace shader
} // namespace svulkan2