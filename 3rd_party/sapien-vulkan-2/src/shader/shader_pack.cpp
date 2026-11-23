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
#include "svulkan2/shader/shader_pack.h"
#include "../common/logger.h"
#include "svulkan2/common/fs.h"
#include "svulkan2/shader/deferred.h"
#include "svulkan2/shader/gbuffer.h"
#include "svulkan2/shader/primitive.h"
#include "svulkan2/shader/primitive_shadow.h"
#include "svulkan2/shader/shadow.h"
#include <filesystem>
#include <set>

namespace fs = std::filesystem;
namespace svulkan2 {
namespace shader {

ShaderPack::ShaderPack(std::string const &dirname) {
  fs::path path(dirname);
  check_dir_required(path, "[shader pack] " + dirname + " is not a directory");
  check_file_required(path / "gbuffer.vert", "[shader pack] gbuffer.vert is required");
  check_file_required(path / "gbuffer.frag", "[shader pack] gbuffer.frag is required");

  // GLSL compiler starts

  std::vector<std::future<void>> futures;

  // shadow pass
  if (check_file(path / "shadow.vert")) {
    mShadowPass = std::make_shared<ShadowPassParser>();
    mShadowPass->setName("Shadow");
    futures.push_back(mShadowPass->loadGLSLFilesAsync((path / "shadow.vert").string(), ""));
  }

  // point shadow pass
  if (check_file(path / "shadow_point.vert")) {
    mPointShadowPass = std::make_shared<PointShadowParser>();
    mPointShadowPass->setName("PointShadow");

    auto vsFile = path / "shadow_point.vert";
    auto fsFile = path / "shadow_point.frag";
    auto gsFile = path / "shadow_point.geom";
    fsFile = check_file(fsFile) ? fsFile : "";
    gsFile = check_file(gsFile) ? gsFile : "";
    futures.push_back(
        mPointShadowPass->loadGLSLFilesAsync(vsFile.string(), fsFile.string(), gsFile.string()));
  }

  mHasDeferred = check_file(path / "deferred.vert") && check_file(path / "deferred.frag");

  mNonShadowPasses = {};
  mGbufferPasses = {};
  mPointPasses = {};
  mLinePasses = {};

  int numGbufferPasses = 0;
  int numPointPasses = 0;
  for (const auto &entry : fs::directory_iterator(path)) {
    std::string filename = entry.path().filename().string();
    if (filename.starts_with("gbuffer") && filename.ends_with(".frag")) {
      numGbufferPasses++;
    }
    if (filename.starts_with("point") && filename.ends_with(".frag")) {
      numPointPasses++;
    }
  }

  // first gbuffer pass
  std::shared_ptr<GbufferPassParser> firstGbufferPass = std::make_shared<GbufferPassParser>();
  {
    firstGbufferPass->setName("Gbuffer");
    mNonShadowPasses.push_back(firstGbufferPass);
    mGbufferPasses.push_back(firstGbufferPass);
    firstGbufferPass->setIndex(mNonShadowPasses.size() - 1);
    std::string vsFile = (path / ("gbuffer.vert")).string();
    std::string fsFile = (path / ("gbuffer.frag")).string();
    futures.push_back(firstGbufferPass->loadGLSLFilesAsync(vsFile, fsFile));
  }

  // ao pass
  {
    std::string vsFile = (path / "ao.vert").string();
    std::string fsFile = (path / "ao.frag").string();
    if (check_file(vsFile) && check_file(fsFile)) {
      auto aoPass = std::make_shared<DeferredPassParser>();
      aoPass->setName("AO");
      mNonShadowPasses.push_back(aoPass);
      aoPass->setIndex(mNonShadowPasses.size() - 1);
      futures.push_back(aoPass->loadGLSLFilesAsync(vsFile, fsFile));
    }
  }

  // deferred pass
  std::string vsFile = (path / "deferred.vert").string();
  std::string fsFile = (path / "deferred.frag").string();
  if (check_file(vsFile) && check_file(fsFile)) {
    auto deferredPass = std::make_shared<DeferredPassParser>();
    deferredPass->setName("Deferred");
    mNonShadowPasses.push_back(deferredPass);
    deferredPass->setIndex(mNonShadowPasses.size() - 1);
    futures.push_back(deferredPass->loadGLSLFilesAsync(vsFile, fsFile));
  }

  // gbuffer passes
  for (int i = 1; i < numGbufferPasses; ++i) {
    std::string suffix = std::to_string(i);
    auto gbufferPass = std::make_shared<GbufferPassParser>();

    gbufferPass->setName("Gbuffer" + suffix);
    mNonShadowPasses.push_back(gbufferPass);
    mGbufferPasses.push_back(gbufferPass);
    gbufferPass->setIndex(mNonShadowPasses.size() - 1);

    std::string vsFile = (path / ("gbuffer" + suffix + ".vert")).string();
    std::string fsFile = (path / ("gbuffer" + suffix + ".frag")).string();
    futures.push_back(gbufferPass->loadGLSLFilesAsync(vsFile, fsFile));
  }

  // point pass
  std::shared_ptr<PointPassParser> firstPointPass{};
  for (int i = 0; i < numPointPasses; ++i) {
    std::string suffix = i == 0 ? "" : std::to_string(i);
    auto pointPass = std::make_shared<PointPassParser>();
    if (i == 0) {
      firstPointPass = pointPass;
    }
    pointPass->setName("Point" + suffix);
    mNonShadowPasses.push_back(pointPass);
    mPointPasses.push_back(pointPass);
    pointPass->setIndex(mNonShadowPasses.size() - 1);

    std::string vsFile = (path / ("point" + suffix + ".vert")).string();
    std::string fsFile = (path / ("point" + suffix + ".frag")).string();
    std::string gsFile = (path / ("point" + suffix + ".geom")).string();
    gsFile = check_file(gsFile) ? gsFile : "";

    futures.push_back(pointPass->loadGLSLFilesAsync(vsFile, fsFile, gsFile));
  }

  std::shared_ptr<LinePassParser> linePass{};
  // line pass
  {
    std::string vsFile = (path / "line.vert").string();
    std::string fsFile = (path / "line.frag").string();
    if (check_file(vsFile) && check_file(fsFile)) {
      linePass = std::make_shared<LinePassParser>();
      linePass->setName("Line");
      mNonShadowPasses.push_back(linePass);
      mLinePasses.push_back(linePass);
      linePass->setIndex(mNonShadowPasses.size() - 1);
      futures.push_back(linePass->loadGLSLFilesAsync(vsFile, fsFile));
    }
  }

  int numCompositePasses = 0;
  for (const auto &entry : fs::directory_iterator(path)) {
    std::string filename = entry.path().filename().string();
    if (filename.starts_with("composite") && filename.ends_with(".frag"))
      numCompositePasses++;
  }

  vsFile = (path / "composite.vert").string();
  for (int i = 0; i < numCompositePasses; i++) {
    fsFile = (path / ("composite" + std::to_string(i) + ".frag")).string();
    auto compositePass = std::make_shared<DeferredPassParser>();
    compositePass->setName("Composite" + std::to_string(i));
    mNonShadowPasses.push_back(compositePass);
    compositePass->setIndex(mNonShadowPasses.size() - 1);
    futures.push_back(compositePass->loadGLSLFilesAsync(vsFile, fsFile));
  }
  for (auto &f : futures) {
    f.get();
  }
  // GLSL compiler ends

  mShaderInputLayouts = generateShaderLayouts();
  updateMaxLightCount();
}

void ShaderPack::updateMaxLightCount() {
  {
    auto set = mShaderInputLayouts->sceneSetDescription;

    {
      // process scene buffer
      if (mShaderInputLayouts->sceneBufferLayout->elements.contains("pointLights")) {
        mMaxPointLightCount =
            mShaderInputLayouts->sceneBufferLayout->elements.at("pointLights").array.at(0);
      }
      if (mShaderInputLayouts->sceneBufferLayout->elements.contains("directionalLights")) {
        mMaxDirectionalLightCount =
            mShaderInputLayouts->sceneBufferLayout->elements.at("directionalLights").array.at(0);
      }
      if (mShaderInputLayouts->sceneBufferLayout->elements.contains("spotLights")) {
        mMaxSpotLightCount =
            mShaderInputLayouts->sceneBufferLayout->elements.at("spotLights").array.at(0);
      }
      if (mShaderInputLayouts->sceneBufferLayout->elements.contains("texturedLights")) {
        mMaxTexturedLightCount =
            mShaderInputLayouts->sceneBufferLayout->elements.at("texturedLights").array.at(0);
      }
    }

    {
      // process shadow buffer
      auto it = std::find_if(set.bindings.begin(), set.bindings.end(),
                             [](std::pair<uint32_t, DescriptorSetDescription::Binding> const &p) {
                               return p.second.name == "ShadowBuffer";
                             });
      mMaxPointShadowCount = 0;
      mMaxDirectionalShadowCount = 0;
      mMaxSpotShadowCount = 0;
      mMaxTexturedShadowCount = 0;
      if (it != set.bindings.end()) {
        auto shadowBuffer = set.buffers.at(it->second.arrayIndex);
        if (shadowBuffer->elements.contains("pointLightBuffers")) {
          mMaxPointShadowCount = shadowBuffer->elements.at("pointLightBuffers").array.at(0) / 6;
        }
        if (shadowBuffer->elements.contains("directionalLightBuffers")) {
          mMaxDirectionalShadowCount =
              shadowBuffer->elements.at("directionalLightBuffers").array.at(0);
        }
        if (shadowBuffer->elements.contains("spotLightBuffers")) {
          mMaxSpotShadowCount = shadowBuffer->elements.at("spotLightBuffers").array.at(0);
        }
        if (shadowBuffer->elements.contains("texturedLightBuffers")) {
          mMaxTexturedShadowCount = shadowBuffer->elements.at("texturedLightBuffers").array.at(0);
        }
      }

      if (mMaxPointShadowCount > mMaxPointLightCount) {
        throw std::runtime_error(
            "ShadowBuffer pointLightBuffers size must not exceed SceneBuffer pointLights");
      }
      if (mMaxDirectionalShadowCount > mMaxDirectionalLightCount) {
        throw std::runtime_error("ShadowBuffer directionalLightBuffers size must not exceed "
                                 "SceneBuffer directionalLights");
      }
      if (mMaxSpotShadowCount > mMaxSpotLightCount) {
        throw std::runtime_error(
            "ShadowBuffer spotLightBuffers size must not exceed SceneBuffer spotLights");
      }
      if (mMaxTexturedShadowCount > mMaxTexturedLightCount) {
        throw std::runtime_error(
            "ShadowBuffer texturedLightBuffers size must not exceed SceneBuffer texturedLights");
      }
    }

    // verify shadow map texture shapes
    {
      uint32_t maxPointTexCount = 0;
      uint32_t maxDirectionalTexCount = 0;
      uint32_t maxSpotTexCount = 0;
      uint32_t maxTexturedTexCount = 0;

      // process shadow textures
      auto it = std::find_if(set.bindings.begin(), set.bindings.end(),
                             [](std::pair<uint32_t, DescriptorSetDescription::Binding> const &p) {
                               return p.second.name == "samplerPointLightDepths";
                             });
      if (it != set.bindings.end()) {
        maxPointTexCount = it->second.arraySize;
      }

      it = std::find_if(set.bindings.begin(), set.bindings.end(),
                        [](std::pair<uint32_t, DescriptorSetDescription::Binding> const &p) {
                          return p.second.name == "samplerDirectionalLightDepths";
                        });
      if (it != set.bindings.end()) {
        maxDirectionalTexCount = it->second.arraySize;
      }

      it = std::find_if(set.bindings.begin(), set.bindings.end(),
                        [](std::pair<uint32_t, DescriptorSetDescription::Binding> const &p) {
                          return p.second.name == "samplerSpotLightDepths";
                        });
      if (it != set.bindings.end()) {
        maxSpotTexCount = it->second.arraySize;
      }

      it = std::find_if(set.bindings.begin(), set.bindings.end(),
                        [](std::pair<uint32_t, DescriptorSetDescription::Binding> const &p) {
                          return p.second.name == "samplerTexturedLightDepths";
                        });
      if (it != set.bindings.end()) {
        maxTexturedTexCount = it->second.arraySize;
      }

      if (maxPointTexCount != mMaxPointShadowCount) {
        throw std::runtime_error("samplerPointLightDepths and ShadowBuffer pointLightBuffers "
                                 "should have matching sizes (N vs 6N)");
      }

      if (maxDirectionalTexCount != mMaxDirectionalShadowCount) {
        throw std::runtime_error("samplerDirectionalLightDepths and ShadowBuffer "
                                 "directionalLightBuffers should have the same size");
      }

      if (maxSpotTexCount != mMaxSpotShadowCount) {
        throw std::runtime_error("samplerSpotLightDepths and ShadowBuffer "
                                 "spotLightBuffers should have the same size");
      }

      if (maxTexturedTexCount != mMaxTexturedShadowCount) {
        throw std::runtime_error("samplerTexturedLightDepths and ShadowBuffer "
                                 "texturedLightBuffers should have the same size");
      }
    }
  }
}

std::shared_ptr<ShaderConfig> ShaderPack::generateShaderLayouts() const {
  DescriptorSetDescription cameraSetDesc;
  DescriptorSetDescription objectSetDesc;
  DescriptorSetDescription sceneSetDesc;
  DescriptorSetDescription lightSetDesc;

  auto layouts = std::make_shared<ShaderConfig>();

  ASSERT(mGbufferPasses.size() > 0, "There must be at least 1 gbuffer pass in a shader pack.");
  layouts->vertexLayout = mGbufferPasses.at(0)->getVertexInputLayout();

  if (mLinePasses.size()) {
    layouts->primitiveVertexLayout = mLinePasses.at(0)->getVertexInputLayout();
  }

  if (mPointPasses.size()) {
    if (layouts->primitiveVertexLayout) {
      ASSERT(*layouts->primitiveVertexLayout == *mPointPasses.at(0)->getVertexInputLayout(),
             "All primitive passes must share the same vertex layout");
    } else {
      layouts->primitiveVertexLayout = mPointPasses.at(0)->getVertexInputLayout();
    }
  }

  std::vector<std::shared_ptr<BaseParser>> passes = mNonShadowPasses;
  if (mShadowPass) {
    passes.push_back(mShadowPass);
  }
  if (mPointShadowPass) {
    passes.push_back(mPointShadowPass);
  }

  for (auto &pass : passes) {
    auto descs = pass->getDescriptorSetDescriptions();
    for (auto &desc : descs) {
      switch (desc.type) {
      case UniformBindingType::eCamera:
        if (cameraSetDesc.type == UniformBindingType::eUnknown) {
          cameraSetDesc = desc;
        } else {
          cameraSetDesc = cameraSetDesc.merge(desc);
        }
        break;
      case UniformBindingType::eObject:
        if (objectSetDesc.type == UniformBindingType::eUnknown) {
          objectSetDesc = desc;
        } else {
          objectSetDesc = objectSetDesc.merge(desc);
        }
        break;
      case UniformBindingType::eScene:
        if (sceneSetDesc.type == UniformBindingType::eUnknown) {
          sceneSetDesc = desc;
        } else {
          sceneSetDesc = sceneSetDesc.merge(desc);
        }
        break;
      case UniformBindingType::eLight:
        if (lightSetDesc.type == UniformBindingType::eUnknown) {
          lightSetDesc = desc;
        } else {
          lightSetDesc = lightSetDesc.merge(desc);
        }
        break;
      case UniformBindingType::eMaterial:
      case UniformBindingType::eTextures:
        break;
      case UniformBindingType::eNone:
      case UniformBindingType::eUnknown:
      default:
        throw std::runtime_error("invalid descriptor set");
      }
    }
  }

  layouts->cameraBufferLayout = cameraSetDesc.buffers.at(cameraSetDesc.bindings.at(0).arrayIndex);
  layouts->objectDataBufferLayout =
      objectSetDesc.buffers.at(objectSetDesc.bindings.at(1).arrayIndex);
  layouts->sceneBufferLayout = sceneSetDesc.buffers.at(sceneSetDesc.bindings.at(0).arrayIndex);

  if (mShadowPass) {
    layouts->lightBufferLayout = lightSetDesc.buffers.at(lightSetDesc.bindings.at(0).arrayIndex);
    for (auto &binding : sceneSetDesc.bindings) {
      if (binding.second.name == "ShadowBuffer") {
        layouts->shadowBufferLayout = sceneSetDesc.buffers.at(binding.second.arrayIndex);
        break;
      }
    }
    if (!layouts->shadowBufferLayout) {
      throw std::runtime_error("Scene must declare ShadowBuffer");
    }
  }

  layouts->sceneSetDescription = sceneSetDesc;
  layouts->cameraSetDescription = cameraSetDesc;
  layouts->objectSetDescription = objectSetDesc;
  layouts->lightSetDescription = lightSetDesc;

  return layouts;
}

} // namespace shader
} // namespace svulkan2