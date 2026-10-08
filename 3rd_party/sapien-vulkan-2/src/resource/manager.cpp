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
#include "svulkan2/resource/manager.h"
#include "svulkan2/common/image.h"
#include "svulkan2/core/context.h"
#include "svulkan2/shader/compute.h"
#include "svulkan2/shader/rt.h"
#include <filesystem>
#include <random>

namespace fs = std::filesystem;

namespace svulkan2 {
namespace resource {

SVResourceManager::SVResourceManager() {
  std::vector<char> rawData = toRawBytes(std::vector<uint8_t>{255, 255, 255, 255});

  mDefaultTexture1D = SVTexture::FromRawData(1, 1, 1, vk::Format::eR8G8B8A8Unorm, rawData, 1);
  mDefaultTexture2D = SVTexture::FromRawData(1, 1, 1, vk::Format::eR8G8B8A8Unorm, rawData, 2);
  mDefaultTexture3D = SVTexture::FromRawData(1, 1, 1, vk::Format::eR8G8B8A8Unorm, rawData, 3);

  mDefaultCubemap = SVCubemap::FromData(1, vk::Format::eR8G8B8A8Unorm,
                                        {
                                            std::vector<char>{0, 0, 0, 0},
                                            std::vector<char>{0, 0, 0, 0},
                                            std::vector<char>{0, 0, 0, 0},
                                            std::vector<char>{0, 0, 0, 0},
                                            std::vector<char>{0, 0, 0, 0},
                                            std::vector<char>{0, 0, 0, 0},
                                        });
}

std::shared_ptr<shader::ShaderPack>
SVResourceManager::CreateShaderPack(std::string const &dirname) {
  std::lock_guard<std::mutex> lock(mShaderPackLock);
  auto dir = fs::canonical(dirname);
  std::string key = dir.string();

  auto it = mShaderPackRegistry.find(key);
  if (it != mShaderPackRegistry.end()) {
    return it->second;
  }

  if (!fs::is_directory(dir)) {
    throw std::runtime_error("invalid shader pack directory: " + dirname);
  }
  auto shaderPack = std::make_shared<shader::ShaderPack>(dirname);
  mShaderPackRegistry[key] = shaderPack;

  if (shaderPack->getShaderInputLayouts()->vertexLayout) {
    setVertexLayout(shaderPack->getShaderInputLayouts()->vertexLayout);
  }

  if (shaderPack->getShaderInputLayouts()->primitiveVertexLayout) {
    setPrimitiveVertexLayout(shaderPack->getShaderInputLayouts()->primitiveVertexLayout);
  }

  return shaderPack;
}

std::shared_ptr<shader::ShaderPackInstance>
SVResourceManager::CreateShaderPackInstance(shader::ShaderPackInstanceDesc const &desc) {
  std::lock_guard<std::mutex> lock(mShaderPackInstanceLock);
  auto it = mShaderPackInstanceRegistry.find(desc.config->shaderDir);
  if (it != mShaderPackInstanceRegistry.end()) {
    for (auto inst : it->second) {
      if (inst->getDesc() == desc) {
        return inst;
      }
    }
  }
  auto inst = std::make_shared<shader::ShaderPackInstance>(desc);
  mShaderPackInstanceRegistry[desc.config->shaderDir].push_back(inst);
  return inst;
}

std::shared_ptr<shader::RayTracingShaderPack>
SVResourceManager::CreateRTShaderPack(std::string const &dirname) {
  std::lock_guard<std::mutex> lock(mShaderPackLock);
  auto dir = fs::canonical(dirname);
  std::string key = dir.string();

  auto it = mRTShaderPackRegistry.find(key);
  if (it != mRTShaderPackRegistry.end()) {
    return it->second;
  }

  if (!fs::is_directory(dir)) {
    throw std::runtime_error("invalid shader pack directory: " + dirname);
  }
  auto shaderPack = std::make_shared<shader::RayTracingShaderPack>(dirname);
  mRTShaderPackRegistry[key] = shaderPack;

  setVertexLayout(shaderPack->computeCompatibleInputVertexLayout());
  if (auto layout = shaderPack->computePrimitiveLayout()) {
    setPrimitiveVertexLayout(layout);
  }

  return shaderPack;
}

std::shared_ptr<SVImage> SVResourceManager::CreateImageFromFile(std::string const &filename,
                                                                uint32_t mipLevels,
                                                                uint32_t desiredChannels) {
  std::lock_guard<std::mutex> lock(mCreateLock);
  std::string path = fs::canonical(filename).string();
  SVImageDescription desc = {.source = SVImageDescription::SourceType::eFILE,
                             .filenames = {path},
                             .desiredChannels = desiredChannels,
                             .mipLevels = mipLevels};

  auto it = mImageRegistry.find(path);
  if (it != mImageRegistry.end()) {
    for (auto &img : it->second) {
      if (img->getDescription() == desc) {
        return img;
      }
    }
  }
  auto img = SVImage::FromFile({path}, mipLevels, desiredChannels);
  mImageRegistry[path].push_back(img);
  return img;
}

std::shared_ptr<SVTexture> SVResourceManager::CreateTextureFromFile(
    std::string const &filename, uint32_t mipLevels, vk::Filter magFilter, vk::Filter minFilter,
    vk::SamplerAddressMode addressModeU, vk::SamplerAddressMode addressModeV, bool srgb,
    uint32_t desiredChannels) {
  std::lock_guard<std::mutex> lock(mCreateLock);
  std::string path = fs::canonical(filename).string();

  SVTextureDescription desc = {.source = SVTextureDescription::SourceType::eFILE,
                               .filename = path,
                               .desiredChannels = desiredChannels,
                               .mipLevels = mipLevels,
                               .magFilter = magFilter,
                               .minFilter = minFilter,
                               .addressModeU = addressModeU,
                               .addressModeV = addressModeV,
                               .srgb = srgb};

  auto it = mTextureRegistry.find(path);
  if (it != mTextureRegistry.end()) {
    for (auto &tex : it->second) {
      if (tex->getDescription() == desc) {
        return tex;
      }
    }
  }
  auto tex = SVTexture::FromFile(path, mipLevels, magFilter, minFilter, addressModeU, addressModeV,
                                 srgb, desiredChannels);
  mTextureRegistry[path].push_back(tex);
  return tex;
}

std::shared_ptr<SVTexture> SVResourceManager::CreateTextureFromRawData(
    uint32_t width, uint32_t height, uint32_t depth, vk::Format format,
    std::vector<char> const &data, int dim, uint32_t mipLevels, vk::Filter magFilter,
    vk::Filter minFilter, vk::SamplerAddressMode addressModeU, vk::SamplerAddressMode addressModeV,
    vk::SamplerAddressMode addressModeW, bool srgb) {
  auto tex = SVTexture::FromRawData(width, height, depth, format, data, dim, mipLevels, magFilter,
                                    minFilter, addressModeU, addressModeV, addressModeW, srgb);
  return tex;
}

std::shared_ptr<SVCubemap>
SVResourceManager::CreateCubemapFromFile(std::string const &filename, uint32_t mipLevels,
                                         vk::Filter magFilter, vk::Filter minFilter, bool srgb) {
  std::lock_guard<std::mutex> lock(mCreateLock);
  auto source = (filename.ends_with(".ktx") || filename.ends_with(".KTX"))
                    ? SVCubemapDescription::SourceType::eKTX
                    : SVCubemapDescription::SourceType::eLATLONG;
  SVCubemapDescription desc = {.source = source,
                               .filenames = {fs::canonical(filename).string(), "", "", "", "", ""},
                               .mipLevels = mipLevels,
                               .magFilter = magFilter,
                               .minFilter = minFilter,
                               .srgb = srgb};
  auto cubemap =
      SVCubemap::FromFile(desc.filenames[0], desc.mipLevels, desc.magFilter, desc.minFilter, srgb);
  auto it = mCubemapRegistry.find(desc.filenames[0]);
  if (it != mCubemapRegistry.end()) {
    for (auto &tex : it->second) {
      if (tex->getDescription() == desc) {
        return tex;
      }
    }
  }
  mCubemapRegistry[desc.filenames[0]].push_back(cubemap);
  return cubemap;
}

std::shared_ptr<SVCubemap>
SVResourceManager::CreateCubemapFromFiles(std::array<std::string, 6> const &filenames,
                                          uint32_t mipLevels, vk::Filter magFilter,
                                          vk::Filter minFilter, bool srgb) {
  std::lock_guard<std::mutex> lock(mCreateLock);
  SVCubemapDescription desc = {
      .source = SVCubemapDescription::SourceType::eFACES,
      .filenames = {fs::canonical(filenames[0]).string(), fs::canonical(filenames[1]).string(),
                    fs::canonical(filenames[2]).string(), fs::canonical(filenames[3]).string(),
                    fs::canonical(filenames[4]).string(), fs::canonical(filenames[5]).string()},
      .mipLevels = mipLevels,
      .magFilter = magFilter,
      .minFilter = minFilter,
      .srgb = srgb};
  auto cubemap =
      SVCubemap::FromFile(desc.filenames, desc.mipLevels, desc.magFilter, desc.minFilter, srgb);
  auto it = mCubemapRegistry.find(desc.filenames[0]);
  if (it != mCubemapRegistry.end()) {
    for (auto &tex : it->second) {
      if (tex->getDescription() == desc) {
        return tex;
      }
    }
  }
  mCubemapRegistry[desc.filenames[0]].push_back(cubemap);
  return cubemap;
}

std::shared_ptr<SVTexture> SVResourceManager::getDefaultBRDFLUT() {
  std::lock_guard<std::mutex> lock(mCreateLock);
  if (mDefaultBRDFLUT) {
    return mDefaultBRDFLUT;
  }
  return mDefaultBRDFLUT = generateBRDFLUT(512);
}

std::shared_ptr<SVTexture> SVResourceManager::generateBRDFLUT(uint32_t size) {
  auto context = core::Context::Get();
  if (!context->isVulkanAvailable()) {
    return nullptr;
  }
  auto image = shader::generateBRDFLUT(512);
  auto sampler = context->createSampler(vk::SamplerCreateInfo(
      {}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eNearest,
      vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge,
      vk::SamplerAddressMode::eClampToEdge, 0.f, false, 0.f, false, vk::CompareOp::eNever, 0.f,
      0.f, vk::BorderColor::eFloatOpaqueWhite));
  auto view = context->getDevice().createImageViewUnique(vk::ImageViewCreateInfo(
      {}, image->getVulkanImage(), vk::ImageViewType::e2D, image->getFormat(),
      vk::ComponentSwizzle::eIdentity,
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1)));
  return resource::SVTexture::FromImage(resource::SVImage::FromDeviceImage(std::move(image)),
                                        std::move(view), sampler);
}

// TODO: test this function
std::shared_ptr<SVTexture> SVResourceManager::CreateRandomTexture(std::string const &name) {
  if (mRandomTextureRegistry.find(name) != mRandomTextureRegistry.end()) {
    return mRandomTextureRegistry[name];
  }

  if (name.substr(0, 6) != "Random") {
    throw std::runtime_error("random texture name must starts with \"Random\"");
  }
  std::string n = name.substr(6);
  auto idx = n.find('x');
  if (idx == n.npos) {
    throw std::runtime_error("random texture name must starts with \"Random\"");
  }
  std::string firstNum = n.substr(0, idx);

  n = n.substr(idx + 1);
  idx = n.find('_');
  std::string secondNum = n;
  std::string seedNum = "0";
  if (idx != n.npos) {
    secondNum = n.substr(0, idx);
    seedNum = n.substr(idx + 1);
  }
  try {
    int width = std::stoi(firstNum);
    int height = std::stoi(secondNum);
    int seed = std::stoi(seedNum);
    if (width < 0 || height < 0) {
      throw std::runtime_error("random texture creation failed: invalid width, height, or seed.");
    }
    auto real_rand = std::bind(std::uniform_real_distribution<float>(0, 1), std::mt19937(seed));

    std::vector<float> data;
    data.reserve(width * height);
    for (int i = 0; i < width * height; ++i) {
      data.push_back(real_rand());
    }

    mRandomTextureRegistry[name] =
        SVTexture::FromRawData(width, height, 1, vk::Format::eR32Sfloat, toRawBytes(data), 2);
    return mRandomTextureRegistry[name];

  } catch (std::invalid_argument const &) {
    throw std::runtime_error("random texture creation failed: invalid width, height, or seed.");
  }
}

std::shared_ptr<SVModel> SVResourceManager::CreateModelFromFile(std::string const &filename) {
  std::lock_guard<std::mutex> lock(mCreateLock);
  std::string path = fs::canonical(filename).string();

  ModelDescription desc = {.source = ModelDescription::SourceType::eFILE, .filename = path};

  auto prototype = [&]() {
    auto it = mModelRegistry.find(path);
    if (it != mModelRegistry.end()) {
      for (auto &model : it->second) {
        if (model->getDescription() == desc) {
          return model;
        }
      }
    }
    auto model = SVModel::FromFile(path);
    mModelRegistry[path].push_back(model);
    return model;
  }();

  return SVModel::FromPrototype(prototype);
}

std::shared_ptr<SVMetallicMaterial>
SVResourceManager::createMetallicMaterial(glm::vec4 emission, glm::vec4 baseColor, float fresnel,
                                          float roughness, float metallic, float transparency,
                                          float ior) {
  return std::make_shared<SVMetallicMaterial>(emission, baseColor, fresnel, roughness, metallic,
                                              transparency, ior);
}

std::shared_ptr<resource::SVModel> SVResourceManager::createModel(
    std::vector<std::shared_ptr<resource::SVMesh>> const &meshes,
    std::vector<std::shared_ptr<resource::SVMaterial>> const &materials) {
  if (meshes.size() != materials.size()) {
    throw std::runtime_error("create model failed: meshes and materials must have the same size.");
  }
  std::vector<std::shared_ptr<resource::SVShape>> shapes;
  for (uint32_t i = 0; i < meshes.size(); ++i) {
    auto shape = std::make_shared<resource::SVShape>();
    shape->mesh = meshes[i];
    shape->material = materials[i];
    shapes.push_back(shape);
  }
  return resource::SVModel::FromData(shapes);
}

void SVResourceManager::setVertexLayout(std::shared_ptr<InputDataLayout> layout) {
  if (!mVertexLayout) {
    mVertexLayout = layout;
    return;
  }
  if (*mVertexLayout != *layout) {
    throw std::runtime_error(
        "All vertex layouts are required to be the same across all renderers");
  }
}

void SVResourceManager::setPrimitiveVertexLayout(std::shared_ptr<InputDataLayout> layout) {
  if (!mLineVertexLayout) {
    mLineVertexLayout = layout;
    return;
  }
  if (*mLineVertexLayout != *layout) {
    throw std::runtime_error("All primitive vertex layouts are required to be the "
                             "same even across renderers");
  }
}

void SVResourceManager::clearCachedResources(bool models, bool images, bool shaders) {
  std::lock_guard<std::mutex> lock(mCreateLock);
  if (models) {
    mModelRegistry.clear();
  }
  if (images) {
    mTextureRegistry.clear();
    mCubemapRegistry.clear();
    mImageRegistry.clear();
    mRandomTextureRegistry.clear();
  }
  if (shaders) {
    mShaderPackInstanceRegistry.clear();
    mShaderPackRegistry.clear();
  }
}

void SVResourceManager::releaseGPUResourcesUnsafe() {
  std::lock_guard<std::mutex> lock(mCreateLock);
  for (auto &it : mModelRegistry) {
    for (auto &m : it.second) {
      for (auto &s : m->getShapes()) {
        s->mesh->removeFromDevice();
        s->material->removeFromDevice();
      }
    }
  }
  for (auto &it : mTextureRegistry) {
    for (auto &t : it.second) {
      t->removeFromDevice();
    }
  }
  for (auto &it : mCubemapRegistry) {
    for (auto &t : it.second) {
      t->removeFromDevice();
    }
  }
  for (auto &it : mImageRegistry) {
    for (auto &t : it.second) {
      t->removeFromDevice();
    }
  }
}

} // namespace resource
} // namespace svulkan2