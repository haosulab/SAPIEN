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
#include "sapien/sapien_renderer/texture.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace sapien_renderer {

static SapienRenderTexture::FilterMode convertFilterMode(vk::Filter filter) {
  switch (filter) {
  case vk::Filter::eLinear:
    return SapienRenderTexture::FilterMode::eLINEAR;
  case vk::Filter::eNearest:
    return SapienRenderTexture::FilterMode::eNEAREST;
  default:
    throw std::runtime_error("unsupported filter mode");
  }
}

static vk::Filter convertFilterMode(SapienRenderTexture::FilterMode filterMode) {
  switch (filterMode) {
  case SapienRenderTexture::FilterMode::eNEAREST:
    return vk::Filter::eNearest;
  case SapienRenderTexture::FilterMode::eLINEAR:
    return vk::Filter::eLinear;
  default:
    // unreachable
    return vk::Filter::eLinear;
  }
}

static SapienRenderTexture::AddressMode convertAddressMode(vk::SamplerAddressMode address) {
  switch (address) {
  case vk::SamplerAddressMode::eRepeat:
    return SapienRenderTexture::AddressMode::eREPEAT;
  case vk::SamplerAddressMode::eClampToBorder:
    return SapienRenderTexture::AddressMode::eBORDER;
  case vk::SamplerAddressMode::eClampToEdge:
    return SapienRenderTexture::AddressMode::eEDGE;
  case vk::SamplerAddressMode::eMirroredRepeat:
    return SapienRenderTexture::AddressMode::eMIRROR;
  default:
    throw std::runtime_error("unsupported filter mode");
  }
}

static vk::SamplerAddressMode convertAddressMode(SapienRenderTexture::AddressMode addressMode) {
  switch (addressMode) {
  case SapienRenderTexture::AddressMode::eREPEAT:
    return vk::SamplerAddressMode::eRepeat;
  case SapienRenderTexture::AddressMode::eBORDER:
    return vk::SamplerAddressMode::eClampToBorder;
  case SapienRenderTexture::AddressMode::eEDGE:
    return vk::SamplerAddressMode::eClampToEdge;
  case SapienRenderTexture::AddressMode::eMIRROR:
    return vk::SamplerAddressMode::eMirroredRepeat;
  default:
    // unreachable
    return vk::SamplerAddressMode::eRepeat;
  }
}

SapienRenderTexture::SapienRenderTexture(uint32_t width, uint32_t height, uint32_t depth,
                                         vk::Format format, std::vector<char> const &rawData,
                                         int dim, uint32_t mipLevels, FilterMode filterMode,
                                         AddressMode addressMode, bool srgb) {
  mEngine = SapienRenderEngine::Get();
  vk::Filter filter = convertFilterMode(filterMode);
  vk::SamplerAddressMode address = convertAddressMode(addressMode);
  mTexture = svulkan2::resource::SVTexture::FromRawData(width, height, depth, format, rawData, dim,
                                                        mipLevels, filter, filter, address,
                                                        address, address, srgb);
}

vk::Format SapienRenderTexture::getFormat() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getFormat();
}

int SapienRenderTexture::getDim() const { return mTexture->getDescription().dim; }
int SapienRenderTexture::getWidth() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getWidth();
}
int SapienRenderTexture::getHeight() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getHeight();
}
int SapienRenderTexture::getDepth() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getDepth();
}
int SapienRenderTexture::getChannels() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getChannels();
}

SapienRenderTexture::FilterMode SapienRenderTexture::getMinFilterMode() const {
  return convertFilterMode(mTexture->getDescription().minFilter);
}
SapienRenderTexture::FilterMode SapienRenderTexture::getMagFilterMode() const {
  return convertFilterMode(mTexture->getDescription().magFilter);
}

SapienRenderTexture::AddressMode SapienRenderTexture::getAddressModeU() const {
  return convertAddressMode(mTexture->getDescription().addressModeU);
}
SapienRenderTexture::AddressMode SapienRenderTexture::getAddressModeV() const {
  return convertAddressMode(mTexture->getDescription().addressModeV);
}
SapienRenderTexture::AddressMode SapienRenderTexture::getAddressModeW() const {
  return convertAddressMode(mTexture->getDescription().addressModeW);
}
int SapienRenderTexture::getMipmapLevels() const { return mTexture->getDescription().mipLevels; }

bool SapienRenderTexture::getIsSrgb() const { return mTexture->getDescription().srgb; }

SapienRenderTexture2D::SapienRenderTexture2D(uint32_t width, uint32_t height, vk::Format format,
                                             std::vector<char> const &rawData, uint32_t mipLevels,
                                             FilterMode filterMode, AddressMode addressMode,
                                             bool srgb) {
  mEngine = SapienRenderEngine::Get();
  vk::Filter vkf;
  vk::SamplerAddressMode vka;
  switch (filterMode) {
  case FilterMode::eNEAREST:
    vkf = vk::Filter::eNearest;
    break;
  case FilterMode::eLINEAR:
    vkf = vk::Filter::eLinear;
    break;
  }

  switch (addressMode) {
  case AddressMode::eREPEAT:
    vka = vk::SamplerAddressMode::eRepeat;
    break;
  case AddressMode::eBORDER:
    vka = vk::SamplerAddressMode::eClampToBorder;
    break;
  case AddressMode::eEDGE:
    vka = vk::SamplerAddressMode::eClampToEdge;
    break;
  case AddressMode::eMIRROR:
    vka = vk::SamplerAddressMode::eMirroredRepeat;
    break;
  }

  mTexture = mEngine->getContext()->getResourceManager()->CreateTextureFromRawData(
      width, height, 1, format, rawData, 2, mipLevels, vkf, vkf, vka, vka, vka, srgb);
}

SapienRenderTexture2D::SapienRenderTexture2D(std::string_view filename, uint32_t mipLevels,
                                             FilterMode filterMode, AddressMode addressMode,
                                             bool srgb) {
  mEngine = SapienRenderEngine::Get();
  vk::Filter vkf;
  vk::SamplerAddressMode vka;
  switch (filterMode) {
  case FilterMode::eNEAREST:
    vkf = vk::Filter::eNearest;
    break;
  case FilterMode::eLINEAR:
    vkf = vk::Filter::eLinear;
    break;
  }

  switch (addressMode) {
  case AddressMode::eREPEAT:
    vka = vk::SamplerAddressMode::eRepeat;
    break;
  case AddressMode::eBORDER:
    vka = vk::SamplerAddressMode::eClampToBorder;
    break;
  case AddressMode::eEDGE:
    vka = vk::SamplerAddressMode::eClampToEdge;
    break;
  case AddressMode::eMIRROR:
    vka = vk::SamplerAddressMode::eMirroredRepeat;
    break;
  }

  mTexture = mEngine->getContext()->getResourceManager()->CreateTextureFromFile(
      {filename.begin(), filename.end()}, mipLevels, vkf, vkf, vka, vka, srgb);
  mTexture->loadAsync().get();
}

SapienRenderTexture2D::SapienRenderTexture2D(std::shared_ptr<svulkan2::resource::SVTexture> tex) {
  mEngine = SapienRenderEngine::Get();
  mTexture = tex;
}

std::string SapienRenderTexture2D::getFilename() const {
  if (mTexture->getDescription().source ==
      svulkan2::resource::SVTextureDescription::SourceType::eFILE) {
    return mTexture->getDescription().filename;
  }
  return "";
}

void SapienRenderTexture::download(void *data) {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  mTexture->uploadToDevice();

  auto image = getTexture()->getImage()->getDeviceImage();
  auto extent = image->getExtent();
  size_t size =
      extent.width * extent.height * extent.depth * svulkan2::getFormatSize(image->getFormat());

  image->download(data, size, vk::Offset3D{0, 0, 0}, image->getExtent(), 0, 0);

  // transfer image layout back immediately
  auto context = mEngine->getContext();
  auto pool = context->createCommandPool();
  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  image->transitionLayout(cb.get(), vk::ImageLayout::eTransferSrcOptimal,
                          vk::ImageLayout::eShaderReadOnlyOptimal,
                          vk::AccessFlagBits::eTransferRead, vk::AccessFlagBits::eShaderRead,
                          vk::PipelineStageFlagBits::eTransfer,
                          vk::PipelineStageFlagBits::eFragmentShader |
                              vk::PipelineStageFlagBits::eRayTracingShaderKHR);
  cb->end();
  context->getQueue().submitAndWait(cb.get());
}

void SapienRenderTexture::upload(void *data) {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  mTexture->uploadToDevice();

  auto image = getTexture()->getImage()->getDeviceImage();
  auto extent = image->getExtent();
  size_t size =
      extent.width * extent.height * extent.depth * svulkan2::getFormatSize(image->getFormat());

  auto context = mEngine->getContext();
  auto pool = context->createCommandPool();

  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  image->transitionLayout(
      cb.get(), vk::ImageLayout::eShaderReadOnlyOptimal, vk::ImageLayout::eTransferDstOptimal,
      vk::AccessFlagBits::eShaderRead, vk::AccessFlagBits::eTransferWrite,
      vk::PipelineStageFlagBits::eFragmentShader | vk::PipelineStageFlagBits::eRayTracingShaderKHR,
      vk::PipelineStageFlagBits::eTransfer);
  cb->end();
  context->getQueue().submitAndWait(cb.get());

  image->uploadLevel(data, size, 0, 0);

  cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  image->transitionLayout(cb.get(), vk::ImageLayout::eTransferDstOptimal,
                          vk::ImageLayout::eShaderReadOnlyOptimal,
                          vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead,
                          vk::PipelineStageFlagBits::eTransfer,
                          vk::PipelineStageFlagBits::eFragmentShader |
                              vk::PipelineStageFlagBits::eRayTracingShaderKHR);
  cb->end();
  context->getQueue().submitAndWait(cb.get());
}

} // namespace sapien_renderer
} // namespace sapien
