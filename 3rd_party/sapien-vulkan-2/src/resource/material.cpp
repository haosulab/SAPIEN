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
#include "svulkan2/resource/material.h"
#include "svulkan2/core/buffer.h"
#include "svulkan2/core/context.h"

namespace svulkan2 {
namespace resource {

static void updateDescriptorSets(
    vk::Device device, vk::DescriptorSet descriptorSet,
    std::vector<std::tuple<vk::DescriptorType, vk::Buffer, vk::BufferView>> const &bufferData,
    std::vector<std::tuple<vk::ImageView, vk::Sampler>> textureData, uint32_t bindingOffset) {
  
  std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  writeDescriptorSets.reserve(bufferData.size() + textureData.size());

  std::vector<vk::DescriptorBufferInfo> bufferInfos;
  bufferInfos.reserve(bufferData.size());
  for (auto const &bd : bufferData) {
    bufferInfos.push_back(vk::DescriptorBufferInfo(std::get<1>(bd), 0, VK_WHOLE_SIZE));
    writeDescriptorSets.push_back(
        vk::WriteDescriptorSet(descriptorSet, bindingOffset++, 0, 1, std::get<0>(bd), nullptr,
                               &bufferInfos.back(), std::get<2>(bd) ? &std::get<2>(bd) : nullptr));
  }
  std::vector<vk::DescriptorImageInfo> imageInfos;
  imageInfos.reserve(textureData.size());
  for (auto const &tex : textureData) {
    imageInfos.push_back(vk::DescriptorImageInfo(std::get<1>(tex), std::get<0>(tex),
                                                 vk::ImageLayout::eShaderReadOnlyOptimal));
    writeDescriptorSets.push_back(
        vk::WriteDescriptorSet(descriptorSet, bindingOffset++, 0, 1,
                               vk::DescriptorType::eCombinedImageSampler, &imageInfos.back()));
  }
  device.updateDescriptorSets(writeDescriptorSets, nullptr);
}

SVMetallicMaterial::SVMetallicMaterial(glm::vec4 emission, glm::vec4 baseColor, float fresnel,
                                       float roughness, float metallic, float transparency,
                                       float ior, float transmissionRoughness) {
  mBuffer.emission = emission;
  mBuffer.baseColor = baseColor;
  mBuffer.fresnel = fresnel;
  mBuffer.roughness = roughness;
  mBuffer.metallic = metallic;
  mBuffer.transmission = transparency;
  mBuffer.ior = ior;
  mBuffer.transmissionRoughness = transmissionRoughness;
  mBuffer.textureMask = 0;
}

SVMetallicMaterial::~SVMetallicMaterial() {
  if (mContext) {
    std::scoped_lock lock(mContext->getGlobalLock());
    mDescriptorSet.reset();
    mDeviceBuffer.reset();
  }
}

void SVMetallicMaterial::setEmission(glm::vec4 emission) {
  mRequiresBufferUpload = true;
  mBuffer.emission = emission;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

glm::vec4 SVMetallicMaterial::getEmission() const { return mBuffer.emission; }

void SVMetallicMaterial::setBaseColor(glm::vec4 baseColor) {
  mRequiresBufferUpload = true;
  mBuffer.baseColor = baseColor;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

glm::vec4 SVMetallicMaterial::getBaseColor() const { return mBuffer.baseColor; }

void SVMetallicMaterial::setRoughness(float roughness) {
  mRequiresBufferUpload = true;
  mBuffer.roughness = roughness;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

float SVMetallicMaterial::getRoughness() const { return mBuffer.roughness; }

void SVMetallicMaterial::setFresnel(float fresnel) {
  mRequiresBufferUpload = true;
  mBuffer.fresnel = fresnel;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

float SVMetallicMaterial::getFresnel() const { return mBuffer.fresnel; }

void SVMetallicMaterial::setMetallic(float metallic) {
  mRequiresBufferUpload = true;
  mBuffer.metallic = metallic;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

float SVMetallicMaterial::getMetallic() const { return mBuffer.metallic; }

void SVMetallicMaterial::setTransmission(float transmission) {
  mRequiresBufferUpload = true;
  mBuffer.transmission = transmission;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
float SVMetallicMaterial::getTransmission() const { return mBuffer.transmission; }

void SVMetallicMaterial::setIor(float ior) {
  mRequiresBufferUpload = true;
  mBuffer.ior = ior;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
float SVMetallicMaterial::getIor() const { return mBuffer.ior; }

void SVMetallicMaterial::setTransmissionRoughness(float roughness) {
  mRequiresBufferUpload = true;
  mBuffer.transmissionRoughness = roughness;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
float SVMetallicMaterial::getTransmissionRoughness() const {
  return mBuffer.transmissionRoughness;
}

std::shared_ptr<SVTexture> SVMetallicMaterial::getBaseColorTexture() const {
  if (getBit(mBuffer.textureMask, TextureBit::eBaseColor) == 0) {
    return nullptr;
  }
  return mBaseColorTexture;
}

std::shared_ptr<SVTexture> SVMetallicMaterial::getRoughnessTexture() const {
  if (getBit(mBuffer.textureMask, TextureBit::eRoughness) == 0) {
    return nullptr;
  }
  return mRoughnessTexture;
}

std::shared_ptr<SVTexture> SVMetallicMaterial::getNormalTexture() const {
  if (getBit(mBuffer.textureMask, TextureBit::eNormal) == 0) {
    return nullptr;
  }
  return mNormalTexture;
}

std::shared_ptr<SVTexture> SVMetallicMaterial::getMetallicTexture() const {
  if (getBit(mBuffer.textureMask, TextureBit::eMetallic) == 0) {
    return nullptr;
  }
  return mMetallicTexture;
}

std::shared_ptr<SVTexture> SVMetallicMaterial::getEmissionTexture() const {
  if (getBit(mBuffer.textureMask, TextureBit::eEmission) == 0) {
    return nullptr;
  }
  return mEmissionTexture;
}

std::shared_ptr<SVTexture> SVMetallicMaterial::getTransmissionTexture() const {
  if (getBit(mBuffer.textureMask, TextureBit::eTransmission) == 0) {
    return nullptr;
  }
  return mTransmissionTexture;
}

void SVMetallicMaterial::setBaseColorTexture(std::shared_ptr<SVTexture> texture) {
  mRequiresTextureUpload = true;
  mBaseColorTexture = texture;
  if (mBaseColorTexture) {
    setBit(mBuffer.textureMask, TextureBit::eBaseColor);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eBaseColor);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
void SVMetallicMaterial::setRoughnessTexture(std::shared_ptr<SVTexture> texture) {
  mRequiresTextureUpload = true;
  mRoughnessTexture = texture;
  if (mRoughnessTexture) {
    setBit(mBuffer.textureMask, TextureBit::eRoughness);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eRoughness);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

void SVMetallicMaterial::setNormalTexture(std::shared_ptr<SVTexture> texture) {
  mRequiresTextureUpload = true;
  mNormalTexture = texture;
  if (mNormalTexture) {
    setBit(mBuffer.textureMask, TextureBit::eNormal);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eNormal);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

void SVMetallicMaterial::setMetallicTexture(std::shared_ptr<SVTexture> texture) {
  mRequiresTextureUpload = true;
  mMetallicTexture = texture;
  if (mMetallicTexture) {
    setBit(mBuffer.textureMask, TextureBit::eMetallic);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eMetallic);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

void SVMetallicMaterial::setEmissionTexture(std::shared_ptr<SVTexture> texture) {
  mRequiresTextureUpload = true;
  mEmissionTexture = texture;
  if (mEmissionTexture) {
    setBit(mBuffer.textureMask, TextureBit::eEmission);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eEmission);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

void SVMetallicMaterial::setTransmissionTexture(std::shared_ptr<SVTexture> texture) {
  mRequiresTextureUpload = true;
  mTransmissionTexture = texture;
  if (mTransmissionTexture) {
    setBit(mBuffer.textureMask, TextureBit::eTransmission);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eTransmission);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

void SVMetallicMaterial::setBaseColorTextureTransform(glm::vec4 const &transform) {
  mRequiresBufferUpload = true;
  mBuffer.textureTransform[TextureBit::eBaseColor] = transform;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
void SVMetallicMaterial::setRoughnessTextureTransform(glm::vec4 const &transform) {
  mRequiresBufferUpload = true;
  mBuffer.textureTransform[TextureBit::eRoughness] = transform;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
void SVMetallicMaterial::setNormalTextureTransform(glm::vec4 const &transform) {
  mRequiresBufferUpload = true;
  mBuffer.textureTransform[TextureBit::eNormal] = transform;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
void SVMetallicMaterial::setMetallicTextureTransform(glm::vec4 const &transform) {
  mRequiresBufferUpload = true;
  mBuffer.textureTransform[TextureBit::eMetallic] = transform;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
void SVMetallicMaterial::setEmissionTextureTransform(glm::vec4 const &transform) {
  mRequiresBufferUpload = true;
  mBuffer.textureTransform[TextureBit::eEmission] = transform;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}
void SVMetallicMaterial::setTransmissionTextureTransform(glm::vec4 const &transform) {
  mRequiresBufferUpload = true;
  mBuffer.textureTransform[TextureBit::eTransmission] = transform;
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

glm::vec4 SVMetallicMaterial::getBaseColorTextureTransform() const {
  return mBuffer.textureTransform[TextureBit::eBaseColor];
}
glm::vec4 SVMetallicMaterial::getRoughnessTextureTransform() const {
  return mBuffer.textureTransform[TextureBit::eRoughness];
}
glm::vec4 SVMetallicMaterial::getNormalTextureTransform() const {
  return mBuffer.textureTransform[TextureBit::eNormal];
}
glm::vec4 SVMetallicMaterial::getMetallicTextureTransform() const {
  return mBuffer.textureTransform[TextureBit::eMetallic];
}
glm::vec4 SVMetallicMaterial::getEmissionTextureTransform() const {
  return mBuffer.textureTransform[TextureBit::eEmission];
}
glm::vec4 SVMetallicMaterial::getTransmissionTextureTransform() const {
  return mBuffer.textureTransform[TextureBit::eTransmission];
}

void SVMetallicMaterial::setTextures(std::shared_ptr<SVTexture> baseColorTexture,
                                     std::shared_ptr<SVTexture> roughnessTexture,
                                     std::shared_ptr<SVTexture> normalTexture,
                                     std::shared_ptr<SVTexture> metallicTexture,
                                     std::shared_ptr<SVTexture> emissionTexture,
                                     std::shared_ptr<SVTexture> transmissionTexture) {
  mRequiresTextureUpload = true;

  mBaseColorTexture = baseColorTexture;
  mRoughnessTexture = roughnessTexture;
  mNormalTexture = normalTexture;
  mMetallicTexture = metallicTexture;
  mEmissionTexture = emissionTexture;
  mTransmissionTexture = transmissionTexture;

  if (mBaseColorTexture) {
    setBit(mBuffer.textureMask, TextureBit::eBaseColor);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eBaseColor);
  }

  if (mRoughnessTexture) {
    setBit(mBuffer.textureMask, TextureBit::eRoughness);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eRoughness);
  }

  if (mNormalTexture) {
    setBit(mBuffer.textureMask, TextureBit::eNormal);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eNormal);
  }

  if (mMetallicTexture) {
    setBit(mBuffer.textureMask, TextureBit::eMetallic);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eMetallic);
  }

  if (mEmissionTexture) {
    setBit(mBuffer.textureMask, TextureBit::eEmission);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eEmission);
  }

  if (mTransmissionTexture) {
    setBit(mBuffer.textureMask, TextureBit::eTransmission);
  } else {
    unsetBit(mBuffer.textureMask, TextureBit::eTransmission);
  }
  if (mDeviceBuffer) {
    uploadToDevice();
  }
}

void SVMetallicMaterial::uploadToDevice() {
  mContext = core::Context::Get();
  std::scoped_lock lock(mContext->getGlobalLock());
  if (!mDeviceBuffer) {
    vk::BufferUsageFlags flags = vk::BufferUsageFlagBits::eUniformBuffer;
    if (mContext->isRayTracingAvailable()) {
      flags |= vk::BufferUsageFlagBits::eStorageBuffer;
    }

    mDeviceBuffer = core::Buffer::Create(sizeof(SVMetallicMaterial::Buffer), flags,
                                         VMA_MEMORY_USAGE_CPU_TO_GPU);

    auto layout = mContext->getMetallicDescriptorSetLayout();
    mDescriptorSet = mContext->getDescriptorPool().allocateSet(layout);
  }

  if (mRequiresBufferUpload) {
    mDeviceBuffer->upload(mBuffer);
    mRequiresBufferUpload = false;
  }

  if (mRequiresTextureUpload) {
    auto defaultTexture = mContext->getResourceManager()->getDefaultTexture();
    std::vector<std::tuple<vk::ImageView, vk::Sampler>> textures;
    if (mBaseColorTexture) {
      mBaseColorTexture->uploadToDevice();
      textures.push_back({mBaseColorTexture->getImageView(), mBaseColorTexture->getSampler()});
    } else {
      defaultTexture->uploadToDevice();
      textures.push_back({defaultTexture->getImageView(), defaultTexture->getSampler()});
    }
    if (mRoughnessTexture) {
      mRoughnessTexture->uploadToDevice();
      textures.push_back({mRoughnessTexture->getImageView(), mRoughnessTexture->getSampler()});
    } else {
      defaultTexture->uploadToDevice();
      textures.push_back({defaultTexture->getImageView(), defaultTexture->getSampler()});
    }
    if (mNormalTexture) {
      mNormalTexture->uploadToDevice();
      textures.push_back({mNormalTexture->getImageView(), mNormalTexture->getSampler()});
    } else {
      defaultTexture->uploadToDevice();
      textures.push_back({defaultTexture->getImageView(), defaultTexture->getSampler()});
    }
    if (mMetallicTexture) {
      mMetallicTexture->uploadToDevice();
      textures.push_back({mMetallicTexture->getImageView(), mMetallicTexture->getSampler()});
    } else {
      defaultTexture->uploadToDevice();
      textures.push_back({defaultTexture->getImageView(), defaultTexture->getSampler()});
    }
    if (mEmissionTexture) {
      mEmissionTexture->uploadToDevice();
      textures.push_back({mEmissionTexture->getImageView(), mEmissionTexture->getSampler()});
    } else {
      defaultTexture->uploadToDevice();
      textures.push_back({defaultTexture->getImageView(), defaultTexture->getSampler()});
    }
    if (mTransmissionTexture) {
      mTransmissionTexture->uploadToDevice();
      textures.push_back(
          {mTransmissionTexture->getImageView(), mTransmissionTexture->getSampler()});
    } else {
      defaultTexture->uploadToDevice();
      textures.push_back({defaultTexture->getImageView(), defaultTexture->getSampler()});
    }
    updateDescriptorSets(
        mContext->getDevice(), mDescriptorSet.get(),
        {{vk::DescriptorType::eUniformBuffer, mDeviceBuffer->getVulkanBuffer(), nullptr}},
        textures, 0);
    mRequiresTextureUpload = false;
  }
}

void SVMetallicMaterial::removeFromDevice() {
  if (!mContext) {
    return;
  }
  if (!mDescriptorSet) {
    return;
  }
  std::scoped_lock lock(mContext->getGlobalLock());
  mDescriptorSet.reset();
  mDeviceBuffer.reset();

  mRequiresBufferUpload = true;
  if (mBaseColorTexture) {
    mBaseColorTexture->removeFromDevice();
  }
  if (mRoughnessTexture) {
    mRoughnessTexture->removeFromDevice();
  }
  if (mNormalTexture) {
    mNormalTexture->removeFromDevice();
  }
  if (mMetallicTexture) {
    mMetallicTexture->removeFromDevice();
  }
  if (mEmissionTexture) {
    mEmissionTexture->removeFromDevice();
  }
  if (mTransmissionTexture) {
    mTransmissionTexture->removeFromDevice();
  }
}

core::Buffer &SVMetallicMaterial::getDeviceBuffer() const {
  if (!mDeviceBuffer) {
    throw std::runtime_error("failed to get device buffer: not uploaded");
  }
  return *mDeviceBuffer;
}

} // namespace resource
} // namespace svulkan2