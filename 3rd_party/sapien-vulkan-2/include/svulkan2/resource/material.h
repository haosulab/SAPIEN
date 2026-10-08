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
#include "svulkan2/common/glm.h"
#include "svulkan2/core/allocator.h"
#include "texture.h"

namespace svulkan2 {

namespace core {
class Context;
class Buffer;
} // namespace core

namespace resource {

inline void setBit(int &number, int bit) { number |= 1 << bit; }
inline void unsetBit(int &number, int bit) { number &= ~(1 << bit); }
inline int getBit(int number, int bit) { return (number >> bit) & 1; }

class SVMaterial {
public:
  inline vk::DescriptorSet getDescriptorSet() const { return mDescriptorSet.get(); }
  virtual void uploadToDevice() = 0;
  virtual void removeFromDevice() = 0;
  virtual float getOpacity() const = 0;

  virtual ~SVMaterial() = default;

  void setCullMode(vk::CullModeFlagBits cullMode) { mCullMode = cullMode; }
  vk::CullModeFlagBits getCullMode() const { return mCullMode; }

protected:
  std::shared_ptr<core::Context> mContext; // keep alive for descriptor set
  bool mRequiresBufferUpload{true};
  bool mRequiresTextureUpload{true};

  vk::CullModeFlagBits mCullMode{vk::CullModeFlagBits::eBack};

  vk::UniqueDescriptorSet mDescriptorSet;
};

class SVMetallicMaterial : public SVMaterial {

  enum TextureBit {
    eBaseColor = 0,
    eRoughness = 1,
    eNormal = 2,
    eMetallic = 3,
    eEmission = 4,
    eTransmission = 5
  };

  struct Buffer {
    glm::vec4 emission{};
    glm::vec4 baseColor{};
    float fresnel{};
    float roughness{};
    float metallic{};
    float transmission{};
    float ior{};
    float transmissionRoughness{};
    int textureMask{};
    int padding1;
    glm::vec4 textureTransform[6] = {
        {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 1},
    };
  } mBuffer{};
  std::shared_ptr<SVTexture> mBaseColorTexture;
  std::shared_ptr<SVTexture> mRoughnessTexture;
  std::shared_ptr<SVTexture> mNormalTexture;
  std::shared_ptr<SVTexture> mMetallicTexture;
  std::shared_ptr<SVTexture> mEmissionTexture;
  std::shared_ptr<SVTexture> mTransmissionTexture;

  std::unique_ptr<core::Buffer> mDeviceBuffer;

public:
  SVMetallicMaterial(glm::vec4 emission = {0, 0, 0, 1}, glm::vec4 baseColor = {0, 0, 0, 1},
                     float fresnel = 0, float roughness = 1, float metallic = 0,
                     float transparency = 0, float ior = 1.f, float transmissionRoughness = 0.f);

  void setEmission(glm::vec4 emission);
  glm::vec4 getEmission() const;

  void setBaseColor(glm::vec4 baseColor);
  glm::vec4 getBaseColor() const;

  void setRoughness(float roughness);
  float getRoughness() const;

  void setFresnel(float fresnel);
  float getFresnel() const;

  void setMetallic(float metallic);
  float getMetallic() const;

  void setTransmission(float transmission);
  float getTransmission() const;

  void setIor(float ior);
  float getIor() const;

  void setTransmissionRoughness(float roughness);
  float getTransmissionRoughness() const;

  void setBaseColorTexture(std::shared_ptr<SVTexture> texture);
  void setRoughnessTexture(std::shared_ptr<SVTexture> texture);
  void setNormalTexture(std::shared_ptr<SVTexture> texture);
  void setMetallicTexture(std::shared_ptr<SVTexture> texture);
  void setEmissionTexture(std::shared_ptr<SVTexture> texture);
  void setTransmissionTexture(std::shared_ptr<SVTexture> texture);

  void setBaseColorTextureTransform(glm::vec4 const &transform);
  void setRoughnessTextureTransform(glm::vec4 const &transform);
  void setNormalTextureTransform(glm::vec4 const &transform);
  void setMetallicTextureTransform(glm::vec4 const &transform);
  void setEmissionTextureTransform(glm::vec4 const &transform);
  void setTransmissionTextureTransform(glm::vec4 const &transform);

  glm::vec4 getBaseColorTextureTransform() const;
  glm::vec4 getRoughnessTextureTransform() const;
  glm::vec4 getNormalTextureTransform() const;
  glm::vec4 getMetallicTextureTransform() const;
  glm::vec4 getEmissionTextureTransform() const;
  glm::vec4 getTransmissionTextureTransform() const;

  // TODO: getters
  std::shared_ptr<SVTexture> getBaseColorTexture() const;
  std::shared_ptr<SVTexture> getRoughnessTexture() const;
  std::shared_ptr<SVTexture> getNormalTexture() const;
  std::shared_ptr<SVTexture> getMetallicTexture() const;
  std::shared_ptr<SVTexture> getEmissionTexture() const;
  std::shared_ptr<SVTexture> getTransmissionTexture() const;

  void setTextures(std::shared_ptr<SVTexture> baseColorTexture,
                   std::shared_ptr<SVTexture> roughnessTexture,
                   std::shared_ptr<SVTexture> normalTexture,
                   std::shared_ptr<SVTexture> metallicTexture,
                   std::shared_ptr<SVTexture> emissionTexture,
                   std::shared_ptr<SVTexture> transmissionTexture);

  void uploadToDevice() override;
  void removeFromDevice() override;
  inline float getOpacity() const override { return mBuffer.baseColor.a; }

  core::Buffer &getDeviceBuffer() const;

  SVMetallicMaterial(SVMetallicMaterial const &) = delete;
  SVMetallicMaterial &operator=(SVMetallicMaterial const &) = delete;
  SVMetallicMaterial(SVMetallicMaterial const &&) = delete;
  SVMetallicMaterial &operator=(SVMetallicMaterial const &&) = delete;

  virtual ~SVMetallicMaterial();
};

} // namespace resource
} // namespace svulkan2