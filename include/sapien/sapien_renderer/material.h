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
#include "texture.h"
#include <svulkan2/resource/material.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderEngine;

class SapienRenderMaterial {
public:
  SapienRenderMaterial();
  SapienRenderMaterial(std::array<float, 4> emission, std::array<float, 4> baseColor,
                       float fresnel, float roughness, float metallic, float transmission,
                       float ior, float transmissionRoughness);
  explicit SapienRenderMaterial(std::shared_ptr<svulkan2::resource::SVMetallicMaterial> material);

  void setEmission(std::array<float, 4> color);
  [[nodiscard]] std::array<float, 4> getEmission() const;
  void setBaseColor(std::array<float, 4> color);
  [[nodiscard]] std::array<float, 4> getBaseColor() const;
  void setRoughness(float roughness);
  [[nodiscard]] float getRoughness() const;
  void setSpecular(float specular);
  [[nodiscard]] float getSpecular() const;
  void setMetallic(float metallic);
  [[nodiscard]] float getMetallic() const;
  void setIOR(float ior);
  [[nodiscard]] float getIOR() const;
  void setTransmission(float transmission);
  [[nodiscard]] float getTransmission() const;
  void setTransmissionRoughness(float roughness);
  [[nodiscard]] float getTransmissionRoughness() const;

  void setEmissionTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getEmissionTexture() const;
  void setBaseColorTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getBaseColorTexture() const;
  void setRoughnessTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getRoughnessTexture() const;
  void setMetallicTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getMetallicTexture() const;
  void setNormalTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getNormalTexture() const;
  void setTransmissionTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getTransmissionTexture() const;

  // TODO: support the following?
  // void setEmissionTextureFromFilename(std::string_view filename);
  // void setDiffuseTextureFromFilename(std::string_view filename);
  // void setRoughnessTextureFromFilename(std::string_view filename);
  // void setMetallicTextureFromFilename(std::string_view filename);
  // void setNormalTextureFromFilename(std::string_view filename);
  // void setTransmissionTextureFromFilename(std::string_view filename);

  [[nodiscard]] std::shared_ptr<svulkan2::resource::SVMetallicMaterial> getMaterial() const {
    return mMaterial;
  }

public:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVMetallicMaterial> mMaterial;
};

} // namespace sapien_renderer
} // namespace sapien
