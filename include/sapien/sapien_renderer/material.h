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
