#pragma once
#include "sapien/serialize.h"
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
  void setDiffuseTexture(std::shared_ptr<SapienRenderTexture2D> texture);
  [[nodiscard]] std::shared_ptr<SapienRenderTexture2D> getDiffuseTexture() const;
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

  template <class Archive> void save(Archive &ar) const {
    ar(getEmission(), getBaseColor(), getRoughness(), getSpecular(), getMetallic(), getIOR(),
       getTransmission(), getTransmissionRoughness());
    ar(getEmissionTexture(), getDiffuseTexture(), getRoughnessTexture(), getMetallicTexture(),
       getNormalTexture(), getTransmissionTexture());
  }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<SapienRenderMaterial> &construct) {
    std::array<float, 4> emission;
    std::array<float, 4> baseColor;
    float roughness;
    float specular;
    float metallic;
    float ior;
    float transmission;
    float transmissionRoughness;
    ar(emission, baseColor, roughness, specular, metallic, ior, transmission,
       transmissionRoughness);
    std::shared_ptr<SapienRenderTexture2D> emissionTexture;
    std::shared_ptr<SapienRenderTexture2D> diffuseTexture;
    std::shared_ptr<SapienRenderTexture2D> roughnessTexture;
    std::shared_ptr<SapienRenderTexture2D> metallicTexture;
    std::shared_ptr<SapienRenderTexture2D> normalTexture;
    std::shared_ptr<SapienRenderTexture2D> transmissionTexture;
    ar(emissionTexture, diffuseTexture, roughnessTexture, metallicTexture, normalTexture,
       transmissionTexture);

    construct(emission, baseColor, specular, roughness, metallic, transmission, ior,
              transmissionRoughness);
    construct->setEmissionTexture(emissionTexture);
    construct->setDiffuseTexture(diffuseTexture);
    construct->setRoughnessTexture(roughnessTexture);
    construct->setMetallicTexture(metallicTexture);
    construct->setNormalTexture(normalTexture);
    construct->setTransmissionTexture(transmissionTexture);
  }

public:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVMetallicMaterial> mMaterial;
};

} // namespace sapien_renderer
} // namespace sapien
