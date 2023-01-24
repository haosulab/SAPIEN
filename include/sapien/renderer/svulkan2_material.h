#pragma once
#include "render_interface.h"
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/scene/scene.h>

namespace sapien {
namespace Renderer {

class SVulkan2Texture : public IPxrTexture {
  std::shared_ptr<svulkan2::resource::SVTexture> mTexture;

public:
  explicit SVulkan2Texture(std::shared_ptr<svulkan2::resource::SVTexture> texture);
  [[nodiscard]] virtual int getMipmapLevels() const override;
  [[nodiscard]] virtual int getWidth() const override;
  [[nodiscard]] virtual int getHeight() const override;
  [[nodiscard]] virtual int getChannels() const override;
  [[nodiscard]] virtual Type::Enum getType() const override;
  [[nodiscard]] virtual AddressMode::Enum getAddressMode() const override;
  [[nodiscard]] virtual FilterMode::Enum getFilterMode() const override;
  [[nodiscard]] virtual std::string getFilename() const override;

  [[nodiscard]] inline std::shared_ptr<svulkan2::resource::SVTexture> getTexture() const {
    return mTexture;
  }
};

class SVulkan2Material : public IPxrMaterial {
  std::shared_ptr<svulkan2::resource::SVMetallicMaterial> mMaterial;
  class SVulkan2Renderer *mParentRenderer;

public:
  explicit SVulkan2Material(std::shared_ptr<svulkan2::resource::SVMetallicMaterial> material,
                            SVulkan2Renderer *renderer);
  void setEmission(std::array<float, 4> color) override;
  [[nodiscard]] std::array<float, 4> getEmission() const override;
  void setBaseColor(std::array<float, 4> color) override;
  [[nodiscard]] std::array<float, 4> getBaseColor() const override;
  void setRoughness(float roughness) override;
  [[nodiscard]] float getRoughness() const override;
  void setSpecular(float specular) override;
  [[nodiscard]] float getSpecular() const override;
  void setMetallic(float metallic) override;
  [[nodiscard]] float getMetallic() const override;
  void setIOR(float ior) override;
  [[nodiscard]] float getIOR() const override;
  void setTransmission(float transmission) override;
  [[nodiscard]] float getTransmission() const override;
  void setTransmissionRoughness(float roughness) override;
  [[nodiscard]] float getTransmissionRoughness() const override;

  void setEmissionTexture(std::shared_ptr<IPxrTexture> texture) override;
  void setEmissionTextureFromFilename(std::string_view filename) override;
  [[nodiscard]] std::shared_ptr<IPxrTexture> getEmissionTexture() const override;
  void setDiffuseTexture(std::shared_ptr<IPxrTexture> texture) override;
  void setDiffuseTextureFromFilename(std::string_view filename) override;
  [[nodiscard]] std::shared_ptr<IPxrTexture> getDiffuseTexture() const override;
  void setRoughnessTexture(std::shared_ptr<IPxrTexture> texture) override;
  void setRoughnessTextureFromFilename(std::string_view filename) override;
  [[nodiscard]] std::shared_ptr<IPxrTexture> getRoughnessTexture() const override;
  void setMetallicTexture(std::shared_ptr<IPxrTexture> texture) override;
  void setMetallicTextureFromFilename(std::string_view filename) override;
  [[nodiscard]] std::shared_ptr<IPxrTexture> getMetallicTexture() const override;
  void setNormalTexture(std::shared_ptr<IPxrTexture> texture) override;
  void setNormalTextureFromFilename(std::string_view filename) override;
  [[nodiscard]] std::shared_ptr<IPxrTexture> getNormalTexture() const override;
  void setTransmissionTexture(std::shared_ptr<IPxrTexture> texture) override;
  void setTransmissionTextureFromFilename(std::string_view filename) override;
  [[nodiscard]] std::shared_ptr<IPxrTexture> getTransmissionTexture() const override;

  [[nodiscard]] std::shared_ptr<svulkan2::resource::SVMetallicMaterial> getMaterial() const {
    return mMaterial;
  }
};

} // namespace Renderer
} // namespace sapien
