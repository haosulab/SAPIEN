#include "sapien/renderer/svulkan2_material.h"
#include "sapien/renderer/svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

SVulkan2Texture::SVulkan2Texture(std::shared_ptr<svulkan2::resource::SVTexture> texture)
    : mTexture(texture) {}

int SVulkan2Texture::getMipmapLevels() const { return mTexture->getDescription().mipLevels; };

int SVulkan2Texture::getWidth() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getWidth();
}

int SVulkan2Texture::getHeight() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getHeight();
}

int SVulkan2Texture::getChannels() const {
  if (!mTexture->isLoaded()) {
    mTexture->loadAsync().get();
  }
  return mTexture->getImage()->getChannels();
}

IRenderTexture::Type::Enum SVulkan2Texture::getType() const {
  switch (mTexture->getDescription().format) {
  case svulkan2::resource::SVTextureDescription::Format::eUINT8:
    return Type::eBYTE;
  case svulkan2::resource::SVTextureDescription::Format::eFLOAT:
    return Type::eFLOAT;
  default:
    return Type::eOTHER;
  }
  return Type::eOTHER;
}

IRenderTexture::AddressMode::Enum SVulkan2Texture::getAddressMode() const {
  switch (mTexture->getDescription().addressModeU) {
  case vk::SamplerAddressMode::eRepeat:
    return AddressMode::eREPEAT;
  case vk::SamplerAddressMode::eClampToEdge:
    return AddressMode::eEDGE;
  case vk::SamplerAddressMode::eClampToBorder:
    return AddressMode::eBORDER;
  default:
    return AddressMode::eMIRROR;
  }
  return AddressMode::eREPEAT;
};

IRenderTexture::FilterMode::Enum SVulkan2Texture::getFilterMode() const {
  switch (mTexture->getDescription().magFilter) {
  case vk::Filter::eNearest:
    return FilterMode::eNEAREST;
  case vk::Filter::eLinear:
    return FilterMode::eLINEAR;
  default:
    return FilterMode::eLINEAR;
  }
  return FilterMode::eNEAREST;
};

std::string SVulkan2Texture::getFilename() const {
  if (mTexture->getDescription().source ==
      svulkan2::resource::SVTextureDescription::SourceType::eFILE) {
    return mTexture->getDescription().filename;
  }
  return "";
}

SVulkan2Material::SVulkan2Material(
    std::shared_ptr<svulkan2::resource::SVMetallicMaterial> material, SVulkan2Renderer *renderer)
    : mMaterial(material), mParentRenderer(renderer) {}

void SVulkan2Material::setEmission(std::array<float, 4> color) {
  mMaterial->setEmission({color[0], color[1], color[2], color[3]});
}
std::array<float, 4> SVulkan2Material::getEmission() const {
  auto color = mMaterial->getEmission();
  return {color.r, color.g, color.b, color.a};
}

void SVulkan2Material::setBaseColor(std::array<float, 4> color) {
  mMaterial->setBaseColor({color[0], color[1], color[2], color[3]});
}
std::array<float, 4> SVulkan2Material::getBaseColor() const {
  auto color = mMaterial->getBaseColor();
  return {color.r, color.g, color.b, color.a};
}

void SVulkan2Material::setRoughness(float roughness) { mMaterial->setRoughness(roughness); }
float SVulkan2Material::getRoughness() const { return mMaterial->getRoughness(); }
void SVulkan2Material::setSpecular(float specular) { mMaterial->setFresnel(specular); }
float SVulkan2Material::getSpecular() const { return mMaterial->getFresnel(); }
void SVulkan2Material::setMetallic(float metallic) { mMaterial->setMetallic(metallic); }
float SVulkan2Material::getMetallic() const { return mMaterial->getMetallic(); }
void SVulkan2Material::setIOR(float ior) { mMaterial->setIor(ior); }
float SVulkan2Material::getIOR() const { return mMaterial->getIor(); }
void SVulkan2Material::setTransmission(float transmission) {
  mMaterial->setTransmission(transmission);
}
float SVulkan2Material::getTransmission() const { return mMaterial->getTransmission(); }
void SVulkan2Material::setTransmissionRoughness(float roughness) {
  mMaterial->setTransmissionRoughness(roughness);
}
float SVulkan2Material::getTransmissionRoughness() const {
  return mMaterial->getTransmissionRoughness();
}

void SVulkan2Material::setEmissionTexture(std::shared_ptr<IRenderTexture> texture) {
  if (auto tex = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mMaterial->setEmissionTexture(tex->getTexture());
  } else {
    mMaterial->setEmissionTexture(nullptr);
  }
}

std::shared_ptr<IRenderTexture> SVulkan2Material::getEmissionTexture() const {
  auto tex = mMaterial->getEmissionTexture();
  return tex ? std::make_shared<SVulkan2Texture>(tex) : nullptr;
}

void SVulkan2Material::setDiffuseTexture(std::shared_ptr<IRenderTexture> texture) {
  if (auto tex = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mMaterial->setDiffuseTexture(tex->getTexture());
  } else {
    mMaterial->setDiffuseTexture(nullptr);
  }
}
std::shared_ptr<IRenderTexture> SVulkan2Material::getDiffuseTexture() const {
  auto tex = mMaterial->getDiffuseTexture();
  return tex ? std::make_shared<SVulkan2Texture>(tex) : nullptr;
}

void SVulkan2Material::setNormalTexture(std::shared_ptr<IRenderTexture> texture) {
  if (auto tex = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mMaterial->setNormalTexture(tex->getTexture());
  } else {
    mMaterial->setNormalTexture(nullptr);
  }
}
std::shared_ptr<IRenderTexture> SVulkan2Material::getNormalTexture() const {
  auto tex = mMaterial->getNormalTexture();
  return tex ? std::make_shared<SVulkan2Texture>(tex) : nullptr;
}

void SVulkan2Material::setRoughnessTexture(std::shared_ptr<IRenderTexture> texture) {
  if (auto tex = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mMaterial->setRoughnessTexture(tex->getTexture());
  } else {
    mMaterial->setRoughnessTexture(nullptr);
  }
}
std::shared_ptr<IRenderTexture> SVulkan2Material::getRoughnessTexture() const {
  auto tex = mMaterial->getRoughnessTexture();
  return tex ? std::make_shared<SVulkan2Texture>(tex) : nullptr;
}

void SVulkan2Material::setMetallicTexture(std::shared_ptr<IRenderTexture> texture) {
  if (auto tex = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mMaterial->setMetallicTexture(tex->getTexture());
  } else {
    mMaterial->setMetallicTexture(nullptr);
  }
}
std::shared_ptr<IRenderTexture> SVulkan2Material::getMetallicTexture() const {
  auto tex = mMaterial->getMetallicTexture();
  return tex ? std::make_shared<SVulkan2Texture>(tex) : nullptr;
}

void SVulkan2Material::setTransmissionTexture(std::shared_ptr<IRenderTexture> texture) {
  if (auto tex = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mMaterial->setTransmissionTexture(tex->getTexture());
  } else {
    mMaterial->setTransmissionTexture(nullptr);
  }
}
std::shared_ptr<IRenderTexture> SVulkan2Material::getTransmissionTexture() const {
  auto tex = mMaterial->getTransmissionTexture();
  return tex ? std::make_shared<SVulkan2Texture>(tex) : nullptr;
}

void SVulkan2Material::setEmissionTextureFromFilename(std::string_view filename) {
  setEmissionTexture(mParentRenderer->createTexture(filename));
}
void SVulkan2Material::setDiffuseTextureFromFilename(std::string_view filename) {
  setDiffuseTexture(mParentRenderer->createTexture(filename));
}
void SVulkan2Material::setNormalTextureFromFilename(std::string_view filename) {
  setNormalTexture(mParentRenderer->createTexture(filename));
}
void SVulkan2Material::setMetallicTextureFromFilename(std::string_view filename) {
  setMetallicTexture(mParentRenderer->createTexture(filename));
}
void SVulkan2Material::setRoughnessTextureFromFilename(std::string_view filename) {
  setRoughnessTexture(mParentRenderer->createTexture(filename));
}
void SVulkan2Material::setTransmissionTextureFromFilename(std::string_view filename) {
  setTransmissionTexture(mParentRenderer->createTexture(filename));
}

} // namespace Renderer
} // namespace sapien
