#include "sapien/component/sapien_renderer/texture.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace component {

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
    return vk::Filter::eLinear;
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

SapienRenderTexture2D::SapienRenderTexture2D(std::string_view filename, uint32_t mipLevels,
                                             FilterMode filterMode, AddressMode addressMode) {
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
      {filename.begin(), filename.end()}, mipLevels, vkf, vkf, vka, vka, true);
  mTexture->loadAsync().get();
}

SapienRenderTexture2D::SapienRenderTexture2D(std::shared_ptr<svulkan2::resource::SVTexture> tex) {
  mEngine = SapienRenderEngine::Get();
  mTexture = tex;
}

// SapienRenderTexture2D::Type SapienRenderTexture2D::getType() const {
// switch (mTexture->getDescription().format) {
// case svulkan2::resource::SVTextureDescription::Format::eUINT8:
//   return Type::eBYTE;
// case svulkan2::resource::SVTextureDescription::Format::eFLOAT:
//   return Type::eFLOAT;
// default:
//   return Type::eOTHER;
// }
// return Type::eOTHER;
// }

std::string SapienRenderTexture2D::getFilename() const {
  if (mTexture->getDescription().source ==
      svulkan2::resource::SVTextureDescription::SourceType::eFILE) {
    return mTexture->getDescription().filename;
  }
  return "";
}

} // namespace component
} // namespace sapien
