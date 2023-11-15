#pragma once
#include "./image.h"
#include "sapien/serialize.h"
#include <filesystem>
#include <svulkan2/resource/material.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderEngine;

class SapienRenderTexture {
public:
  enum class FilterMode { eNEAREST, eLINEAR };
  enum class AddressMode { eREPEAT, eBORDER, eEDGE, eMIRROR };

  SapienRenderTexture(uint32_t width, uint32_t height, uint32_t depth, vk::Format format,
                      std::vector<char> const &rawData, int dim, uint32_t mipLevels = 1,
                      FilterMode filterMode = FilterMode::eLINEAR,
                      AddressMode addressMode = AddressMode::eREPEAT, bool srgb = false);

  vk::Format getFormat() const;
  int getDim() const;
  int getWidth() const;
  int getHeight() const;
  int getDepth() const;

  FilterMode getFilterMode() const { return getMinFilterMode(); }
  FilterMode getMinFilterMode() const;
  FilterMode getMagFilterMode() const;

  AddressMode getAddressMode() const { return getAddressModeU(); }
  AddressMode getAddressModeU() const;
  AddressMode getAddressModeV() const;
  AddressMode getAddressModeW() const;
  int getMipmapLevels() const;
  bool getIsSrgb() const;
  int getChannels() const;

  void download(void *data);
  void upload(void *data);

  std::shared_ptr<svulkan2::resource::SVTexture> getTexture() const { return mTexture; }

protected:
  SapienRenderTexture() {}

  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVTexture> mTexture;
};

class SapienRenderTexture2D : public SapienRenderTexture {

public:
  SapienRenderTexture2D(std::string_view filename, uint32_t mipLevels, FilterMode filterMode,
                        AddressMode addressMode);
  explicit SapienRenderTexture2D(std::shared_ptr<svulkan2::resource::SVTexture> tex);

  std::string getFilename() const;

  template <class Archive> void save(Archive &ar) const {
    auto desc = mTexture->getDescription();
    if (desc.source != svulkan2::resource::SVTextureDescription::SourceType::eFILE) {
      throw std::runtime_error("textured not created from file is not currently serializable");
    }

    std::string filename =
        std::filesystem::relative(std::filesystem::path(getFilename())).string();
    uint32_t mipLevels = getMipmapLevels();
    int filterMode = static_cast<int>(getFilterMode());
    int addressMode = static_cast<int>(getAddressMode());
    ar(filename, mipLevels, filterMode, addressMode);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<SapienRenderTexture2D> &construct) {
    std::string filename;
    uint32_t mipLevels;
    int filterMode;
    int addressMode;
    ar(filename, mipLevels, filterMode, addressMode);
    construct(filename, mipLevels, static_cast<FilterMode>(filterMode),
              static_cast<AddressMode>(addressMode));
  }
};

} // namespace sapien_renderer
} // namespace sapien
