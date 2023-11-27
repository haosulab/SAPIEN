#pragma once
#include "sapien/serialize.h"
#include <svulkan2/resource/cubemap.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderEngine;

class SapienRenderCubemap {

public:
  explicit SapienRenderCubemap(std::string const &filename);
  explicit SapienRenderCubemap(std::string const &px, std::string const &nx, std::string const &py,
                               std::string const &ny, std::string const &pz,
                               std::string const &nz);

  [[nodiscard]] inline std::shared_ptr<svulkan2::resource::SVCubemap> getCubemap() const {
    return mCubemap;
  }

  void exportKtx(std::string const &filename);

  // TODO: test
  template <class Archive> void save(Archive &ar) const {
    auto desc = mCubemap->getDescription();
    if (desc.source == svulkan2::resource::SVCubemapDescription::SourceType::eCUSTOM) {
      throw std::runtime_error("serialization of cubemap not loaded from file");
    }
    auto type = static_cast<int>(desc.source);
    ar(type, desc.filenames);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<SapienRenderCubemap> &construct) {
    int type;
    std::array<std::string, 6> filenames;
    ar(type, filenames);
    switch (static_cast<svulkan2::resource::SVCubemapDescription::SourceType>(type)) {
    case svulkan2::resource::SVCubemapDescription::SourceType::eFACES:
      construct(filenames[0], filenames[1], filenames[2], filenames[3], filenames[4],
                filenames[5]);
      break;
    case svulkan2::resource::SVCubemapDescription::SourceType::eKTX:
      construct(filenames[0]);
      break;
    default:
      throw std::runtime_error("invalid cubemap source type");
    }
  }

private:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVCubemap> mCubemap;
};

} // namespace sapien_renderer
} // namespace sapien
