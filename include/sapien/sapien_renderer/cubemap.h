#pragma once
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

private:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVCubemap> mCubemap;
};

} // namespace sapien_renderer
} // namespace sapien
