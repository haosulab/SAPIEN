#include "sapien/component/sapien_renderer/cubemap.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace component {

SapienRenderCubemap::SapienRenderCubemap(std::string const &filename) {
  if (!filename.ends_with("ktx") && !filename.ends_with("KTX")) {
    throw std::runtime_error("Only .ktx file is supported for cube map");
  }
  mEngine = SapienRenderEngine::Get();
  mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromKTX(filename, 5);
}

SapienRenderCubemap::SapienRenderCubemap(std::string const &px, std::string const &nx,
                                         std::string const &py, std::string const &ny,
                                         std::string const &pz, std::string const &nz) {
  mEngine = SapienRenderEngine::Get();
  mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromFiles(
      {px, nx, py, ny, pz, nz}, 5);
}

void SapienRenderCubemap::exportKtx(std::string const &filename) {
  mCubemap->load();
  mCubemap->uploadToDevice();
  mCubemap->exportKTX(filename);
}

} // namespace component
} // namespace sapien
