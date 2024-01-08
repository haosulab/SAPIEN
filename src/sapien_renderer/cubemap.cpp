#include "sapien/sapien_renderer/cubemap.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace sapien_renderer {

SapienRenderCubemap::SapienRenderCubemap(std::string const &filename) {
  mEngine = SapienRenderEngine::Get();
  mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromFile(filename, 5);
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

} // namespace sapien_renderer
} // namespace sapien
