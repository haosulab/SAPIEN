#include "sapien/sapien_renderer/cubemap.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace sapien_renderer {

SapienRenderCubemap::SapienRenderCubemap(std::string const &filename) {
  if (!filename.ends_with(".ktx") && !filename.ends_with(".KTX") && !filename.ends_with("exr") &&
      !filename.ends_with(".EXR")) {
    throw std::runtime_error("Only .ktx and .exr file are supported for cube map");
  }
  mEngine = SapienRenderEngine::Get();

  if (filename.ends_with(".exr") || filename.ends_with(".EXR")) {
    mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromEXR(filename, 5);
  } else {
    mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromKTX(filename, 5);
  }
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
