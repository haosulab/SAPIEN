#include "sapien/renderer/svulkan2_renderer.h"
#include "sapien/renderer/svulkan2_material.h"
#include "sapien/renderer/svulkan2_rigidbody.h"
#include "sapien/renderer/svulkan2_scene.h"
#include "sapien/renderer/svulkan2_shape.h"
#include <svulkan2/resource/material.h>

namespace sapien {
namespace Renderer {

static std::weak_ptr<svulkan2::core::Context> gContext;
static std::weak_ptr<svulkan2::resource::SVResourceManager> gResourceManager;

std::string gDefaultViewerShaderDirectory = "";
void setDefaultViewerShaderDirectory(std::string const &dir) {
  gDefaultViewerShaderDirectory = dir;
}

std::string gDefaultCameraShaderDirectory = "";
void setDefaultCameraShaderDirectory(std::string const &dir) {
  gDefaultCameraShaderDirectory = dir;
}

void SVulkan2Renderer::setLogLevel(std::string const &level) {
  if (level == "debug") {
    svulkan2::log::getLogger()->set_level(spdlog::level::debug);
  } else if (level == "info") {
    svulkan2::log::getLogger()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    svulkan2::log::getLogger()->set_level(spdlog::level::warn);
  } else if (level == "err" || level == "error") {
    svulkan2::log::getLogger()->set_level(spdlog::level::err);
  } else if (level == "critical") {
    svulkan2::log::getLogger()->set_level(spdlog::level::critical);
  } else if (level == "off") {
    svulkan2::log::getLogger()->set_level(spdlog::level::off);
  } else {
    svulkan2::log::getLogger()->error("Invalid log level \"{}\"", level);
  }
}

SVulkan2Renderer::SVulkan2Renderer(bool offscreenOnly, uint32_t maxNumMaterials,
                                   uint32_t maxNumTextures, uint32_t defaultMipLevels,
                                   std::string const &device, std::string const &culling,
                                   bool doNotLoadTexture) {
  if (!gContext.expired()) {
    mContext = gContext.lock();
    mResourceManager = gResourceManager.lock();
    spdlog::get("SAPIEN")->warn("A second renderer will share the same internal context with the "
                                "first one. Arguments passed to constructor will be ignored.");
  } else {
    gContext = mContext = svulkan2::core::Context::Create(
        !offscreenOnly, maxNumMaterials, maxNumTextures, defaultMipLevels, doNotLoadTexture, device);
    gResourceManager = mResourceManager = mContext->createResourceManager();
  }
  if (culling == "back") {
    mCullMode = vk::CullModeFlagBits::eBack;
  } else if (culling == "front") {
    mCullMode = vk::CullModeFlagBits::eFront;
  } else if (culling == "front_and_back" || culling == "both") {
    mCullMode = vk::CullModeFlagBits::eFrontAndBack;
  } else if (culling == "none") {
    mCullMode = vk::CullModeFlagBits::eNone;
  } else {
    throw std::runtime_error("culling mode must be one of back, front, both, or none");
  }
}

IPxrScene *SVulkan2Renderer::createScene(std::string const &name) {
  mScenes.push_back(std::make_unique<SVulkan2Scene>(this, name));
  return mScenes.back().get();
}

void SVulkan2Renderer::removeScene(IPxrScene *scene) {
  mContext->getDevice().waitIdle(); // wait for all render tasks to finish
  mScenes.erase(std::remove_if(mScenes.begin(), mScenes.end(),
                               [scene](auto &s) { return scene == s.get(); }),
                mScenes.end());
}

std::shared_ptr<IPxrMaterial> SVulkan2Renderer::createMaterial() {
  auto mat = std::make_shared<SVulkan2Material>(
      std::make_shared<svulkan2::resource::SVMetallicMaterial>(), this);
  mat->setBaseColor({1.0, 1.0, 1.0, 1});
  return mat;
}

std::shared_ptr<IRenderMesh> SVulkan2Renderer::createMesh(std::vector<float> const &vertices,
                                                          std::vector<uint32_t> const &indices) {
  auto mesh = svulkan2::resource::SVMesh::Create(vertices, indices);
  return std::make_shared<SVulkan2Mesh>(mesh);
}

std::shared_ptr<IPxrTexture>
SVulkan2Renderer::createTexture(std::string_view filename, uint32_t mipLevels,
                                IPxrTexture::FilterMode::Enum filterMode,
                                IPxrTexture::AddressMode::Enum addressMode) {
  vk::Filter vkf;
  vk::SamplerAddressMode vka;
  switch (filterMode) {
  case IPxrTexture::FilterMode::eNEAREST:
    vkf = vk::Filter::eNearest;
    break;
  case IPxrTexture::FilterMode::eLINEAR:
    vkf = vk::Filter::eLinear;
    break;
  }

  switch (addressMode) {
  case IPxrTexture::AddressMode::eREPEAT:
    vka = vk::SamplerAddressMode::eRepeat;
    break;
  case IPxrTexture::AddressMode::eBORDER:
    vka = vk::SamplerAddressMode::eClampToBorder;
    break;
  case IPxrTexture::AddressMode::eEDGE:
    vka = vk::SamplerAddressMode::eClampToEdge;
    break;
  case IPxrTexture::AddressMode::eMIRROR:
    vka = vk::SamplerAddressMode::eMirroredRepeat;
    break;
  }

  auto texture = mContext->getResourceManager()->CreateTextureFromFile(
      {filename.begin(), filename.end()}, mipLevels, vkf, vkf, vka, vka);
  texture->loadAsync().get();
  return std::make_shared<SVulkan2Texture>(texture);
}

} // namespace Renderer
} // namespace sapien
