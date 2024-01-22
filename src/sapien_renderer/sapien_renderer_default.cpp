#include "sapien/sapien_renderer/sapien_renderer_default.h"
#include <filesystem>
#include <svulkan2/core/context.h>

namespace fs = std::filesystem;

namespace sapien {
namespace sapien_renderer {

void SapienRendererDefault::setGlobalConfig(bool offscreenOnly, int32_t maxNumMaterials,
                                            uint32_t maxNumTextures, uint32_t defaultMipMaps,
                                            std::string const &device, bool doNotLoadTexture) {
  auto &d = Get();
  d.offscreenOnly = offscreenOnly;
  d.defaultMipMaps = defaultMipMaps;
  d.device = device;
  d.doNotLoadTexture = doNotLoadTexture;
  d.maxNumMaterials = maxNumMaterials;
  d.maxNumTextures = maxNumTextures;
}

bool SapienRendererDefault::getOffscreenOnly() { return Get().offscreenOnly; }
uint32_t SapienRendererDefault::getDefaultMipMaps() { return Get().defaultMipMaps; }
std::string SapienRendererDefault::getDevice() { return Get().device; }
bool SapienRendererDefault::getDoNotLoadTexture() { return Get().doNotLoadTexture; }
uint32_t SapienRendererDefault::getMaxNumMaterials() { return Get().maxNumMaterials; }
uint32_t SapienRendererDefault::getMaxNumTextures() { return Get().maxNumTextures; }

void SapienRendererDefault::setViewerShaderDirectory(std::string const &dir) {
  auto path = fs::path(dir);
  if (!fs::is_directory(path)) {
    path = fs::canonical(Get().shaderSearchPath) / dir;
  }
  if (!fs::is_directory(path)) {
    throw std::runtime_error("invalid shader direcotry: " + dir);
  }
  Get().viewerShaderDirectory = fs::canonical(path).string();
}
void SapienRendererDefault::setCameraShaderDirectory(std::string const &dir) {
  auto path = fs::path(dir);
  if (!fs::is_directory(path)) {
    path = fs::canonical(Get().shaderSearchPath) / dir;
  }
  if (!fs::is_directory(path)) {
    throw std::runtime_error("invalid shader direcotry: " + dir);
  }
  Get().cameraShaderDirectory = fs::canonical(path).string();
}
void SapienRendererDefault::setRayTracingSamplesPerPixel(int spp) {
  Get().rayTracingSamplesPerPixel = spp;
}
void SapienRendererDefault::setRayTracingPathDepth(int depth) {
  Get().rayTracingPathDepth = depth;
}
void SapienRendererDefault::setRayTracingDenoiser(
    svulkan2::renderer::RTRenderer::DenoiserType type) {
  Get().rayTracingDenoiserType = type;
}
void SapienRendererDefault::setRayTracingDoFAperture(float radius) {
  Get().rayTracingDoFAperture = radius;
}
void SapienRendererDefault::setRayTracingDoFPlane(float depth) {
  Get().rayTracingDoFPlane = depth;
}
void SapienRendererDefault::setMSAA(int msaa) { Get().msaa = msaa; }

void SapienRendererDefault::setRenderTargetFormat(std::string const &name, vk::Format format) {
  Get().renderTargetFormats[name] = format;
}

std::string SapienRendererDefault::getViewerShaderDirectory() {
  return Get().viewerShaderDirectory;
}
std::string SapienRendererDefault::getCameraShaderDirectory() {
  return Get().cameraShaderDirectory;
}
int SapienRendererDefault::getRayTracingSamplesPerPixel() {
  return Get().rayTracingSamplesPerPixel;
}
int SapienRendererDefault::getRayTracingPathDepth() { return Get().rayTracingPathDepth; }
svulkan2::renderer::RTRenderer::DenoiserType SapienRendererDefault::getRayTracingDenoiser() {
  return Get().rayTracingDenoiserType;
}
float SapienRendererDefault::getRayTracingDoFAperture() { return Get().rayTracingDoFAperture; }
float SapienRendererDefault::getRayTracingDoFPlane() { return Get().rayTracingDoFPlane; }
int SapienRendererDefault::getMSAA() { return Get().msaa; }

SapienRendererDefault &SapienRendererDefault::Get() {
  static SapienRendererDefault gSapienRendererDefault;
  return gSapienRendererDefault;
}

void SapienRendererDefault::internalSetShaderSearchPath(std::string const &dir) {
  Get().shaderSearchPath = dir;
  if (!fs::is_directory(fs::path(dir))) {
    throw std::runtime_error("failed to set shader search path: invalid directory");
  }
}

void SapienRendererDefault::setImguiIniFilename(std::string const &filename) {
  svulkan2::renderer::GuiWindow::setImguiIniFileLocation(filename);
}

std::string SapienRendererDefault::getImguiIniFilename() {
  return svulkan2::renderer::GuiWindow::getImguiIniFileLocation();
}

} // namespace sapien_renderer
} // namespace sapien
