/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sapien/sapien_renderer/sapien_renderer_default.h"
#include <filesystem>
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/vr.h>

namespace fs = std::filesystem;

namespace sapien {
namespace sapien_renderer {

void SapienRendererDefault::setGlobalConfig(int32_t maxNumMaterials, uint32_t maxNumTextures,
                                            uint32_t defaultMipMaps, bool doNotLoadTexture) {
  auto &d = Get();

  d.defaultMipMaps = defaultMipMaps;
  d.doNotLoadTexture = doNotLoadTexture;
  d.maxNumMaterials = maxNumMaterials;
  d.maxNumTextures = maxNumTextures;
}

uint32_t SapienRendererDefault::getDefaultMipMaps() { return Get().defaultMipMaps; }

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

void SapienRendererDefault::setVRActionManifestFilename(std::string const &filename) {
  svulkan2::renderer::VRDisplay::setActionManifestPath(filename);
}
std::string SapienRendererDefault::getVRActionManifestFilename() {
  return svulkan2::renderer::VRDisplay::getActionManifestPath();
}

void SapienRendererDefault::enableVR() { Get().vrEnabled = true; }
bool SapienRendererDefault::getVREnabled() { return Get().vrEnabled; }

} // namespace sapien_renderer
} // namespace sapien
