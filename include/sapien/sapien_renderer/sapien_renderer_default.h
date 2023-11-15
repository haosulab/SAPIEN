#pragma once
#include <memory>
#include <string>
#include <svulkan2/common/vk.h>
#include <svulkan2/renderer/rt_renderer.h>
#include <unordered_map>

namespace sapien {
namespace sapien_renderer {

class SapienRendererDefault {
public:
  static SapienRendererDefault &Get();

  static void setImguiIniFilename(std::string const &filename);
  static std::string getImguiIniFilename();

  static void setViewerShaderDirectory(std::string const &dir);
  static void setCameraShaderDirectory(std::string const &dir);
  static void setRayTracingSamplesPerPixel(int spp);
  static void setRayTracingPathDepth(int depth);
  static void setRayTracingDenoiser(svulkan2::renderer::RTRenderer::DenoiserType type);
  static void setRayTracingDoFAperture(float radius);
  static void setRayTracingDoFPlane(float depth);
  static void setMSAA(int msaa);

  static std::string getViewerShaderDirectory();
  static std::string getCameraShaderDirectory();
  static int getRayTracingSamplesPerPixel();
  static int getRayTracingPathDepth();
  static svulkan2::renderer::RTRenderer::DenoiserType getRayTracingDenoiser();
  static float getRayTracingDoFAperture();
  static float getRayTracingDoFPlane();
  static int getMSAA();

  // TODO: set render target format
  static void setRenderTargetFormat(std::string const &name, vk::Format format);

  static void internalSetShaderSearchPath(std::string const &dir);

public:
  std::string viewerShaderDirectory{};
  std::string cameraShaderDirectory{};
  std::unordered_map<std::string, vk::Format> renderTargetFormats;
  int rayTracingSamplesPerPixel{32};
  int rayTracingPathDepth{8};
  int rayTracingRussianRouletteMinBounces{-1};
  svulkan2::renderer::RTRenderer::DenoiserType rayTracingDenoiserType{
      svulkan2::renderer::RTRenderer::DenoiserType::eNONE};
  int msaa{1};

  float rayTracingDoFAperture = 0.f;
  float rayTracingDoFPlane = 1.f;

private:
  std::string shaderSearchPath;
};

} // namespace sapien_renderer
} // namespace sapien
