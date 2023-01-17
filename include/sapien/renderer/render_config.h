#pragma once
#include <memory>
#include <string>
#include <unordered_map>

namespace sapien {
namespace Renderer {

struct RenderConfig {
  std::string viewerShaderDirectory{};
  std::string cameraShaderDirectory{};

  std::unordered_map<std::string, std::string> renderTargetFormats;

  int rayTracingSamplesPerPixel{32};
  int rayTracingPathDepth{8};
  int rayTracingRussianRouletteMinBounces{-1};
  bool rayTracingUseDenoiser{false};
};

RenderConfig &GetRenderConfig();

} // namespace Renderer
} // namespace sapien
