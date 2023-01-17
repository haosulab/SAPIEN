#include "sapien/renderer/render_config.h"

namespace sapien {
namespace Renderer {

std::unique_ptr<RenderConfig> gRenderConfig;
RenderConfig &GetRenderConfig() {
  if (!gRenderConfig) {
    gRenderConfig = std::make_unique<RenderConfig>();
  }
  return *gRenderConfig;
}

} // namespace Renderer
} // namespace sapien
