#pragma once
#include "light_component.h"
#include "render_body_component.h"
#include <variant>

namespace sapien {
namespace sapien_renderer {

struct RenderSceneNode {
  std::string name;
  Pose pose{};
  Vec3 scale{1};

  std::vector<std::unique_ptr<RenderSceneNode>> children;
  std::shared_ptr<RenderShapeTriangleMesh> mesh;
  std::shared_ptr<SapienRenderLightComponent> light;
};

std::unique_ptr<RenderSceneNode> LoadScene(std::string const &filename, bool applyScale = false);

} // namespace sapien_renderer
} // namespace sapien
