#pragma once
#include "light_component.h"
#include "render_body_component.h"
#include <variant>

namespace sapien {
namespace sapien_renderer {

struct RenderSceneLoaderNode {
  std::string name;
  Pose pose{};
  Vec3 scale{1};

  std::vector<std::shared_ptr<RenderSceneLoaderNode>> children;
  std::shared_ptr<RenderShapeTriangleMesh> mesh;
  std::shared_ptr<SapienRenderLightComponent> light;

  std::tuple<std::vector<std::shared_ptr<RenderShapeTriangleMesh>>,
             std::vector<std::shared_ptr<SapienRenderLightComponent>>>
  flatten();
};

std::unique_ptr<RenderSceneLoaderNode> LoadScene(std::string const &filename,
                                                 bool applyScale = false);

} // namespace sapien_renderer
} // namespace sapien
