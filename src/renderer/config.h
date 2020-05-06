#pragma once

namespace sapien {
namespace Renderer {
struct OptifuserConfig {
  bool useShadow = true;
  bool useAo = false;
  int shadowMapSize = 2048;
  float shadowFrustumSize = 10.f;

  inline OptifuserConfig(){};
};
} // namespace Renderer
}; // namespace sapien
