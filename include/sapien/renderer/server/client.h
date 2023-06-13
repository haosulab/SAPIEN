#pragma once

#include "common.h"
#include "sapien/renderer/render_interface.h"

namespace sapien {
namespace Renderer {
namespace server {

class ClientRenderer : public IRenderer {
public:
  static std::shared_ptr<ClientRenderer> Create(std::string const &address, uint64_t processIndex);
};

} // namespace server
} // namespace Renderer
} // namespace sapien
