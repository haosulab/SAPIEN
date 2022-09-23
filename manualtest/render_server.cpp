#include "sapien/renderer/server/server.h"

int main() {
  sapien::Renderer::server::setDefaultShaderDirectory("../vulkan_shader/ibl");
  sapien::Renderer::server::RenderServer server(5000, 5000, 4, "", false);
  std::string address = "localhost:15003";
  server.start(address);
  sleep(1800);
}
