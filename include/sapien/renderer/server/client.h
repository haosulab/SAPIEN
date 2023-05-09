#pragma once

#include "common.h"
#include "renderer/server/protos/render_server.grpc.pb.h"
#include "sapien/renderer/render_interface.h"
#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>

namespace sapien {
namespace Renderer {
namespace server {

using ::sapien::Renderer::ICamera;
using ::sapien::Renderer::IPxrMaterial;
using ::sapien::Renderer::IPxrRenderer;
using ::sapien::Renderer::IPxrRigidbody;
using ::sapien::Renderer::IPxrScene;
using ::sapien::Renderer::IPxrTexture;

class ClientRenderer;
class ClientScene;
class ClientShape;
class ClientRigidbody;

class ClientRenderer : public IPxrRenderer, public std::enable_shared_from_this<ClientRenderer> {
public:
  ClientRenderer(std::string const &address, uint64_t processIndex);

  IPxrScene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;

  std::shared_ptr<IRenderMesh> createMesh(std::vector<float> const &vertices,
                                          std::vector<uint32_t> const &indices) override {
    throw std::runtime_error("Mesh creation is not supported for rendering client");
  };

  proto::RenderService::Stub &getStub() const { return *mStub; }

  inline uint64_t getProcessIndex() const { return mProcessIndex; }

private:
  uint64_t mProcessIndex;
  std::shared_ptr<grpc::Channel> mChannel;
  std::unique_ptr<proto::RenderService::Stub> mStub;

  std::vector<std::shared_ptr<ClientScene>> mScenes;
};

} // namespace server
} // namespace Renderer
} // namespace sapien
