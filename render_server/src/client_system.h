#pragma once
#include "proto/render_server.grpc.pb.h"
#include "sapien/component/system.h"
#include <grpcpp/create_channel.h>
#include <sapien/math/pose.h>

namespace sapien {
namespace render_server {

class ClientCameraComponent;
class ClientRenderBodyComponent;

class ClientSystem : sapien::component::System {
public:
  ClientSystem(std::string const &address, uint64_t index);

  uint64_t getServerId() { return mServerId; }
  uint64_t getIndex() { return mIndex; }
  grpc::Channel &getChannel() const { return *mChannel; };
  proto::RenderService::Stub &getStub() const { return *mStub; }

  void registerCamera(std::shared_ptr<ClientCameraComponent> camera);
  void registerBody(std::shared_ptr<ClientRenderBodyComponent> body);

  // TODO: make into components
  void setAmbientLight(Vec3 const &color);
  void addPointLight(Vec3 const &position, Vec3 const &color, bool shadow, float shadowNear,
                     float shadowFar, int shadowMapSize);
  void addDirectionalLight(Vec3 const &direction, Vec3 const &color, bool shadow,
                           Vec3 const &position, float shadowScale, float shadowNear,
                           float shadowFar, int shadowMapSize);

  void step() override;

  uint64_t nextRenderId() { return mNextRenderId++; };

  ~ClientSystem();

private:
  uint64_t mIndex;
  uint64_t mServerId;
  std::shared_ptr<grpc::Channel> mChannel;
  std::unique_ptr<proto::RenderService::Stub> mStub;
  uint64_t mNextRenderId{1};

  std::vector<std::shared_ptr<ClientCameraComponent>> mCameras;
  std::vector<std::shared_ptr<ClientRenderBodyComponent>> mRenderBodies;
};

} // namespace render_server
} // namespace sapien
