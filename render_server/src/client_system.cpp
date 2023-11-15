#include "client_system.h"

namespace sapien {
namespace render_server {
using ::grpc::ClientContext;
using ::grpc::Status;

ClientSystem::ClientSystem(std::string const &address, uint64_t index) : mIndex(index) {
  grpc::ChannelArguments args;
  args.SetLoadBalancingPolicyName("round_robin");
  mChannel = CreateCustomChannel(address, grpc::InsecureChannelCredentials(), args);
  mStub = proto::RenderService::NewStub(mChannel);

  ClientContext context;
  proto::Index req;
  proto::Id res;

  req.set_index(mIndex);

  Status status = mStub->CreateScene(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

void ClientSystem::registerCamera(std::shared_ptr<ClientCameraComponent> camera) {
  mCameras.push_back(camera);
}

void ClientSystem::registerBody(std::shared_ptr<ClientRenderBodyComponent> body) {
  mRenderBodies.push_back(body);
}

void ClientSystem::setAmbientLight(Vec3 const &color) {
  grpc::ClientContext context;
  proto::IdVec3 req;
  proto::Empty res;

  req.set_id(mServerId);
  req.mutable_data()->set_x(color.x);
  req.mutable_data()->set_y(color.y);
  req.mutable_data()->set_z(color.z);

  Status status = getStub().SetAmbientLight(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}
void ClientSystem::addPointLight(Vec3 const &position, Vec3 const &color, bool shadow,
                                 float shadowNear, float shadowFar, int shadowMapSize) {
  grpc::ClientContext context;
  proto::AddPointLightReq req;
  proto::Id res;

  req.set_scene_id(mServerId);
  req.mutable_position()->set_x(position.x);
  req.mutable_position()->set_y(position.y);
  req.mutable_position()->set_z(position.z);

  req.mutable_color()->set_x(color.x);
  req.mutable_color()->set_y(color.y);
  req.mutable_color()->set_z(color.z);

  req.set_shadow(shadow);
  req.set_shadow_near(shadowNear);
  req.set_shadow_far(shadowFar);
  req.set_shadow_map_size(shadowMapSize);

  Status status = getStub().AddPointLight(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
  // res.id();
}
void ClientSystem::addDirectionalLight(Vec3 const &direction, Vec3 const &color, bool shadow,
                                       Vec3 const &position, float shadowScale, float shadowNear,
                                       float shadowFar, int shadowMapSize) {
  grpc::ClientContext context;
  proto::AddDirectionalLightReq req;
  proto::Id res;

  req.set_scene_id(mServerId);
  req.mutable_direction()->set_x(direction.x);
  req.mutable_direction()->set_y(direction.y);
  req.mutable_direction()->set_z(direction.z);

  req.mutable_color()->set_x(color.x);
  req.mutable_color()->set_y(color.y);
  req.mutable_color()->set_z(color.z);

  req.mutable_position()->set_x(position.x);
  req.mutable_position()->set_y(position.y);
  req.mutable_position()->set_z(position.z);

  req.set_shadow(shadow);
  req.set_shadow_scale(shadowScale);
  req.set_shadow_near(shadowNear);
  req.set_shadow_far(shadowFar);
  req.set_shadow_map_size(shadowMapSize);

  Status status = getStub().AddDirectionalLight(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
  // res.id()
}

ClientSystem::~ClientSystem() {
  ClientContext context;
  proto::Id req;
  proto::Empty res;

  Status status = mStub->RemoveScene(&context, req, &res);
  if (!status.ok()) {
    // ignore error
  }
}

} // namespace render_server
} // namespace sapien
