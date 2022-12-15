#include "sapien/renderer/server/client.h"
#include <spdlog/spdlog.h>

namespace sapien {
namespace Renderer {
namespace server {

using ::grpc::ClientContext;
using ::grpc::Status;

//========== Material ==========//
ClientMaterial::ClientMaterial(std::shared_ptr<ClientRenderer> renderer, rs_id_t id)
    : mRenderer(renderer), mId(id){};

void ClientMaterial::setBaseColor(std::array<float, 4> color) {
  ClientContext context;
  proto::IdVec4 req;
  proto::Empty res;
  proto::Vec4 data;

  req.set_id(mId);
  req.mutable_data()->set_x(color[0]);
  req.mutable_data()->set_y(color[1]);
  req.mutable_data()->set_z(color[2]);
  req.mutable_data()->set_w(color[3]);

  Status status = mRenderer->getStub().SetBaseColor(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

void ClientMaterial::setRoughness(float roughness) {
  ClientContext context;
  proto::IdFloat req;
  proto::Empty res;
  req.set_id(mId);
  req.set_data(roughness);

  Status status = mRenderer->getStub().SetRoughness(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

void ClientMaterial::setSpecular(float specular) {
  ClientContext context;
  proto::IdFloat req;
  proto::Empty res;
  req.set_id(mId);
  req.set_data(specular);

  Status status = mRenderer->getStub().SetSpecular(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

void ClientMaterial::setMetallic(float metallic) {
  ClientContext context;
  proto::IdFloat req;
  proto::Empty res;
  req.set_id(mId);
  req.set_data(metallic);

  Status status = mRenderer->getStub().SetMetallic(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

ClientMaterial::~ClientMaterial() {
  ClientContext context;
  proto::Id req;
  proto::Empty res;
  req.set_id(mId);
  Status status = mRenderer->getStub().RemoveMaterial(&context, req, &res);
  if (!status.ok()) {
    spdlog::get("SAPIEN")->error("remove client material failed. Is the server closed?");
  }
}

//========== Camera ==========//
IPxrScene *ClientCamera::getScene() { return mScene; }
ClientCamera::ClientCamera(ClientScene *scene, rs_id_t id, uint32_t width, uint32_t height, float cx,
                           float cy, float fx, float fy, float near, float far, float skew)
    : mScene(scene), mId(id), mWidth(width), mHeight(height), mCx(cx), mCy(cy), mFx(fy), mFy(fy),
      mNear(near), mFar(far), mSkew(skew) {}
void ClientCamera::takePicture() {
  ClientContext context;
  proto::TakePictureReq req;
  proto::Empty res;
  req.set_scene_id(mScene->getId());
  req.set_camera_id(mId);

  Status status = mScene->getRenderer()->getStub().TakePicture(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

void ClientCamera::setPerspectiveCameraParameters(float near, float far, float fx, float fy,
                                                  float cx, float cy, float skew) {
  ClientContext context;
  proto::CameraParamsReq req;
  proto::Empty res;

  req.set_scene_id(mScene->getId());
  req.set_camera_id(mId);
  req.set_near(near);
  req.set_far(far);
  req.set_fx(fx);
  req.set_fy(fy);
  req.set_cx(cx);
  req.set_cy(cy);
  req.set_skew(skew);

  Status status = mScene->getRenderer()->getStub().SetCameraParameters(&context, req, &res);
  if (status.ok()) {
    mNear = near;
    mFar = far;
    mFx = fx;
    mFy = fy;
    mCx = cx;
    mCy = cy;
    mSkew = skew;
    return;
  }
  throw std::runtime_error(status.error_message());
}

//========== Shape ==========//

ClientShape::ClientShape(ClientRigidbody *body, uint32_t index) : mBody(body), mIndex(index) {}

std::shared_ptr<IPxrMaterial> ClientShape::getMaterial() const {
  ClientContext context;
  proto::BodyUint32Req req;
  proto::Id res;
  req.set_scene_id(mBody->getScene()->getId());
  req.set_body_id(mBody->getId());
  req.set_id(mIndex);

  Status status =
      mBody->getScene()->getRenderer()->getStub().GetShapeMaterial(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
  return std::make_shared<ClientMaterial>(mBody->getScene()->getRenderer()->shared_from_this(),
                                          res.id());
}

//========== Body ==========//
ClientRigidbody::ClientRigidbody(ClientScene *scene, rs_id_t id) : mScene(scene), mId(id) {}

void ClientRigidbody::setUniqueId(uint32_t uniqueId) {
  mUniqueId = uniqueId;
  ClientContext context;
  proto::BodyIdReq req;
  proto::Empty res;
  req.set_scene_id(mScene->getId());
  req.set_body_id(mId);
  req.set_id(uniqueId);

  Status status = mScene->getRenderer()->getStub().SetUniqueId(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}
uint32_t ClientRigidbody::getUniqueId() const { return mUniqueId; }
void ClientRigidbody::setSegmentationId(uint32_t segmentationId) {
  mSegmentationId = segmentationId;

  ClientContext context;
  proto::BodyIdReq req;
  proto::Empty res;
  req.set_scene_id(mScene->getId());
  req.set_body_id(mId);
  req.set_id(segmentationId);

  Status status = mScene->getRenderer()->getStub().SetSegmentationId(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}
uint32_t ClientRigidbody::getSegmentationId() const { return mSegmentationId; }

void ClientRigidbody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
}

void ClientRigidbody::update(const physx::PxTransform &transform) {
  mCurrentPose = transform * mInitialPose;
}

void ClientRigidbody::destroy() { mScene->removeRigidbody(this); }

std::vector<std::shared_ptr<IPxrRenderShape>> ClientRigidbody::getRenderShapes() {
  ClientContext context;
  proto::BodyReq req;
  proto::Uint32 res;
  req.set_scene_id(mScene->getId());
  req.set_body_id(mId);
  Status status = mScene->getRenderer()->getStub().GetShapeCount(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
  uint32_t count = res.value();

  std::vector<std::shared_ptr<IPxrRenderShape>> shapes;
  for (uint32_t i = 0; i < count; ++i) {
    shapes.push_back(std::make_shared<ClientShape>(this, i));
  }
  return shapes;
}

//========== Scene ==========//
ClientScene::ClientScene(ClientRenderer *renderer, rs_id_t id, std::string const &name)
    : mRenderer(renderer), mId(id), mName(name) {}

IPxrRigidbody *ClientScene::addRigidbody(physx::PxGeometryType::Enum type,
                                         const physx::PxVec3 &scale, const physx::PxVec3 &color) {
  auto mat = mRenderer->createMaterial();
  mat->setBaseColor({color.x, color.y, color.z, 1.f});
  return addRigidbody(type, scale, mat);
}

IPxrRigidbody *ClientScene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) {
  mIdSynced = false;
  ClientContext context;
  proto::AddBodyMeshReq req;
  proto::Id res;

  req.set_scene_id(mId);
  req.set_filename(meshFile);
  req.mutable_scale()->set_x(scale.x);
  req.mutable_scale()->set_y(scale.y);
  req.mutable_scale()->set_z(scale.z);

  Status status = mRenderer->getStub().AddBodyMesh(&context, req, &res);
  if (status.ok()) {
    mBodies.push_back(std::make_unique<ClientRigidbody>(this, res.id()));
    return mBodies.back().get();
  }
  throw std::runtime_error(status.error_message());
}
IPxrRigidbody *ClientScene::addRigidbody(physx::PxGeometryType::Enum type,
                                         const physx::PxVec3 &scale,
                                         std::shared_ptr<IPxrMaterial> material) {
  if (!material) {
    material = mRenderer->createMaterial();
  }

  mIdSynced = false;
  ClientContext context;
  proto::AddBodyPrimitiveReq req;
  proto::Id res;

  req.set_scene_id(mId);
  req.mutable_scale()->set_x(scale.x);
  req.mutable_scale()->set_y(scale.y);
  req.mutable_scale()->set_z(scale.z);

  switch (type) {
  case physx::PxGeometryType::eBOX:
    req.set_type(proto::PrimitiveType::BOX);
    break;
  case physx::PxGeometryType::eCAPSULE:
    req.set_type(proto::PrimitiveType::CAPSULE);
    break;
  case physx::PxGeometryType::eSPHERE:
    req.set_type(proto::PrimitiveType::SPHERE);
    break;
  case physx::PxGeometryType::ePLANE:
    req.set_type(proto::PrimitiveType::PLANE);
    break;
  default:
    throw std::runtime_error("failed to add primitive body: invalid primitve type");
  }

  if (auto mat = std::dynamic_pointer_cast<ClientMaterial>(material)) {
    req.set_material(mat->getId());
  } else {
    throw std::runtime_error("failed to add primitive body: invalid material");
  }

  Status status = mRenderer->getStub().AddBodyPrimitive(&context, req, &res);
  if (status.ok()) {
    mBodies.push_back(std::make_unique<ClientRigidbody>(this, res.id()));
    return mBodies.back().get();
  }
  throw std::runtime_error(status.error_message());
}

void ClientScene::removeRigidbody(IPxrRigidbody *body) {
  mIdSynced = false;
  if (ClientRigidbody *b = dynamic_cast<ClientRigidbody *>(body)) {
    ClientContext context;
    proto::RemoveBodyReq req;
    proto::Empty res;
    req.set_scene_id(mId);
    req.set_body_id(b->getId());

    Status status = mRenderer->getStub().RemoveBody(&context, req, &res);
    if (!status.ok()) {
      throw std::runtime_error(status.error_message());
    }
  }
  throw std::runtime_error("failed to remove body: invalid body");
}

ClientCamera *ClientScene::addCamera(uint32_t width, uint32_t height, float fovy, float near,
                                     float far, std::string const &shaderDir) {
  mIdSynced = false;
  ClientContext context;
  proto::AddCameraReq req;
  proto::Id res;

  req.set_scene_id(mId);
  req.set_width(width);
  req.set_height(height);
  req.set_fovy(fovy);
  req.set_near(near);
  req.set_far(far);
  req.set_shader(shaderDir);

  Status status = mRenderer->getStub().AddCamera(&context, req, &res);
  if (status.ok()) {
    float cx = width / 2.f;
    float cy = height / 2.f;
    float fy = width / 2.f / std::tan(fovy / 2.f);
    float fx = fy;
    float skew = 0.f;
    mCameras.push_back(std::make_unique<ClientCamera>(this, res.id(), width, height, cx, cy, fx,
                                                      fy, near, far, skew));

    return mCameras.back().get();
  }
  throw std::runtime_error(status.error_message());
};

std::vector<ICamera *> ClientScene::getCameras() {
  std::vector<ICamera *> result;
  for (auto &cam : mCameras) {
    result.push_back(cam.get());
  }
  return result;
}

void ClientScene::setAmbientLight(std::array<float, 3> const &color) {
  ClientContext context;
  proto::IdVec3 req;
  proto::Empty res;

  req.set_id(mId);
  req.mutable_data()->set_x(color[0]);
  req.mutable_data()->set_y(color[1]);
  req.mutable_data()->set_z(color[2]);

  Status status = mRenderer->getStub().SetAmbientLight(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

IPointLight *ClientScene::addPointLight(std::array<float, 3> const &position,
                                        std::array<float, 3> const &color, bool enableShadow,
                                        float shadowNear, float shadowFar,
                                        uint32_t shadowMapSize) {
  ClientContext context;
  proto::AddPointLightReq req;
  proto::Id res;

  req.set_scene_id(mId);
  req.mutable_position()->set_x(position[0]);
  req.mutable_position()->set_y(position[1]);
  req.mutable_position()->set_z(position[2]);

  req.mutable_color()->set_x(color[0]);
  req.mutable_color()->set_y(color[1]);
  req.mutable_color()->set_z(color[2]);

  req.set_shadow(enableShadow);
  req.set_shadow_near(shadowNear);
  req.set_shadow_far(shadowFar);
  req.set_shadow_map_size(shadowMapSize);

  Status status = mRenderer->getStub().AddPointLight(&context, req, &res);
  if (status.ok()) {
    auto light = std::make_unique<ClientPointLight>(res.id());
    auto result = light.get();
    mLights.push_back(std::move(light));
    return result;
  }
  throw std::runtime_error(status.error_message());
}

IDirectionalLight *ClientScene::addDirectionalLight(std::array<float, 3> const &direction,
                                                    std::array<float, 3> const &color,
                                                    bool enableShadow,
                                                    std::array<float, 3> const &position,
                                                    float shadowScale, float shadowNear,
                                                    float shadowFar, uint32_t shadowMapSize) {
  ClientContext context;
  proto::AddDirectionalLightReq req;
  proto::Id res;

  req.set_scene_id(mId);
  req.mutable_direction()->set_x(direction[0]);
  req.mutable_direction()->set_y(direction[1]);
  req.mutable_direction()->set_z(direction[2]);

  req.mutable_color()->set_x(color[0]);
  req.mutable_color()->set_y(color[1]);
  req.mutable_color()->set_z(color[2]);

  req.mutable_position()->set_x(position[0]);
  req.mutable_position()->set_y(position[1]);
  req.mutable_position()->set_z(position[2]);

  req.set_shadow(enableShadow);
  req.set_shadow_scale(shadowScale);
  req.set_shadow_near(shadowNear);
  req.set_shadow_far(shadowFar);
  req.set_shadow_map_size(shadowMapSize);

  Status status = mRenderer->getStub().AddDirectionalLight(&context, req, &res);
  if (status.ok()) {
    auto light = std::make_unique<ClientDirectionalLight>(res.id());
    auto result = light.get();
    mLights.push_back(std::move(light));
    return result;
  }
  throw std::runtime_error(status.error_message());
}

void ClientScene::updateRender() {
  syncId();

  ClientContext context;
  proto::UpdateRenderReq req;
  proto::Empty res;

  req.set_scene_id(mId);
  for (auto &body : mBodies) {
    auto &pose = body->getCurrentPose();
    auto p = req.add_body_poses();
    p->mutable_p()->set_x(pose.p.x);
    p->mutable_p()->set_y(pose.p.y);
    p->mutable_p()->set_z(pose.p.z);

    p->mutable_q()->set_w(pose.q.w);
    p->mutable_q()->set_x(pose.q.x);
    p->mutable_q()->set_y(pose.q.y);
    p->mutable_q()->set_z(pose.q.z);
  }

  for (auto &cam : mCameras) {
    auto pose = cam->getPose();
    auto p = req.add_camera_poses();
    p->mutable_p()->set_x(pose.p.x);
    p->mutable_p()->set_y(pose.p.y);
    p->mutable_p()->set_z(pose.p.z);

    p->mutable_q()->set_w(pose.q.w);
    p->mutable_q()->set_x(pose.q.x);
    p->mutable_q()->set_y(pose.q.y);
    p->mutable_q()->set_z(pose.q.z);
  }

  Status status = mRenderer->getStub().UpdateRender(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
};

void ClientScene::updateRenderAndTakePictures(std::vector<ICamera *> const &cameras) {
  syncId();

  ClientContext context;
  proto::UpdateRenderAndTakePicturesReq req;
  proto::Empty res;

  req.set_scene_id(mId);
  for (auto &body : mBodies) {
    auto &pose = body->getCurrentPose();
    auto p = req.add_body_poses();
    p->mutable_p()->set_x(pose.p.x);
    p->mutable_p()->set_y(pose.p.y);
    p->mutable_p()->set_z(pose.p.z);

    p->mutable_q()->set_w(pose.q.w);
    p->mutable_q()->set_x(pose.q.x);
    p->mutable_q()->set_y(pose.q.y);
    p->mutable_q()->set_z(pose.q.z);
  }

  for (auto &cam : mCameras) {
    auto pose = cam->getPose();
    auto p = req.add_camera_poses();
    p->mutable_p()->set_x(pose.p.x);
    p->mutable_p()->set_y(pose.p.y);
    p->mutable_p()->set_z(pose.p.z);

    p->mutable_q()->set_w(pose.q.w);
    p->mutable_q()->set_x(pose.q.x);
    p->mutable_q()->set_y(pose.q.y);
    p->mutable_q()->set_z(pose.q.z);
  }

  for (auto cam : cameras) {
    if (auto c = dynamic_cast<ClientCamera *>(cam)) {
      req.add_camera_ids(c->getId());
    } else {
      throw std::runtime_error("invalid camera");
    }
  }
  Status status = mRenderer->getStub().UpdateRenderAndTakePictures(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
}

void ClientScene::destroy() { getRenderer()->removeScene(this); }

void ClientScene::syncId() {
  if (mIdSynced) {
    return;
  }
  ClientContext context;
  proto::EntityOrderReq req;
  proto::Empty res;

  req.set_scene_id(mId);
  for (auto &body : mBodies) {
    req.add_body_ids(body->getId());
  }

  for (auto &cam : mCameras) {
    req.add_camera_ids(cam->getId());
  }

  Status status = mRenderer->getStub().SetEntityOrder(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error(status.error_message());
  }
  mIdSynced = true;
}

//========== Renderer ==========//
ClientRenderer::ClientRenderer(std::string const &address, uint64_t processIndex)
    : mProcessIndex(processIndex) {
  grpc::ChannelArguments args;
  args.SetLoadBalancingPolicyName("round_robin");
  mChannel = grpc::CreateCustomChannel(address, grpc::InsecureChannelCredentials(), args);
  mStub = proto::RenderService::NewStub(mChannel);
}

ClientScene *ClientRenderer::createScene(std::string const &name) {
  ClientContext context;
  proto::Index req;
  proto::Id res;

  req.set_index(mProcessIndex);

  Status status = mStub->CreateScene(&context, req, &res);
  if (status.ok()) {
    mScenes.push_back(std::make_unique<ClientScene>(this, res.id(), name));
    return mScenes.back().get();
  }
  throw std::runtime_error(status.error_message());
}

void ClientRenderer::removeScene(IPxrScene *scene) {
  if (ClientScene *clientScene = dynamic_cast<ClientScene *>(scene)) {
    ClientContext context;
    proto::Id req;
    proto::Empty res;
    req.set_id(clientScene->getId());

    Status status = mStub->RemoveScene(&context, req, &res);
    if (status.ok()) {
      std::erase_if(mScenes, [=](auto const &s) { return s.get() == clientScene; });
      return;
    }
    throw std::runtime_error(status.error_message());
  }
}

std::shared_ptr<IPxrMaterial> ClientRenderer::createMaterial() {
  ClientContext context;
  proto::Empty req;
  proto::Id res;

  Status status = mStub->CreateMaterial(&context, req, &res);
  if (status.ok()) {
    auto mat = std::make_shared<ClientMaterial>(shared_from_this(), res.id());
    return mat;
  }
  throw std::runtime_error(status.error_message());
};

} // namespace server
} // namespace Renderer
} // namespace sapien
