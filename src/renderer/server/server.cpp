#include "sapien/renderer/server/server.h"
#include <algorithm>
#include <easy/profiler.h>
#include <spdlog/spdlog.h>
#include <string>

namespace sapien {
namespace Renderer {
namespace server {

// namespace log {
// std::shared_ptr<spdlog::logger> getLogger() { return spdlog::get("RenderServer"); }

// template <typename... Args> inline void debug(spdlog::string_view_t fmt, const Args &...args) {
//   getLogger()->debug(fmt, args...);
// };

// template <typename... Args> inline void info(spdlog::string_view_t fmt, const Args &...args) {
//   getLogger()->info(fmt, args...);
// };

// template <typename... Args> inline void warn(spdlog::string_view_t fmt, const Args &...args) {
//   getLogger()->warn(fmt, args...);
// };

// template <typename... Args> inline void error(spdlog::string_view_t fmt, const Args &...args) {
//   getLogger()->error(fmt, args...);
// };

// template <typename... Args> inline void critical(spdlog::string_view_t fmt, const Args &...args)
// {
//   getLogger()->critical(fmt, args...);
// };
// } // namespace log

// disable logger
namespace log {
template <typename... Args> inline void debug(spdlog::string_view_t fmt, const Args &...args){};

template <typename... Args> inline void info(spdlog::string_view_t fmt, const Args &...args){};

template <typename... Args> inline void warn(spdlog::string_view_t fmt, const Args &...args){};

template <typename... Args> inline void error(spdlog::string_view_t fmt, const Args &...args){};

template <typename... Args> inline void critical(spdlog::string_view_t fmt, const Args &...args){};
} // namespace log

typedef std::unique_lock<std::shared_mutex> WriteLock;
typedef std::shared_lock<std::shared_mutex> ReadLock;

std::string gDefaultShaderDirectory;
void setDefaultShaderDirectory(std::string const &dir) { gDefaultShaderDirectory = dir; }

// ========== Renderer ==========//
Status RenderServiceImpl::CreateScene(ServerContext *c, const proto::Index *req, proto::Id *res) {
  log::info("CreateScene");
  auto index = req->index();
  id_t id = generateId();

  auto info = std::make_shared<SceneInfo>();
  info->sceneIndex = index;
  info->sceneId = id;
  info->scene = std::make_unique<svulkan2::scene::Scene>();
  info->threadRunner = std::make_unique<ThreadPool>(1);
  info->threadRunner->init();

  mSceneMap.set(id, info);

  {
    WriteLock lock(mSceneListLock);
    if (mSceneList.size() <= index) {
      mSceneList.resize(index + 1, nullptr);
    }
    mSceneList[index] = info;
  }

  res->set_id(id);

  log::info("Scene Created: {}", id);
  return Status::OK;
}

Status RenderServiceImpl::RemoveScene(ServerContext *c, const proto::Id *req, proto::Empty *res) {
  log::info("RemoveScene {}", req->id());
  // TODO: make sure nothing is running
  auto info = mSceneMap.get(req->id());

  {
    WriteLock lock(mSceneListLock);
    if (mSceneList.at(info->sceneIndex) == info) {
      mSceneList[info->sceneIndex] = nullptr;
    }
  }

  mSceneMap.erase(req->id());

  return Status::OK;
}

Status RenderServiceImpl::CreateMaterial(ServerContext *c, const proto::Empty *req,
                                         proto::Id *res) {
  log::info("CreateMaterial");
  id_t id = generateId();

  auto mat = std::make_shared<svulkan2::resource::SVMetallicMaterial>();
  mat->setBaseColor({1.0, 1.0, 1.0, 1.0});

  mMaterialMap.set(id, mat);

  res->set_id(id);
  log::info("Material Created {}", res->id());
  return Status::OK;
}

Status RenderServiceImpl::RemoveMaterial(ServerContext *c, const proto::Id *req,
                                         proto::Empty *res) {
  log::info("RemoveMaterial {}", req->id());
  mMaterialMap.erase(req->id());
  return Status::OK;
}

// ========== Scene ==========//
Status RenderServiceImpl::AddBodyMesh(ServerContext *c, const proto::AddBodyMeshReq *req,
                                      proto::Id *res) {
  id_t id = generateId();

  auto info = mSceneMap.get(req->scene_id());
  svulkan2::scene::Object *object =
      &info->scene->addObject(mResourceManager->CreateModelFromFile(req->filename()));
  info->objectMap[id] = object;

  res->set_id(id);
  return Status::OK;
}

Status RenderServiceImpl::AddBodyPrimitive(ServerContext *c, const proto::AddBodyPrimitiveReq *req,
                                           proto::Id *res) {
  id_t id = generateId();

  glm::vec3 scale{req->scale().x(), req->scale().y(), req->scale().z()};
  auto mat = mMaterialMap.get(req->material());
  auto info = mSceneMap.get(req->scene_id());

  svulkan2::scene::Object *object;
  switch (req->type()) {
  case proto::PrimitiveType::BOX: {
    auto shape = svulkan2::resource::SVShape::Create(mCubeMesh, mat);
    object = &info->scene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    object->setScale({scale.x, scale.y, scale.z});
    break;
  }

  case proto::PrimitiveType::SPHERE: {
    auto shape = svulkan2::resource::SVShape::Create(mSphereMesh, mat);
    object = &info->scene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    object->setScale({scale.x, scale.y, scale.z});
    break;
  }

  case proto::PrimitiveType::PLANE: {
    auto shape = svulkan2::resource::SVShape::Create(mPlaneMesh, mat);
    object = &info->scene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    object->setScale({scale.x, scale.y, scale.z});
    break;
  }

  case proto::PrimitiveType::CAPSULE: {
    auto mesh = svulkan2::resource::SVMesh::CreateCapsule(scale.y, scale.x, 32, 8);
    auto shape = svulkan2::resource::SVShape::Create(mesh, mat);
    object = &info->scene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    object->setScale({scale.x, scale.y, scale.z});
    break;
  }
  default:
    throw std::runtime_error("this should never happen");
  }

  info->objectMap[id] = object;

  res->set_id(id);
  return Status::OK;
}

Status RenderServiceImpl::RemoveBody(ServerContext *c, const proto::RemoveBodyReq *req,
                                     proto::Empty *res) {

  auto info = mSceneMap.get(req->scene_id());

  {
    auto it = info->objectMap.find(req->body_id());
    info->scene->removeNode(*it->second);
    info->objectMap.erase(it);
  }

  return Status::OK;
}

Status RenderServiceImpl::AddCamera(ServerContext *c, const proto::AddCameraReq *req,
                                    proto::Id *res) {
  log::info("AddCamera");
  try {

    id_t id = generateId();

    auto sceneInfo = mSceneMap.get(req->scene_id());

    uint64_t cameraIndex = sceneInfo->cameraMap.size();
    auto camInfo = std::make_shared<CameraInfo>();
    camInfo->cameraIndex = cameraIndex;

    sceneInfo->cameraMap[id] = camInfo;
    sceneInfo->cameraList.push_back(camInfo);

    auto config = std::make_shared<svulkan2::RendererConfig>();
    config->colorFormat4 = vk::Format::eR32G32B32A32Sfloat;
    config->depthFormat = vk::Format::eD32Sfloat;
    config->shaderDir = req->shader().empty() ? gDefaultShaderDirectory : req->shader();

    camInfo->renderer = std::make_unique<svulkan2::renderer::Renderer>(config);
    camInfo->renderer->resize(req->width(), req->height());
    camInfo->renderer->setScene(*sceneInfo->scene);

    camInfo->camera = &sceneInfo->scene->addCamera();
    camInfo->camera->setPerspectiveParameters(req->near(), req->far(), req->fovy(), req->width(),
                                              req->height());

    camInfo->semaphore = mContext->createTimelineSemaphore(0);
    camInfo->frameCounter = 0;

    camInfo->commandPool = mContext->createCommandPool();
    camInfo->commandBuffer = camInfo->commandPool->allocateCommandBuffer();

    res->set_id(id);
    log::info("Camera Added {}", id);
  } catch (const std::exception &e) {
    std::cerr << "Render server failed: " << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INTERNAL, e.what());
  }
  return Status::OK;
}

Status RenderServiceImpl::SetAmbientLight(ServerContext *c, const proto::IdVec3 *req,
                                          proto::Empty *res) {
  mSceneMap.get(req->id())->scene->setAmbientLight(
      {req->data().x(), req->data().y(), req->data().z(), 1.0});
  return Status::OK;
}

Status RenderServiceImpl::AddPointLight(ServerContext *c, const proto::AddPointLightReq *req,
                                        proto::Id *res) {
  id_t id = generateId(); // TODO: implement remove light
  auto info = mSceneMap.get(req->scene_id());
  auto &light = info->scene->addPointLight();

  glm::vec3 pos = {req->position().x(), req->position().y(), req->position().z()};
  glm::vec3 color = {req->color().x(), req->color().y(), req->color().z()};
  light.setPosition(pos);
  light.setColor(color);
  light.enableShadow(req->shadow());
  light.setShadowParameters(req->shadow_near(), req->shadow_far(), req->shadow_map_size());

  res->set_id(id);
  return Status::OK;
}

Status RenderServiceImpl::AddDirectionalLight(ServerContext *c,
                                              const proto::AddDirectionalLightReq *req,
                                              proto::Id *res) {
  id_t id = generateId(); // TODO: implement remove light

  auto info = mSceneMap.get(req->scene_id());
  auto &light = info->scene->addDirectionalLight();

  glm::vec3 dir = {req->direction().x(), req->direction().y(), req->direction().z()};
  glm::vec3 pos = {req->position().x(), req->position().y(), req->position().z()};
  glm::vec3 color = {req->color().x(), req->color().y(), req->color().z()};

  light.setDirection(dir);
  light.setColor(color);
  light.enableShadow(req->shadow());
  light.setPosition(pos);
  light.setShadowParameters(req->shadow_near(), req->shadow_far(), req->shadow_scale(),
                            req->shadow_map_size());

  res->set_id(id);
  return Status::OK;
}

Status RenderServiceImpl::SetEntityOrder(ServerContext *c, const proto::EntityOrderReq *req,
                                         proto::Empty *res) {

  {
    auto info = mSceneMap.get(req->scene_id());
    info->orderedCameras.clear();
    info->orderedObjects.clear();

    info->orderedObjects.reserve(req->body_ids_size());
    for (int i = 0; i < req->body_ids_size(); ++i) {
      info->orderedObjects.push_back(info->objectMap[req->body_ids(i)]);
    }

    info->orderedCameras.reserve(req->camera_ids_size());
    for (int i = 0; i < req->camera_ids_size(); ++i) {
      info->orderedCameras.push_back(info->cameraMap.at(req->camera_ids(i))->camera);
    }
  }

  return Status::OK;
}

Status RenderServiceImpl::UpdateRender(ServerContext *c, const proto::UpdateRenderReq *req,
                                       proto::Empty *res) {
  EASY_FUNCTION();

  auto info = mSceneMap.get(req->scene_id());

  for (int i = 0; i < req->body_poses_size(); ++i) {
    glm::vec3 p{req->body_poses(i).p().x(), req->body_poses(i).p().y(),
                req->body_poses(i).p().z()};
    glm::quat q{req->body_poses(i).q().w(), req->body_poses(i).q().x(), req->body_poses(i).q().y(),
                req->body_poses(i).q().z()};
    info->orderedObjects[i]->setPosition(p);
    info->orderedObjects[i]->setRotation(q);
  }

  for (int i = 0; i < req->camera_poses_size(); ++i) {
    glm::vec3 p{req->camera_poses(i).p().x(), req->camera_poses(i).p().y(),
                req->camera_poses(i).p().z()};
    glm::quat q{req->camera_poses(i).q().w(), req->camera_poses(i).q().x(),
                req->camera_poses(i).q().y(), req->camera_poses(i).q().z()};
    info->orderedCameras[i]->setPosition(p);
    info->orderedCameras[i]->setRotation(q);
  }

  info->scene->getRootNode().updateGlobalModelMatrixRecursive(); // TODO: check this

  return Status::OK;
}

Status RenderServiceImpl::UpdateRenderAndTakePictures(
    ServerContext *c, const proto::UpdateRenderAndTakePicturesReq *req, proto::Empty *res) {
  auto sceneInfo = mSceneMap.get(req->scene_id());

  for (int i = 0; i < req->body_poses_size(); ++i) {
    glm::vec3 p{req->body_poses(i).p().x(), req->body_poses(i).p().y(),
                req->body_poses(i).p().z()};
    glm::quat q{req->body_poses(i).q().w(), req->body_poses(i).q().x(), req->body_poses(i).q().y(),
                req->body_poses(i).q().z()};
    sceneInfo->orderedObjects[i]->setPosition(p);
    sceneInfo->orderedObjects[i]->setRotation(q);
  }

  for (int i = 0; i < req->camera_poses_size(); ++i) {
    glm::vec3 p{req->camera_poses(i).p().x(), req->camera_poses(i).p().y(),
                req->camera_poses(i).p().z()};
    glm::quat q{req->camera_poses(i).q().w(), req->camera_poses(i).q().x(),
                req->camera_poses(i).q().y(), req->camera_poses(i).q().z()};
    sceneInfo->orderedCameras[i]->setPosition(p);
    sceneInfo->orderedCameras[i]->setRotation(q);
  }

  sceneInfo->scene->getRootNode().updateGlobalModelMatrixRecursive(); // TODO: check this

  for (int i = 0; i < req->camera_ids_size(); ++i) {
    uint64_t camera_id = req->camera_ids(i);
    auto camInfo = sceneInfo->cameraMap.at(camera_id);
    camInfo->frameCounter++;

    sceneInfo->threadRunner->submit(
        [context = mContext, sem = camInfo->semaphore.get(), cb = camInfo->commandBuffer.get(),
         renderer = camInfo->renderer.get(), cam = camInfo->camera, fillInfo = camInfo->fillInfo,
         frame = camInfo->frameCounter]() {
          uint64_t waitFrame = frame - 1;
          auto result = context->getDevice().waitSemaphores(
              vk::SemaphoreWaitInfo({}, sem, waitFrame), UINT64_MAX);
          if (result != vk::Result::eSuccess) {
            throw std::runtime_error("take picture failed: wait failed");
          }
          cb.reset();
          cb.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
          renderer->render(*cam, {}, {}, {}, {});

          for (auto &entry : fillInfo) {
            auto [name, buffer, offset] = entry;
            auto target = renderer->getRenderTarget(name);
            auto extent = target->getImage().getExtent();
            vk::Format format = target->getFormat();
            vk::DeviceSize size =
                extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);
            target->getImage().recordCopyToBuffer(cb, buffer, offset, size, vk::Offset3D{0, 0, 0},
                                                  extent);
          }
          cb.end();
          context->getQueue().submit(cb, {}, {}, {}, sem, frame, {});
        });
  }
  return Status::OK;
}

// ========== Material ==========//
Status RenderServiceImpl::SetBaseColor(ServerContext *c, const proto::IdVec4 *req,
                                       proto::Empty *res) {
  mMaterialMap.get(req->id())->setBaseColor(
      {req->data().x(), req->data().y(), req->data().z(), req->data().w()});

  return Status::OK;
}

Status RenderServiceImpl::SetRoughness(ServerContext *c, const proto::IdFloat *req,
                                       proto::Empty *res) {
  mMaterialMap.get(req->id())->setRoughness(req->data());

  return Status::OK;
}

Status RenderServiceImpl::SetSpecular(ServerContext *c, const proto::IdFloat *req,
                                      proto::Empty *res) {
  mMaterialMap.get(req->id())->setFresnel(req->data());
  return Status::OK;
}

Status RenderServiceImpl::SetMetallic(ServerContext *c, const proto::IdFloat *req,
                                      proto::Empty *res) {
  mMaterialMap.get(req->id())->setMetallic(req->data());
  return Status::OK;
}

// ========== Body ==========//
Status RenderServiceImpl::SetUniqueId(ServerContext *c, const proto::BodyIdReq *req,
                                      proto::Empty *res) {

  auto info = mSceneMap.get(req->scene_id());
  auto obj = info->objectMap.at(req->body_id());

  glm::vec4 seg = obj->getSegmentation();
  seg[0] = req->id();
  obj->setSegmentation(seg);

  return Status::OK;
}

Status RenderServiceImpl::SetSegmentationId(ServerContext *c, const proto::BodyIdReq *req,
                                            proto::Empty *res) {
  {
    auto info = mSceneMap.get(req->scene_id());
    auto obj = info->objectMap.at(req->body_id());

    glm::vec4 seg = obj->getSegmentation();
    seg[1] = req->id();
    obj->setSegmentation(seg);
  }
  return Status::OK;
}

// ========== Camera ==========//
Status RenderServiceImpl::TakePicture(ServerContext *c, const proto::TakePictureReq *req,
                                      proto::Empty *res) {
  EASY_FUNCTION();
  log::info("TakePicture {} {}", req->scene_id(), req->camera_id());

  auto sceneInfo = mSceneMap.get(req->scene_id());
  auto camInfo = sceneInfo->cameraMap.at(req->camera_id());
  camInfo->frameCounter++;

  // auto context = mContext;
  // auto sem = camInfo->semaphore.get();
  // auto cb = camInfo->commandBuffer.get();
  // auto renderer = camInfo->renderer.get();
  // auto cam = camInfo->camera;
  // auto fillInfo = camInfo->fillInfo;
  // auto frame = camInfo->frameCounter;

  sceneInfo->threadRunner->submit([context = mContext, sem = camInfo->semaphore.get(),
                                   cb = camInfo->commandBuffer.get(),
                                   renderer = camInfo->renderer.get(), cam = camInfo->camera,
                                   fillInfo = camInfo->fillInfo, frame = camInfo->frameCounter]() {
    uint64_t waitFrame = frame - 1;
    auto result =
        context->getDevice().waitSemaphores(vk::SemaphoreWaitInfo({}, sem, waitFrame), UINT64_MAX);
    if (result != vk::Result::eSuccess) {
      throw std::runtime_error("take picture failed: wait failed");
    }
    cb.reset();
    cb.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    renderer->render(*cam, {}, {}, {}, {});

    for (auto &entry : fillInfo) {
      auto [name, buffer, offset] = entry;
      auto target = renderer->getRenderTarget(name);
      auto extent = target->getImage().getExtent();
      vk::Format format = target->getFormat();
      vk::DeviceSize size =
          extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);
      target->getImage().recordCopyToBuffer(cb, buffer, offset, size, vk::Offset3D{0, 0, 0},
                                            extent);
    }
    cb.end();
    context->getQueue().submit(cb, {}, {}, {}, sem, frame, {});
  });

  return Status::OK;
}

Status RenderServiceImpl::SetCameraParameters(ServerContext *c, const proto::CameraParamsReq *req,
                                              proto::Empty *res) {
  log::info("SetCameraParameters {} {}", req->scene_id(), req->camera_id());

  auto info = mSceneMap.get(req->scene_id());
  auto cam = info->cameraMap.at(req->camera_id())->camera;
  cam->setPerspectiveParameters(req->near(), req->far(), req->fx(), req->fy(), req->cx(),
                                req->cy(), cam->getWidth(), cam->getHeight(), req->skew());

  return Status::OK;
}

RenderServiceImpl::RenderServiceImpl(
    std::shared_ptr<svulkan2::core::Context> context,
    std::shared_ptr<svulkan2::resource::SVResourceManager> manager)
    : mContext(context), mResourceManager(manager) {

  mCubeMesh = svulkan2::resource::SVMesh::CreateCube();
  mSphereMesh = svulkan2::resource::SVMesh::CreateUVSphere(32, 16);
  mPlaneMesh = svulkan2::resource::SVMesh::CreateYZPlane();
}

RenderServer::RenderServer(uint32_t maxNumMaterials, uint32_t maxNumTextures,
                           uint32_t defaultMipLevels, std::string const &device,
                           bool doNotLoadTexture) {
  mContext = svulkan2::core::Context::Create(false, maxNumMaterials, maxNumTextures,
                                             defaultMipLevels, doNotLoadTexture, device);
  mResourceManager = mContext->createResourceManager();
  spdlog::stderr_color_mt("RenderServer");
}

void RenderServer::start(std::string const &address) {
  mService = std::make_unique<RenderServiceImpl>(mContext, mResourceManager);
  grpc::ServerBuilder builder;
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());
  builder.RegisterService(mService.get());
  mServer = builder.BuildAndStart();
  log::info("Render server listening on {}", address);
}

void RenderServer::stop() {
  mServer->Shutdown();
  mServer->Wait();
}

bool RenderServer::waitAll(uint64_t timeout) {
  std::vector<vk::Semaphore> sems;
  std::vector<uint64_t> values;

  for (auto &kv : mService->mSceneMap.flat()) {
    for (auto &kv2 : kv.second->cameraMap) {
      sems.push_back(kv2.second->semaphore.get());
      values.push_back(kv2.second->frameCounter);
    }
  }
  auto result =
      mContext->getDevice().waitSemaphores(vk::SemaphoreWaitInfo({}, sems, values), timeout);
  if (result == vk::Result::eTimeout) {
    return false;
  }
  if (result == vk::Result::eSuccess) {
    return true;
  }
  throw std::runtime_error("failed to wait");
}

bool RenderServer::waitScenes(std::vector<int> const &list, uint64_t timeout) {
  std::vector<vk::Semaphore> sems;
  std::vector<uint64_t> values;
  for (int index : list) {
    for (auto cam : mService->mSceneList.at(index)->cameraList) {
      sems.push_back(cam->semaphore.get());
      values.push_back(cam->frameCounter);
    }
  }
  auto result =
      mContext->getDevice().waitSemaphores(vk::SemaphoreWaitInfo({}, sems, values), timeout);
  if (result == vk::Result::eTimeout) {
    return false;
  }
  if (result == vk::Result::eSuccess) {
    return true;
  }
  throw std::runtime_error("failed to wait");
}

VulkanCudaBuffer *RenderServer::allocateBuffer(std::string const &type,
                                               std::vector<int> const &shape) {
  mBuffers.push_back(std::make_unique<VulkanCudaBuffer>(
      mContext->getDevice(), mContext->getPhysicalDevice(), type, shape));
  return mBuffers.back().get();
}

std::vector<VulkanCudaBuffer *>
RenderServer::autoAllocateBuffers(std::vector<std::string> render_targets) {
  int maxSceneIndex = 0;

  int minCameraCount = INT32_MAX;
  int maxCameraCount = 0;
  int maxCameraWidth = 0;
  int maxCameraHeight = 0;
  int minCameraWidth = INT32_MAX;
  int minCameraHeight = INT32_MAX;

  // TODO: lock

  for (auto &kv : mService->mSceneMap.flat()) {
    maxSceneIndex = std::max(maxSceneIndex, static_cast<int>(kv.second->sceneIndex));
    maxCameraCount = std::max(maxCameraCount, static_cast<int>(kv.second->cameraMap.size()));
    minCameraCount = std::min(minCameraCount, static_cast<int>(kv.second->cameraMap.size()));
    for (auto &kv2 : kv.second->cameraMap) {
      int width = kv2.second->camera->getWidth();
      int height = kv2.second->camera->getHeight();

      maxCameraHeight = std::max(maxCameraHeight, height);
      maxCameraWidth = std::max(maxCameraWidth, width);
      minCameraHeight = std::min(minCameraHeight, height);
      minCameraWidth = std::min(minCameraWidth, width);
    }
  }

  if (maxSceneIndex >= 1024) {
    throw std::runtime_error("The largest scene index is " + std::to_string(maxSceneIndex) +
                             ". This is probably due to an error.");
  }

  if (maxCameraCount == 0) {
    throw std::runtime_error("No cameras are added.");
  }

  if (minCameraWidth == 0 || minCameraHeight == 0) {
    throw std::runtime_error("Some camera has size 0");
  }
  if (maxCameraWidth >= 16384 || maxCameraHeight >= 16384) {
    throw std::runtime_error("Some camera size is too large");
  }

  if (minCameraWidth != maxCameraWidth || minCameraHeight != maxCameraHeight) {
    log::warn("There are multiple camera sizes. This is not a good idea.");
  }

  if (maxCameraCount != minCameraCount) {
    log::warn("Different scenes have different number of cameras. This is not a good idea.");
  }

  int maxSceneCount = maxSceneIndex + 1;

  int channels, formatSize;

  std::vector<VulkanCudaBuffer *> buffers;
  for (std::string target : render_targets) {
    VulkanCudaBuffer *buffer;
    if (target == "color" || target == "Color") {
      target = "Color";
      channels = 4;
      formatSize = 4;
      buffer = allocateBuffer(
          "<f4", {maxSceneCount, maxCameraCount, maxCameraHeight, maxCameraWidth, channels});
    } else if (target == "position" || target == "Position") {
      target = "Position";
      channels = 4;
      formatSize = 4;
      buffer = allocateBuffer(
          "<f4", {maxSceneCount, maxCameraCount, maxCameraHeight, maxCameraWidth, channels});
    } else if (target == "segmentation" || target == "Segmentation") {
      target = "Segmentation";
      channels = 4;
      formatSize = 4;
      buffer = allocateBuffer(
          "<i4", {maxSceneCount, maxCameraCount, maxCameraHeight, maxCameraWidth, channels});
    } else {
      throw std::runtime_error("Target type " + target + " is not implemented");
    }
    buffers.push_back(buffer);

    size_t stride = maxCameraWidth * maxCameraHeight * channels * formatSize;

    for (auto &kv : mService->mSceneMap.flat()) {
      auto sceneIndex = kv.second->sceneIndex;
      for (auto &kv2 : kv.second->cameraMap) {
        auto cameraIndex = kv2.second->cameraIndex;
        size_t offset = (sceneIndex * maxCameraCount + cameraIndex) * stride;
        kv2.second->fillInfo.push_back({target, buffer->getBuffer(), offset});
      }
    }
  }

  return buffers;
}

std::string RenderServer::summary() const {
  int sceneSize, materialSize;
  {
    auto lock = mService->mSceneMap.lockRead();
    mService->mSceneMap.getMap().size();
  }
  {
    auto lock = mService->mMaterialMap.lockRead();
    mService->mMaterialMap.getMap().size();
  }

  std::stringstream ss;
  ss << "Scene     " << sceneSize << "\n";
  ss << "Materials " << materialSize << "\n";
  return ss.str();
}

VulkanCudaBuffer::VulkanCudaBuffer(vk::Device device, vk::PhysicalDevice physicalDevice,
                                   std::string const &type, std::vector<int> const &shape)
    : mDevice(device), mPhysicalDevice(physicalDevice), mType(type), mShape(shape) {
  if (type.length() < 3 || (type[0] != '<' && type[0] != '>')) {
    throw std::runtime_error("invalid type");
  }
  int typeSize = std::stoi(type.substr(2));
  if (typeSize <= 0) {
    throw std::runtime_error("invalid type");
  }

  mSize = typeSize;
  for (uint32_t i = 0; i < shape.size(); ++i) {
    mSize *= shape[i];
  }
  if (mSize <= 0) {
    throw std::runtime_error("empty buffer is not allowed");
  }

  vk::BufferCreateInfo bufferInfo(
      {}, mSize, vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
      vk::SharingMode::eExclusive);
  mBuffer = device.createBufferUnique(bufferInfo);

  auto memReqs = device.getBufferMemoryRequirements(mBuffer.get());
  vk::MemoryAllocateInfo memoryInfo(
      memReqs.size,
      findMemoryType(memReqs.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal));
  mMemory = device.allocateMemoryUnique(memoryInfo);
  device.bindBufferMemory(mBuffer.get(), mMemory.get(), 0);

  mCudaDeviceId = getCudaDeviceIdFromPhysicalDevice(physicalDevice);
  if (mCudaDeviceId < 0) {
    throw std::runtime_error(
        "Vulkan Device is not visible to CUDA. You probably need to unset the "
        "CUDA_VISIBLE_DEVICES variable. Or you can try other "
        "CUDA_VISIBLE_DEVICES until you find a working one.");
  }
  checkCudaErrors(cudaSetDevice(mCudaDeviceId));
  cudaExternalMemoryHandleDesc externalMemoryHandleDesc = {};
  externalMemoryHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;
  externalMemoryHandleDesc.size = memReqs.size;
  vk::MemoryGetFdInfoKHR vkMemoryGetFdInfoKHR;
  vkMemoryGetFdInfoKHR.setPNext(nullptr);
  vkMemoryGetFdInfoKHR.setMemory(mMemory.get());
  vkMemoryGetFdInfoKHR.setHandleType(vk::ExternalMemoryHandleTypeFlagBits::eOpaqueFd);
  auto cudaFd = device.getMemoryFdKHR(vkMemoryGetFdInfoKHR);
  externalMemoryHandleDesc.handle.fd = cudaFd;
  checkCudaErrors(cudaImportExternalMemory(&mCudaMem, &externalMemoryHandleDesc));

  cudaExternalMemoryBufferDesc externalMemBufferDesc = {};
  externalMemBufferDesc.offset = 0;
  externalMemBufferDesc.size = memReqs.size;
  externalMemBufferDesc.flags = 0;

  checkCudaErrors(cudaExternalMemoryGetMappedBuffer(&mCudaPtr, mCudaMem, &externalMemBufferDesc));
}

VulkanCudaBuffer::~VulkanCudaBuffer() {
  if (mCudaPtr) {
    checkCudaErrors(cudaDestroyExternalMemory(mCudaMem));
    checkCudaErrors(cudaFree(mCudaPtr));
  }
}

uint32_t VulkanCudaBuffer::findMemoryType(uint32_t typeFilter,
                                          vk::MemoryPropertyFlags properties) {
  auto memProps = mPhysicalDevice.getMemoryProperties();
  for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
    if ((typeFilter & (1 << i)) &&
        (memProps.memoryTypes[i].propertyFlags & properties) == properties) {
      return i;
    }
  }
  throw std::runtime_error("cannot find suitable memory to allocate buffer");
}

} // namespace server
} // namespace Renderer
} // namespace sapien
