//
// Created by jet on 7/18/21.
//

#include "kuafu_scene.hpp"
#include "kuafu_light.hpp"
#include "kuafu_renderer.hpp"
#include "kuafu_utils.hpp"
#include <core/context/global.hpp>
#include <core/geometry.hpp>
#include <spdlog/spdlog.h>

namespace sapien::Renderer {
//========= KuafuCamera =========//
KuafuCamera::KuafuCamera(int width, int height, float fovy, KuafuScene *scene, size_t idx) {
  mIdx = idx;
  pParentScene = scene;
  pKCamera = pParentScene->getKScene()->createCamera(width, height);
  auto widthF = static_cast<float>(width);
  auto heightF = static_cast<float>(height);
  float fy = heightF / 2.f / std::tan(fovy / 2.f);
  setPerspectiveCameraParameters(0.0, 0.0, fy, fy, widthF / 2.f, heightF / 2.f, 0.f);
}

void KuafuCamera::setPxPose(const physx::PxTransform &pose) {
  pKCamera->setPose(toGlmMat4(physx::PxMat44(pose)));
}

float KuafuCamera::getPrincipalPointX() const { return pKCamera->getPrincipalPointX(); };

float KuafuCamera::getPrincipalPointY() const { return pKCamera->getPrincipalPointY(); };
float KuafuCamera::getFocalX() const { return pKCamera->getFocalX(); };
float KuafuCamera::getFocalY() const { return pKCamera->getFocalY(); };
float KuafuCamera::getNear() const { return pKCamera->getNear(); };
float KuafuCamera::getFar() const { return pKCamera->getFar(); };
float KuafuCamera::getSkew() const { return pKCamera->getSkew(); };

void KuafuCamera::setPerspectiveCameraParameters(float near, float far, float fx, float fy,
                                                 float cx, float cy, float skew) {
  setFullPerspective(fx, fy, cx, cy, static_cast<float>(pKCamera->getWidth()),
                     static_cast<float>(pKCamera->getHeight()), skew);
}

void KuafuCamera::takePicture() {
  auto scene = pParentScene->getKScene();
  pParentScene->pKRenderer->setScene(scene);
  if (pParentScene->lastCameraIdx != mIdx) { // Camera changed!
    spdlog::get("SAPIEN")->info(
        "KF: Camera is different from last render! Reduced frame rate is expected.");
    if (pParentScene->pKRenderer->getConfig().getPresent()) {
      auto window = pParentScene->pKRenderer->getWindow();
      vk::Extent2D extent = window->getSize();
      auto w = pKCamera->getWidth();
      auto h = pKCamera->getHeight();
      if (static_cast<int>(extent.width) != w || static_cast<int>(extent.height) != h) {
        spdlog::get("SAPIEN")->info(
            "KF: Rendering on a different size of the viewer. Trying to resize the window!");
        window->resize(w, h);
      }
    }
    scene->setCamera(pKCamera);
    pParentScene->lastCameraIdx = mIdx;
  }
  pParentScene->pKRenderer->run();
};

std::vector<float> KuafuCamera::getFloatImage(std::string const &name) {
  if (name == "Color") {
    auto rgba = pParentScene->pKRenderer->downloadLatestFrame(pKCamera);
    size_t size = rgba.size();
    std::vector<float> ret(size);
    // Kuafu is BGRA
    for (size_t i = 0; i < size / 4; i++) {
      ret[4 * i + 0] = static_cast<float>(rgba[4 * i + 2]) / 255;
      ret[4 * i + 1] = static_cast<float>(rgba[4 * i + 1]) / 255;
      ret[4 * i + 2] = static_cast<float>(rgba[4 * i + 0]) / 255;
      ret[4 * i + 3] = static_cast<float>(rgba[4 * i + 3]) / 255;
    }
    return ret;
  }
  throw std::invalid_argument("Unrecognized image name " + name);
};

std::vector<uint32_t> KuafuCamera::getUintImage(std::string const &name) {
  throw std::invalid_argument("Integer image is not supported.");
};

physx::PxTransform KuafuCamera::getPose() const {
  spdlog::get("SAPIEN")->warn(
      "KF: Camera::getPose does not make sense, maybe you need mount.getPose?");
  return physx::PxTransform(toPxMat44(pKCamera->getPose()));
}

void KuafuCamera::setPose(physx::PxTransform const &pose) { setPxPose(pose); };

IPxrScene *KuafuCamera::getScene() { return pParentScene; }

//========= KuafuRenderShape =======//

std::shared_ptr<IPxrMaterial> KuafuRenderShape::getMaterial() const {
  auto matIndex = pKInstance->geometry->matIndex.front();
  auto kMat = kuafu::global::materials[matIndex];
  auto mat = std::make_shared<KuafuMaterial>(kMat);
  return mat;
}

inline void KuafuRenderShape::setMaterial(std::shared_ptr<IPxrMaterial> m) {
  // FIXME: relies on non-instancing
  auto mat = std::dynamic_pointer_cast<KuafuMaterial>(m)->getKMaterial();
  pKInstance->geometry->setMaterial(mat);
  pKInstance->geometry->initialized = false;
  pParentScene->getKScene()->markGeometriesChanged();
  pParentScene->getKScene()->markGeometryInstancesChanged();
}

//========= KuafuRigidBody =========//

KuafuRigidBody::KuafuRigidBody(KuafuScene *scene, KuafuGeometryInstances instances,
                               const physx::PxVec3 &scale)
    : pParentScene(scene), mScale(scale), pKGeometryInstances(std::move(instances)) {}

void KuafuRigidBody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void KuafuRigidBody::update(const physx::PxTransform &transform) {
  auto t = toGlmMat4(physx::PxMat44(transform * mInitialPose));
  t = glm::scale(t, {mScale.x, mScale.y, mScale.z});
  for (auto &ins : pKGeometryInstances)
    ins->setTransform(t);
}

void KuafuRigidBody::setVisibility(float visibility) {
  if (visibility == 0.0)
    setVisible(false);
  else if (visibility == 1.0)
    setVisible(true);
  else
    spdlog::get("SAPIEN")->error("KF: setVisibility with non-1/0 visibility is not supported yet");
}

void KuafuRigidBody::setVisible(bool visible) { // FIXME: The correctness of the function
  for (auto &ins : pKGeometryInstances) {       //   relies on the fact that sapien does not
    auto scene = pParentScene->getKScene();     //   use geometry instancing at all
    auto geometry = ins->geometry;
    if (visible && geometry->hideRender) {
      geometry->hideRender = false;
      scene->markGeometriesChanged();
      scene->markGeometryInstancesChanged();
      kuafu::global::frameCount = -1;
    }

    if (!visible && !geometry->hideRender) {
      geometry->hideRender = true;
      scene->markGeometriesChanged();
      scene->markGeometryInstancesChanged();
      kuafu::global::frameCount = -1;
    }
  }
}
void KuafuRigidBody::setRenderMode(uint32_t mode) {
  if (mode == 0) {
    setVisible(true);
    return;
  }
  if (mode == 1) {
    setVisible(false);
    return;
  }
  if (mode == 2) {
    spdlog::get("SAPIEN")->error("KF: setRenderMode(2) not supported yet");
  }
}

void KuafuRigidBody::setShadeFlat(bool shadeFlat) {
  spdlog::get("SAPIEN")->error("KF: shadeFlat not supported yet");
}
bool KuafuRigidBody::getShadeFlat() {
  spdlog::get("SAPIEN")->error("KF: shadeFlat not supported yet");
  return false;
}

void KuafuRigidBody::destroy() {
  pParentScene->getKScene()->removeGeometryInstances(pKGeometryInstances);
  pParentScene->removeRigidbody(this);
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {
  try {
    auto obj = kuafu::loadScene(meshFile, true);
    if (material)
      for (auto &o : obj)
        o->setMaterial(std::dynamic_pointer_cast<KuafuMaterial>(material)->getKMaterial());

    auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(scale.x, scale.y, scale.z));
    KuafuGeometryInstances rigidBodyInstances;

    for (auto &o : obj) {
      getKScene()->submitGeometry(o);
      auto ins = kuafu::instance(o, transform);
      getKScene()->submitGeometryInstance(ins);
      rigidBodyInstances.push_back(ins);
    }

    mBodies.push_back(std::make_unique<KuafuRigidBody>(this, rigidBodyInstances, scale));
    return mBodies.back().get();
  } catch (const std::exception &e) {
    spdlog::get("SAPIEN")->error("KF: Fail to load object: " + std::string(e.what()));
    return nullptr;
  }
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) {
  return addRigidbody(meshFile, scale, nullptr);
}

IPxrRigidbody *KuafuScene::addRigidbody(physx::PxGeometryType::Enum type,
                                        const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {
  std::shared_ptr<kuafu::Geometry> geometry;
  auto mat = std::dynamic_pointer_cast<KuafuMaterial>(material);
  if (!mat)
    mat = std::make_shared<KuafuMaterial>();
  auto kMat = mat->getKMaterial();
  auto new_scale = scale;

  switch (type) {
  case physx::PxGeometryType::eBOX:
    geometry = kuafu::createCube(true, kMat);
    break;
  case physx::PxGeometryType::eSPHERE:
    geometry = kuafu::createSphere(true, kMat);
    break;
  case physx::PxGeometryType::ePLANE:
    geometry = kuafu::createYZPlane(true, kMat);
    break;
  case physx::PxGeometryType::eCAPSULE:
    geometry = kuafu::createCapsule(scale.x, scale.y, true, kMat);
    new_scale = {1.F, 1.F, 1.F};
    //    geometry = kuafu::createCube(true, kMat);
    break;
  default:
    spdlog::get("SAPIEN")->error("KF: Failed to add Rigidbody: unimplemented shape");
    return nullptr;
  }

  auto transform = glm::scale(glm::mat4(1.0F), {new_scale.x, new_scale.y, new_scale.z});
  getKScene()->submitGeometry(geometry);
  auto ins = kuafu::instance(geometry, transform);
  getKScene()->submitGeometryInstance(ins);

  mBodies.push_back(
      std::make_unique<KuafuRigidBody>(this, KuafuGeometryInstances{ins}, new_scale));
  return mBodies.back().get();
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::vector<physx::PxVec3> &vertices,
                                        const std::vector<physx::PxVec3> &normals,
                                        const std::vector<uint32_t> &indices,
                                        const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {
  auto g = std::make_shared<kuafu::Geometry>();
  g->initialized = false;
  g->path = "";
  g->dynamic = true;

  if (indices.size() % 3 != 0)
    spdlog::get("SAPIEN")->error("KF: invalid geometry - indices");

  size_t totalAmountOfTriangles = indices.size() / 3;

  kuafu::global::materials.push_back(
      std::dynamic_pointer_cast<KuafuMaterial>(material)->getKMaterial());
  ++kuafu::global::materialIndex; // TODO: check existing dup mat

  g->isOpaque = (material->getBaseColor()[3] >= 1.0F);
  g->matIndex = std::vector<uint32_t>(totalAmountOfTriangles, kuafu::global::materialIndex - 1);

  g->indices = indices;

  // TODO: check duplicate v?
  if (vertices.size() != normals.size())
    spdlog::get("SAPIEN")->error("KF: invalid geometry - normals");

  g->vertices.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    kuafu::Vertex &vertex = g->vertices[i];
    vertex.pos = {vertices[i][0], vertices[i][1], vertices[i][2]};
    vertex.normal = {normals[i][0], normals[i][1], normals[i][2]};
  }

  auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(scale.x, scale.y, scale.z));
  getKScene()->submitGeometry(g);
  auto ins = kuafu::instance(g, transform);
  getKScene()->submitGeometryInstance(ins);

  mBodies.push_back(std::make_unique<KuafuRigidBody>(this, KuafuGeometryInstances{ins}, scale));
  return mBodies.back().get();
}

IPxrRigidbody *KuafuScene::addRigidbody(physx::PxGeometryType::Enum type,
                                        const physx::PxVec3 &scale, const physx::PxVec3 &color) {
  auto material = std::make_shared<KuafuMaterial>();
  material->setBaseColor({color.x, color.y, color.z, 1.f});
  return addRigidbody(type, scale, material);
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::vector<physx::PxVec3> &vertices,
                                        const std::vector<physx::PxVec3> &normals,
                                        const std::vector<uint32_t> &indices,
                                        const physx::PxVec3 &scale, const physx::PxVec3 &color) {
  auto material = std::make_shared<KuafuMaterial>();
  material->setBaseColor({color.x, color.y, color.z, 1.f});
  return addRigidbody(vertices, normals, indices, scale, material);
}

void KuafuScene::removeRigidbody(IPxrRigidbody *body) {
  for (auto it = mBodies.begin(); it != mBodies.end(); ++it)
    if (it->get() == body) {
      mBodies.erase(it);
      return;
    }
}

ICamera *KuafuScene::addCamera(uint32_t width, uint32_t height, float fovy, float near, float far,
                               const std::string &shaderDir) {
  if (!shaderDir.empty()) {
    spdlog::get("SAPIEN")->warn("KF: user-specified shader not supported");
  }
  auto cam = std::make_unique<KuafuCamera>(width, height, fovy, this, nextCameraIdx++);
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void KuafuScene::removeCamera(ICamera *camera) {
  //  KF_WARN("KF: removeCamera called on {}, idx={}", long(camera),
  //  dynamic_cast<KuafuCamera*>(camera)->mIdx);
  getKScene()->removeCamera(dynamic_cast<KuafuCamera *>(camera)->pKCamera);
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [camera](auto &c) { return camera == c.get(); }),
                 mCameras.end());
}

std::vector<ICamera *> KuafuScene::getCameras() {
  std::vector<ICamera *> cams;
  for (auto &cam : mCameras) {
    cams.push_back(cam.get());
  }
  return cams;
}

void KuafuScene::setAmbientLight(const std::array<float, 3> &color) {
  getKScene()->setClearColor({color[0], color[1], color[2], 1.0f});
}

std::array<float, 3> KuafuScene::getAmbientLight() const {
  auto clearColor = getKScene()->getClearColor();
  return {clearColor[0], clearColor[1], clearColor[2]};
}

IPointLight *KuafuScene::addPointLight(std::array<float, 3> const &position,
                                       std::array<float, 3> const &color, bool enableShadow,
                                       float shadowNear, float shadowFar, uint32_t shadowMapSize) {
  auto light = std::make_shared<kuafu::PointLight>();
  light->position = {position[0], position[1], position[2]};
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->radius = 0.1; // TODO: expose this

  getKScene()->addPointLight(light); // TODO: check if success

  mLights.push_back(std::make_unique<KuafuPointLight>(light));
  return dynamic_cast<IPointLight *>(mLights.back().get());
}

IDirectionalLight *KuafuScene::addDirectionalLight(std::array<float, 3> const &direction,
                                                   std::array<float, 3> const &color,
                                                   bool enableShadow,
                                                   std::array<float, 3> const &position,
                                                   float shadowScale, float shadowNear,
                                                   float shadowFar, uint32_t shadowMapSize) {
  auto light = std::make_shared<kuafu::DirectionalLight>();
  light->direction = {direction[0], direction[1], direction[2]};
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->softness = 0.1; // TODO: expose this

  getKScene()->setDirectionalLight(light); // TODO: check if success

  mLights.push_back(std::make_unique<KuafuDirectionalLight>(light));
  return dynamic_cast<IDirectionalLight *>(mLights.back().get());
}

ISpotLight *KuafuScene::addSpotLight(std::array<float, 3> const &position,
                                     std::array<float, 3> const &direction, float fovInner,
                                     float fovOuter, std::array<float, 3> const &color,
                                     bool enableShadow, float shadowNear, float shadowFar,
                                     uint32_t shadowMapSize) {
  auto light = std::make_shared<kuafu::ActiveLight>();
  glm::vec3 p = {position[0], position[1], position[2]};
  glm::vec3 dir = {direction[0], direction[1], direction[2]};
  light->viewMat = glm::lookAt(p, p + dir, kuafu::utils::getPerpendicular(dir));
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->softness = 0.0;
  light->texPath = "";
  light->fov = fovInner;
  if (fovInner != fovOuter)
    spdlog::get("SAPIEN")->warn("KF: fovInner != fovOuter does not take effect");

  getKScene()->addActiveLight(light); // TODO: check if success

  mLights.push_back(std::make_unique<KuafuSpotLight>(light));
  return dynamic_cast<ISpotLight *>(mLights.back().get());
}

IActiveLight *KuafuScene::addActiveLight(physx::PxTransform const &pose,
                                         std::array<float, 3> const &color, float fov,
                                         std::string_view texPath, float shadowNear,
                                         float shadowFar, uint32_t shadowMapSize) {
  auto light = std::make_shared<kuafu::ActiveLight>();
  light->viewMat = glm::inverse(toGlmMat4(pose));
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->softness = 0.0;
  light->texPath = texPath;
  light->fov = fov;

  getKScene()->addActiveLight(light); // TODO: check if success

  mLights.push_back(std::make_unique<KuafuActiveLight>(light));
  return dynamic_cast<IActiveLight *>(mLights.back().get());
};

void KuafuScene::removeLight(ILight *light) {
  dynamic_cast<IKuafuLight *>(light)->_kfRemoveFromScene(pKScene);
  mLights.erase(std::remove_if(mLights.begin(), mLights.end(),
                               [light](auto &l) { return light == l.get(); }),
                mLights.end());
}

void KuafuScene::destroy() {
  //  spdlog::get("SAPIEN")->warn("KF: KuafuScene::destroy called!");
  while (!mBodies.empty()) { // Warning: b->destroy() will change the container!
    auto &b = mBodies.back();
    b->destroy();
  }

  while (!mCameras.empty()) {
    auto &c = mCameras.back();
    removeCamera(c.get());
  }

  while (!mLights.empty()) {
    auto &l = mLights.back();
    removeLight(l.get());
  }

  pKRenderer->removeScene(pKScene);
}
} // namespace sapien::Renderer
