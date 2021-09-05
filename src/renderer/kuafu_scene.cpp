//
// Created by jet on 7/18/21.
//

#include "kuafu_utils.hpp"
#include "kuafu_scene.hpp"
#include "kuafu_light.hpp"
#include "kuafu_renderer.hpp"
#include <core/context/global.hpp>
#include <core/geometry.hpp>
#include <spdlog/spdlog.h>

namespace sapien::Renderer {

//========= KCamera =========//
void KCamera::processKeyboard() {
  const float defaultSpeed = 2.5F;
  static float currentSpeed = defaultSpeed;
  float finalSpeed = currentSpeed * kuafu::Time::getDeltaTime();

  if (kuafu::global::keys::eW) {
    mPosition += mDirFront * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eS) {
    mPosition -= mDirFront * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eA) {
    mPosition += mDirRight * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eD) {
    mPosition -= mDirRight * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eQ) {
    mPosition -= mDirUp * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eE) {
    mPosition += mDirUp * finalSpeed;
    mViewNeedsUpdate = true;
  }
}

//========= KuafuCamera =========//

void KuafuCamera::setPxPose(const physx::PxTransform &pose) {
  mKCamera->setPose(toGlmMat4(physx::PxMat44(pose)));
}

void KuafuCamera::takePicture() {
  if (!pParentScene->mUseViewer)
    pParentScene->getKScene().setCamera(mKCamera);
  pParentScene->pKRenderer->run();
};

std::vector<float> KuafuCamera::getColorRGBA() {
  const auto& rgba = pParentScene->pKRenderer->downloadLatestFrame();
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
};

void KuafuCamera::setInitialPose(const physx::PxTransform &pose) {
  mInitialPose = pose;
  setPxPose(pose);
}

physx::PxTransform KuafuCamera::getPose() const {
  spdlog::get("SAPIEN")->warn(
      "KF: Camera::getPose does not make sense, maybe you need mount.getPose?");
  return physx::PxTransform(toPxMat44(mKCamera->getPose()));
}

void KuafuCamera::setPose(physx::PxTransform const &pose) {
  auto p = pose * mInitialPose;
  setPxPose(p);
};

IPxrScene *KuafuCamera::getScene() { return pParentScene; }


//========= KuafuRigidBody =========//

KuafuRigidBody::KuafuRigidBody(
    KuafuScene *scene,  std::vector<size_t> indices, const physx::PxVec3& scale)
    : mParentScene(scene), mScale(scale), mKGeometryInstanceIndices(std::move(indices)) {}

void KuafuRigidBody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void KuafuRigidBody::update(const physx::PxTransform &transform) {
  auto t = toGlmMat4(physx::PxMat44(transform * mInitialPose));
  t = glm::scale(t, {mScale.x, mScale.y, mScale.z});
  for (auto idx: mKGeometryInstanceIndices) {
    auto obj = mParentScene->getKScene().getGeometryInstance(idx);
    obj->setTransform(t);
  }
}

void KuafuRigidBody::setVisibility(float visibility) {
  spdlog::get("SAPIEN")->error("KF: setVisibility not supported yet");
}
void KuafuRigidBody::setVisible(bool visible) {       // The correctness of the function
  for (auto idx: mKGeometryInstanceIndices) {         // relies on the fact that sapien does not
    auto& scene = mParentScene->getKScene();          // use geometry instancing at all
    auto instance = scene.getGeometryInstance(idx);
    auto geometry = scene.getGeometryByGlobalIndex(instance->geometryIndex);
    if (visible && geometry->hideRender) {
      geometry->hideRender = false;
      scene.markGeometriesChanged();
      scene.markGeometryInstancesChanged();
      kuafu::global::frameCount = -1;
    }

    if (!visible && !geometry->hideRender) {
      geometry->hideRender = true;
      scene.markGeometriesChanged();
      scene.markGeometryInstancesChanged();
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

void KuafuRigidBody::destroy() {
  // TODO: kuafu_urgent
  spdlog::get("SAPIEN")->warn("KF: FIXME: KuafuRigidBody::destroy");
  for (auto idx: mKGeometryInstanceIndices) {
    auto obj = mParentScene->getKScene().getGeometryInstance(idx);

    // An out-of-sight t
    glm::mat4 t;
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++)
        t[i][j] = 0;

    t[0][0] = t[1][1] = t[2][2] = 0.0001f;
    t[3][0] = t[3][1] = t[3][2] = 10000.f;
    t[3][3] = 1.f;

    obj->setTransform(t);
  }
//  pParentScene->removeRigidbody(this);
}

IPxrRigidbody *KuafuScene::addRigidbodyWithNewMaterial(
    const std::string &meshFile, const physx::PxVec3 &scale,
    const std::shared_ptr<IPxrMaterial>& material) {
  try {
    auto obj = kuafu::loadScene(meshFile, true);
    if (material)
      for (auto &o: obj)
        o->setMaterial(std::dynamic_pointer_cast<KuafuMaterial>(material)->getKMaterial());

    auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(scale.x, scale.y, scale.z));
    std::vector<size_t> rigidBodyIndices;

    auto orgGeoCnt = getKScene().getGeometryInstanceCount();

    for (size_t i = 0; i < obj.size(); i++) {
      getKScene().submitGeometry(obj[i]);
      getKScene().submitGeometryInstance(
          kuafu::instance(obj[i], transform));
      rigidBodyIndices.push_back(orgGeoCnt + i);
    }

    mBodies.push_back(std::make_unique<KuafuRigidBody>(this, rigidBodyIndices, scale));
    return mBodies.back().get();
  } catch (const std::exception &e) {
    spdlog::get("SAPIEN")->error("KF: Fail to load object: " + std::string(e.what()));
    return nullptr;
  }
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) {
  return addRigidbodyWithNewMaterial(meshFile, scale, nullptr);
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
  getKScene().submitGeometry(geometry);
  getKScene().submitGeometryInstance(
      kuafu::instance(geometry, transform));

  size_t rigidBodyIdx = getKScene().getGeometryInstanceCount() - 1;
  mBodies.push_back(std::make_unique<KuafuRigidBody>(
      this, std::vector<size_t>{rigidBodyIdx}, new_scale));
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
  g->geometryIndex = kuafu::global::geometryIndex++;
  g->dynamic = true;

  if (indices.size() % 3 != 0)
    spdlog::get("SAPIEN")->error("KF: invalid geometry - indices");

  size_t totalAmountOfTriangles = indices.size() / 3;

  kuafu::global::materials.push_back(
      std::dynamic_pointer_cast<KuafuMaterial>(material)->getKMaterial());
  ++kuafu::global::materialIndex;             // TODO: check existing dup mat

  g->isOpaque = (material->getBaseColor()[3] >= 1.0F);
  g->matIndex = std::vector<uint32_t>(totalAmountOfTriangles, kuafu::global::materialIndex - 1);

  g->indices = indices;

  // TODO: check duplicate v?
  if (vertices.size() != normals.size())
    spdlog::get("SAPIEN")->error("KF: invalid geometry - normals");

  g->vertices.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    kuafu::Vertex& vertex = g->vertices[i];
    vertex.pos = {vertices[i][0], vertices[i][1], vertices[i][2]};
    vertex.normal = {normals[i][0], normals[i][1], normals[i][2]};
  }

  auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(scale.x, scale.y, scale.z));
  getKScene().submitGeometry(g);
  getKScene().submitGeometryInstance(
      kuafu::instance(g, transform));

  size_t rigidBodyIdx = getKScene().getGeometryInstanceCount() - 1;
  mBodies.push_back(std::make_unique<KuafuRigidBody>(
      this, std::vector<size_t>{rigidBodyIdx}, scale));
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
  /*TODO:kuafu_urgent check this impl*/
}

ICamera *KuafuScene::addCamera(const std::string &name, uint32_t width, uint32_t height,
                               float fovx, float fovy, float near, float far,
                               const std::string &shaderDir) {
  if (fovx != 0) {
    spdlog::get("SAPIEN")->warn(
        "KF: Current camera implementation does not support non-square"
        "pixels, and fovy will be used. Set fovx to 0 to suppress this warning");
  }
  if (!shaderDir.empty()) {
    spdlog::get("SAPIEN")->warn(
        "KF: user-specified shader not supported");
  }
  auto cam = std::make_unique<KuafuCamera>(name, width, height, fovy, this);
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void KuafuScene::removeCamera(ICamera *camera) {
  spdlog::get("SAPIEN")->error("KF: removeCamera not implemented yet");
}

std::vector<ICamera *> KuafuScene::getCameras() {
  std::vector<ICamera *> cams;
  for (auto &cam : mCameras) {
    cams.push_back(cam.get());
  }
  return cams;
}

void KuafuScene::setAmbientLight(const std::array<float, 3> &color) {
  pKRenderer->getConfig().setClearColor({color[0], color[1], color[2], 1.0f});
}

std::array<float, 3> KuafuScene::getAmbientLight() const {
  auto clearColor = pKRenderer->getConfig().getClearColor();
  return {clearColor[0], clearColor[1], clearColor[2]};
}

IPointLight *KuafuScene::addPointLight(const std::array<float, 3> &position,
                                       const std::array<float, 3> &color, bool enableShadow,
                                       float shadowNear, float shadowFar) {
  auto light = std::make_shared<kuafu::PointLight>();
  light->position = {position[0], position[1], position[2]};
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->radius = 0.1;    // TODO: expose this

  getKScene().addPointLight(light);          // TODO: check if success
  mPointLights.push_back(std::make_shared<KuafuPointLight>(light));
  return mPointLights.back().get();
}

IDirectionalLight *KuafuScene::addDirectionalLight(
    const std::array<float, 3> &direction, const std::array<float, 3> &color, bool enableShadow,
    const std::array<float, 3> &position, float shadowScale, float shadowNear, float shadowFar) {
  auto light = std::make_shared<kuafu::DirectionalLight>();
  light->direction = {direction[0], direction[1], direction[2]};
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->softness = 0.1;    // TODO: expose this

  getKScene().setDirectionalLight(light);       // TODO: check if success
  mDirectionalLights.push_back(std::make_shared<KuafuDirectionalLight>(light));
  return mDirectionalLights.back().get();
}

ISpotLight *KuafuScene::addSpotLight(const std::array<float, 3> &position,
                                     const std::array<float, 3> &direction, float fovInner,
                                     float fovOuter, const std::array<float, 3> &color,
                                     bool enableShadow, float shadowNear, float shadowFar) {
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

  getKScene().addActiveLight(light);       // TODO: check if success
  mSpotLights.push_back(std::make_shared<KuafuSpotLight>(light));
  return mSpotLights.back().get();
}

IActiveLight *KuafuScene::addActiveLight(physx::PxTransform const &pose,
                                         std::array<float, 3> const &color,
                                         float fov, std::string_view texPath) {
  auto light = std::make_shared<kuafu::ActiveLight>();
  light->viewMat = glm::inverse(toGlmMat4(pose));
  light->color = {color[0], color[1], color[2]};
  light->strength = 1.0;
  light->softness = 0.0;
  light->texPath = texPath;
  light->fov = fov;

  getKScene().addActiveLight(light);        // TODO: check if success
  mActiveLights.push_back(std::make_shared<KuafuActiveLight>(light));
  return mActiveLights.back().get();
};

void KuafuScene::removeLight(ILight *light) {
  spdlog::get("SAPIEN")->error("KF: removeLight not implemented yet");
}

void KuafuScene::destroy() {
  spdlog::get("SAPIEN")->error("KF: destroy not implemented yet");
}
}