//
// Created by jet on 7/18/21.
//

#include "kuafu_scene.hpp"
#include "kuafu_renderer.hpp"
#include <core/geometry.hpp>
#include <core/context/global.hpp>
#include <spdlog/spdlog.h>

namespace sapien::Renderer {

glm::mat4 toGlmMat4(physx::PxMat44 mat) {
  glm::mat4 ret;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      ret[i][j] = mat[i][j];
  return ret;
}

physx::PxMat44 toPxMat44(glm::mat4 mat) {
  physx::PxMat44 ret;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      ret[i][j] = mat[i][j];
  return ret;
}

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
    mPosition -= mDirRight * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eD) {
    mPosition += mDirRight * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eQ) {
    mPosition -= _up * finalSpeed;
    mViewNeedsUpdate = true;
  }

  if (kuafu::global::keys::eE) {
    mPosition += _up * finalSpeed;
    mViewNeedsUpdate = true;
  }


}

//========= KuafuCamera =========//

void KuafuCamera::setPxPose(const physx::PxTransform &pose) {
  mKCamera->setPosition({pose.p.x, pose.p.y, pose.p.z});
  mKCamera->setFront({-pose.p.x, -pose.p.y, -pose.p.z});
//  mKCamera->setFront({1, 0, 0});
//  auto t = physx::PxMat44(pose);
//  mKCamera->setFront({t[0][1], t[1][1], t[2][1]});

  // TODO: kuafu_urgent fix this func
//  spdlog::get("SAPIEN")->warn("check setPxPose");
}

void KuafuCamera::takePicture() {
  if (!pParentScene->mUseViewer)
    pParentScene->getKScene().setCamera(mKCamera);
  pParentScene->pKRenderer->run();
};

void KuafuCamera::setInitialPose(const physx::PxTransform &pose) {
//  auto cam = mParentscene->getKScene().getCamera();
//  cam->setPosition({-12.6F, 1.1F, 19.4F});
//  cam->setFront({0.67F, 0.0F, -0.8F});

  mInitialPose = pose;
  setPxPose(pose);
}

physx::PxTransform KuafuCamera::getPose() const {
  auto view = mKCamera->getViewInverseMatrix();
  // TODO: kuafu_urgent inverse?
  spdlog::get("SAPIEN")->warn("check getPose");

  return physx::PxTransform(toPxMat44(view));
}

void KuafuCamera::setPose(physx::PxTransform const &pose) {
  auto p = pose * mInitialPose;
  setPxPose(p);
};

IPxrScene *KuafuCamera::getScene() { return pParentScene; }


//========= KuafuRigidBody =========//

KuafuRigidBody::KuafuRigidBody(
    KuafuScene *scene,  size_t idx, physx::PxVec3 scale)
    : mParentScene(scene), mScale(scale), mKGeometryInstanceIdx(idx) {}

void KuafuRigidBody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void KuafuRigidBody::update(const physx::PxTransform &transform) {
  auto t = toGlmMat4(physx::PxMat44(transform * mInitialPose));
  t = glm::scale(t, {mScale.x, mScale.y, mScale.z});
  auto obj = mParentScene->getKScene().getGeometryInstance(mKGeometryInstanceIdx);
  obj->setTransform(t);
}

void KuafuRigidBody::setVisibility(float visibility) {
  spdlog::get("SAPIEN")->error("setVisibility not supported by rt pipeline");
}
void KuafuRigidBody::setVisible(bool visible) {
  auto instance = mParentScene->getKScene().getGeometryInstance(mKGeometryInstanceIdx);
  auto geometry = mParentScene->getKScene().getGeometryByGlobalIndex(instance->geometryIndex);
  auto matIdx = geometry->matIndex.front();  // TODO: kuafu_urgent
  if (visible && mHaveSetInvisible) {
    spdlog::get("SAPIEN")->warn("The object may have lost original material");
    auto mat = kuafu::global::materials[matIdx];
    mat.d = 1.0f;
    geometry->setMaterial(mat);
    geometry->initialized = false;
    mParentScene->getKScene().markGeometriesChanged();
    mParentScene->getKScene().markGeometryInstancesChanged();
    mHaveSetInvisible = false;
  }

  if (!visible && ! mHaveSetInvisible) {
    spdlog::get("SAPIEN")->warn("The object may lost material if set invisible");
    auto mat = kuafu::global::materials[matIdx];
    mat.d = 0.0f;
    geometry->setMaterial(mat);
    geometry->initialized = false;
    mParentScene->getKScene().markGeometriesChanged();
    mParentScene->getKScene().markGeometryInstancesChanged();
    mHaveSetInvisible = true;
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
    spdlog::get("SAPIEN")->error("setRenderMode(2) not supported by rt pipeline");
  }
}

void KuafuRigidBody::destroy() {
  /* TODO:kuafu_urgent rewrite this */
  auto obj = mParentScene->getKScene().getGeometryInstance(mKGeometryInstanceIdx);

  // An out-of-sight t
  glm::mat4 t;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      t[i][j] = 0;

  t[0][0] = t[1][1] = t[2][2] = 0.001f;
  t[3][0] = t[3][1] = t[3][2] = 1000.f;
  t[3][3] = 1.f;

  obj->setTransform(t);
//  pParentScene->removeRigidbody(this);
}

IPxrRigidbody *KuafuScene::addRigidbodyWithNewMaterial(
    const std::string &meshFile, const physx::PxVec3 &scale, std::shared_ptr<IPxrMaterial> material) {
  try {
    auto obj = kuafu::loadObj(meshFile, true);
    if (material)
      obj->setMaterial(*dynamic_cast<KuafuMaterial*>(material.get())->getKMaterial());  // TODO: kuafu_urgent: bind this material
    auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(scale.x, scale.y, scale.z));
    getKScene().submitGeometry(obj);
    getKScene().submitGeometryInstance(
        kuafu::instance(obj, transform));

    size_t rigidBodyIdx = getKScene().getGeometryInstanceCount() - 1;
    mBodies.push_back(std::make_unique<KuafuRigidBody>(this, rigidBodyIdx, scale));
    return mBodies.back().get();

  } catch (const std::exception &) {
    spdlog::get("SAPIEN")->error("fail to load object");
    return nullptr;
  }
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) {
  return addRigidbodyWithNewMaterial(meshFile, scale, nullptr);
}

IPxrRigidbody *KuafuScene::addRigidbody(physx::PxGeometryType::Enum type,
                                        const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {
//  kuafu::Geometry geometry;
  switch (type) {
  case physx::PxGeometryType::eBOX: {
    auto ret = addRigidbodyWithNewMaterial(
        "/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets/models/cube.obj", scale, material);
    spdlog::get("SAPIEN")->warn("eBOX: tmp impl");
    return ret;
    break;
  }
  case physx::PxGeometryType::eSPHERE: {
    auto ret = addRigidbodyWithNewMaterial(
        "/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets/models/sphere.obj", scale, material);
    spdlog::get("SAPIEN")->warn("eSPHERE: tmp impl");
    return ret;
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    auto ret = addRigidbodyWithNewMaterial(
        "/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets/models/plane.obj", scale, material);
    spdlog::get("SAPIEN")->warn("ePLANE: tmp impl");
    return ret;
    break;
  }
  case physx::PxGeometryType::eCAPSULE: {
    auto ret = addRigidbodyWithNewMaterial(
        "/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets/models/cube.obj",  scale, material);
    spdlog::get("SAPIEN")->warn("eCAPSULE: tmp impl, incorrect");
    return ret;
    break;
  }
  default:
    spdlog::get("SAPIEN")->error("Failed to add Rididbody: unimplemented shape");
    return nullptr;
  }
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::vector<physx::PxVec3> &vertices,
                                        const std::vector<physx::PxVec3> &normals,
                                        const std::vector<uint32_t> &indices,
                                        const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {
  std::shared_ptr<kuafu::Geometry> g;
  g->path = "";
  g->geometryIndex = kuafu::global::geometryIndex++;
  g->dynamic = true;
  g->subMeshCount = 1;

  if (indices.size() % 3 != 0)
    spdlog::get("SAPIEN")->error("invalid geometry");

  size_t totalAmountOfTriangles = indices.size() / 3;



  spdlog::get("SAPIEN")->error("addRigidbody3 not implemented yet");
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
        "Current camera implementation does not support non-square"
        "pixels, and fovy will be used. Set fovx to 0 to suppress this warning");
  }
  auto cam = std::make_unique<KuafuCamera>(name, width, height, fovy, this);
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void KuafuScene::removeCamera(ICamera *camera) {
  spdlog::get("SAPIEN")->error("removeCamera not implemented yet");
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
  auto lightSphere = kuafu::loadObj("/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets/models/sphere.obj");
  kuafu::Material lightMaterial;
  lightMaterial.emission = glm::vec3({color[0], color[1], color[2]});
  lightSphere->setMaterial(lightMaterial);

  auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(0.05F));
  transform = glm::translate(transform, {position[0], position[1], position[2]});

  getKScene().submitGeometry(lightSphere);
  getKScene().submitGeometryInstance(
      kuafu::instance(lightSphere, transform));

  spdlog::get("SAPIEN")->warn("addPointLight: incorrect tmp impl");
}

IDirectionalLight *KuafuScene::addDirectionalLight(
    const std::array<float, 3> &direction, const std::array<float, 3> &color, bool enableShadow,
    const std::array<float, 3> &position, float shadowScale, float shadowNear, float shadowFar) {
  auto lightPlane = kuafu::loadObj("/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets/models/plane.obj");
  kuafu::Material lightMaterial;
  lightMaterial.emission = glm::vec3({color[0], color[1], color[2]});
  lightPlane->setMaterial(lightMaterial);

  auto transform = glm::rotate(glm::mat4(1.0F), glm::radians(180.F), {0, 1, 0});
  transform = glm::translate(transform, {position[0], position[1], position[2]});

  getKScene().submitGeometry(lightPlane);
  getKScene().submitGeometryInstance(
      kuafu::instance(lightPlane, transform));

  spdlog::get("SAPIEN")->warn("addDirectionalLight: incorrect tmp impl");
}

ISpotLight *KuafuScene::addSpotLight(const std::array<float, 3> &position,
                                     const std::array<float, 3> &direction, float fovInner,
                                     float fovOuter, const std::array<float, 3> &color,
                                     bool enableShadow, float shadowNear, float shadowFar) {
  spdlog::get("SAPIEN")->error("addSpotLight not implemented yet");
}

void KuafuScene::removeLight(ILight *light) {
  spdlog::get("SAPIEN")->error("removeLight not implemented yet");
}

void KuafuScene::destroy() {
  spdlog::get("SAPIEN")->error("destroy not implemented yet");
}
}