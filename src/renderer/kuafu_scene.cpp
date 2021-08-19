//
// Created by jet on 7/18/21.
//

#include "kuafu_scene.hpp"
#include "core/geometry.hpp"
#include <spdlog/spdlog.h>

namespace sapien::Renderer {

glm::mat4 fromPxMat44(physx::PxMat44 mat) {
  glm::mat4 ret;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      ret[i][j] = mat[i][j];
  return ret;
}

//========= KuafuRigidBody =========//

KuafuRigidBody::KuafuRigidBody(
    KuafuScene *scene,  size_t idx)
    : mParentScene(scene), mKGeometryInstanceIdx(idx) {}

void KuafuRigidBody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void KuafuRigidBody::update(const physx::PxTransform &transform) {
  auto t = fromPxMat44(physx::PxMat44(transform * mInitialPose));
  auto obj = mParentScene->mKRenderer->getScene().getGeometryInstance(mKGeometryInstanceIdx);
  obj->setTransform(t);
}

void KuafuRigidBody::setVisibility(float visibility) {
  assert(false); /*not supported by rt pipeline*/
}
void KuafuRigidBody::setVisible(bool visible) {
  assert(false); /*not supported by rt pipeline*/
}
void KuafuRigidBody::setRenderMode(uint32_t mode) {
  assert(false); /*not supported by rt pipeline*/
}
void KuafuRigidBody::destroy() {
  /*TODO:kuafu_urgent check this impl*/
  auto obj = mParentScene->mKRenderer->getScene().getGeometryInstance(mKGeometryInstanceIdx);

  // An out-of-sight t
  glm::mat4 t;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      t[i][j] = 0;

  t[0][0] = t[1][1] = t[2][2] = 0.001f;
  t[0][3] = t[1][3] = t[2][3] = 1000.f;
  t[3][3] = 1.f;

  obj->setTransform(t);
//  mParentScene->removeRigidbody(this);
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) {
  try {
    auto obj = kuafu::loadObj(meshFile, true);
    auto transform = glm::scale(glm::mat4(1.0F), glm::vec3(scale.x, scale.y, scale.z));
    mKRenderer->getScene().submitGeometry(obj);
    mKRenderer->getScene().submitGeometryInstance(
        kuafu::instance(obj, transform));

    size_t rigidBodyIdx = mKRenderer->getScene().getGeometryInstanceCount() - 1;
    mBodies.push_back(std::make_unique<KuafuRigidBody>(this, rigidBodyIdx));
    return mBodies.back().get();

  } catch (const std::exception &) {
    spdlog::get("SAPIEN")->error("fail to load object");
    return nullptr;
  }
}

IPxrRigidbody *KuafuScene::addRigidbody(physx::PxGeometryType::Enum type,
                                        const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {

  assert(false); /* not implemented yet */
  return nullptr;
}

IPxrRigidbody *KuafuScene::addRigidbody(const std::vector<physx::PxVec3> &vertices,
                                        const std::vector<physx::PxVec3> &normals,
                                        const std::vector<uint32_t> &indices,
                                        const physx::PxVec3 &scale,
                                        std::shared_ptr<IPxrMaterial> material) {
  assert(false); /* not implemented yet */
  return nullptr;
}

void KuafuScene::removeRigidbody(IPxrRigidbody *body) {
  /*TODO:kuafu_urgent check this impl*/
}

ICamera *KuafuScene::addCamera(const std::string &name, uint32_t width, uint32_t height,
                               float fovx, float fovy, float near, float far,
                               const std::string &shaderDir) {
  assert(false); /* not implemented yet */
  return nullptr;
}

void KuafuScene::removeCamera(ICamera *camera) { assert(false); /* not implemented yet */ }

std::vector<ICamera *> KuafuScene::getCameras() {
  assert(false); /* not implemented yet */
  return std::vector<ICamera *>();
}

void KuafuScene::setAmbientLight(const std::array<float, 3> &color) {
  assert(false); /* not implemented yet */
}

std::array<float, 3> KuafuScene::getAmbientLight() const {
  assert(false); /* not implemented yet */
  return std::array<float, 3>();
}

IPointLight *KuafuScene::addPointLight(const std::array<float, 3> &position,
                                       const std::array<float, 3> &color, bool enableShadow,
                                       float shadowNear, float shadowFar) {
  assert(false); /* not implemented yet */
  return nullptr;
}

IDirectionalLight *KuafuScene::addDirectionalLight(
    const std::array<float, 3> &direction, const std::array<float, 3> &color, bool enableShadow,
    const std::array<float, 3> &position, float shadowScale, float shadowNear, float shadowFar) {
  assert(false); /* not implemented yet */
  return nullptr;
}

ISpotLight *KuafuScene::addSpotLight(const std::array<float, 3> &position,
                                     const std::array<float, 3> &direction, float fovInner,
                                     float fovOuter, const std::array<float, 3> &color,
                                     bool enableShadow, float shadowNear, float shadowFar) {
  assert(false); /* not implemented yet */
  return nullptr;
}

void KuafuScene::removeLight(ILight *light) {
  assert(false); /* not implemented yet */
}

void KuafuScene::destroy() {
  assert(false); /* not implemented yet */
}
}