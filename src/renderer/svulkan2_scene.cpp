#include "svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

SVulkan2Scene::SVulkan2Scene(SVulkan2Renderer *renderer, std::string const &name)
    : mParentRenderer(renderer), mName(name) {
  mScene = std::make_unique<svulkan2::scene::Scene>();
}

void SVulkan2Scene::setAmbientLight(std::array<float, 3> const &color) {
  mScene->setAmbientLight({color[0], color[1], color[2], 1.f});
}

std::array<float, 3> SVulkan2Scene::getAmbientLight() const {
  auto light = mScene->getAmbientLight();
  return {light.x, light.y, light.z};
}

SVulkan2PointLight *SVulkan2Scene::addPointLight(std::array<float, 3> const &position,
                                                 std::array<float, 3> const &color,
                                                 bool enableShadow, float shadowNear,
                                                 float shadowFar) {
  auto &light = mScene->addPointLight();
  light.setColor({color[0], color[1], color[2]});
  light.setTransform({.position = glm::vec4(position[0], position[1], position[2], 1.f)});
  if (enableShadow) {
    light.enableShadow(true);
    light.setShadowParameters(shadowNear, shadowFar);
  }
  auto l = std::make_unique<SVulkan2PointLight>(light);
  auto result = l.get();
  mLights.push_back(std::move(l));
  return result;
}

SVulkan2DirectionalLight *SVulkan2Scene::addDirectionalLight(
    std::array<float, 3> const &direction, std::array<float, 3> const &color, bool enableShadow,
    std::array<float, 3> const &position, float shadowScale, float shadowNear, float shadowFar) {
  auto &light = mScene->addDirectionalLight();
  light.setDirection({direction[0], direction[1], direction[2]});
  light.setColor({color[0], color[1], color[2]});
  if (enableShadow) {
    light.enableShadow(true);
    light.setPosition({position[0], position[1], position[2]});
    light.setShadowParameters(shadowNear, shadowFar, shadowScale);
  }
  auto l = std::make_unique<SVulkan2DirectionalLight>(light);
  auto result = l.get();
  mLights.push_back(std::move(l));
  return result;
}

SVulkan2SpotLight *SVulkan2Scene::addSpotLight(std::array<float, 3> const &position,
                                               std::array<float, 3> const &direction,
                                               float fovInner, float fovOuter,
                                               std::array<float, 3> const &color,
                                               bool enableShadow, float shadowNear,
                                               float shadowFar) {
  auto &light = mScene->addSpotLight();
  light.setPosition({position[0], position[1], position[2]});
  light.setDirection({direction[0], direction[1], direction[2]});
  light.setFov(fovOuter);
  light.setFovSmall(fovInner);
  light.setColor({color[0], color[1], color[2]});
  if (enableShadow) {
    light.enableShadow(true);
    light.setShadowParameters(shadowNear, shadowFar);
  }
  auto l = std::make_unique<SVulkan2SpotLight>(light);
  auto result = l.get();
  mLights.push_back(std::move(l));
  return result;
}

void SVulkan2Scene::removeLight(ILight *light) {
  if (auto l = dynamic_cast<SVulkan2DirectionalLight *>(light)) {
    mScene->removeNode(*l->getInternalLight());
  } else if (auto l = dynamic_cast<SVulkan2PointLight *>(light)) {
    mScene->removeNode(*l->getInternalLight());
  } else if (auto l = dynamic_cast<SVulkan2SpotLight *>(light)) {
    mScene->removeNode(*l->getInternalLight());
  }
  mLights.erase(std::remove_if(mLights.begin(), mLights.end(),
                               [light](auto &l) { return light == l.get(); }),
                mLights.end());
}

void SVulkan2Scene::destroy() { mParentRenderer->removeScene(this); }

IPxrRigidbody *SVulkan2Scene::addRigidbody(const std::string &meshFile,
                                           const physx::PxVec3 &scale) {
  if (!std::filesystem::exists(meshFile)) {
    mBodies.push_back(
        std::make_unique<SVulkan2Rigidbody>(this, std::vector<svulkan2::scene::Object *>{}));
    return mBodies.back().get();
  }

  auto model = mParentRenderer->mContext->getResourceManager()->CreateModelFromFile(meshFile);
  std::vector<svulkan2::scene::Object *> objects2;
  auto &obj = mScene->addObject(model);
  obj.setScale({scale.x, scale.y, scale.z});
  objects2.push_back(&obj);
  mBodies.push_back(std::make_unique<SVulkan2Rigidbody>(this, objects2));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(physx::PxGeometryType::Enum type,
                                           const physx::PxVec3 &scale,
                                           const physx::PxVec3 &color) {
  auto material = std::make_shared<svulkan2::resource::SVMetallicMaterial>(
      glm::vec4{color.x, color.y, color.z, 1.f});
  return addRigidbody(type, scale, std::make_shared<SVulkan2Material>(material));
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(physx::PxGeometryType::Enum type,
                                           const physx::PxVec3 &scale,
                                           std::shared_ptr<IPxrMaterial> material) {
  auto mat = std::dynamic_pointer_cast<SVulkan2Material>(material);
  if (!mat) {
    mat = std::static_pointer_cast<SVulkan2Material>(mParentRenderer->createMaterial());
  }
  svulkan2::scene::Object *object;
  switch (type) {
  case physx::PxGeometryType::eBOX: {
    if (!mCubeMesh) {
      mCubeMesh = svulkan2::resource::SVMesh::CreateCube();
    }
    auto shape = svulkan2::resource::SVShape::Create(mCubeMesh, mat->getMaterial());
    auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    obj.setScale({scale.x, scale.y, scale.z});
    object = &obj;
    break;
  }
  case physx::PxGeometryType::eSPHERE: {
    if (!mSphereMesh) {
      mSphereMesh = svulkan2::resource::SVMesh::CreateUVSphere(32, 16);
    }
    auto shape = svulkan2::resource::SVShape::Create(mSphereMesh, mat->getMaterial());
    auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    obj.setScale({scale.x, scale.y, scale.z});
    object = &obj;
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    if (!mPlaneMesh) {
      mPlaneMesh = svulkan2::resource::SVMesh::CreateYZPlane();
    }
    auto shape = svulkan2::resource::SVShape::Create(mPlaneMesh, mat->getMaterial());
    auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    obj.setScale({scale.x, scale.y, scale.z});
    object = &obj;
    break;
  }
  case physx::PxGeometryType::eCAPSULE: {
    auto mesh = svulkan2::resource::SVMesh::CreateCapsule(scale.y, scale.x, 32, 8);
    auto shape = svulkan2::resource::SVShape::Create(mesh, mat->getMaterial());
    auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData({shape}));
    obj.setScale({1, 1, 1});
    object = &obj;
    break;
  }
  default:
    throw std::runtime_error("Failed to ad rigidbody: unsupported render body type");
  }

  mBodies.push_back(
      std::make_unique<SVulkan2Rigidbody>(this, std::vector<svulkan2::scene::Object *>{object}));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                           std::vector<physx::PxVec3> const &normals,
                                           std::vector<uint32_t> const &indices,
                                           const physx::PxVec3 &scale,
                                           const physx::PxVec3 &color) {
  auto material = std::make_shared<svulkan2::resource::SVMetallicMaterial>(
      glm::vec4{color.x, color.y, color.z, 1.f});
  return addRigidbody(vertices, normals, indices, scale,
                      std::make_shared<SVulkan2Material>(material));
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                           std::vector<physx::PxVec3> const &normals,
                                           std::vector<uint32_t> const &indices,
                                           const physx::PxVec3 &scale,
                                           std::shared_ptr<IPxrMaterial> material) {
  auto mat = std::dynamic_pointer_cast<SVulkan2Material>(material);
  if (!mat) {
    mat = std::static_pointer_cast<SVulkan2Material>(mParentRenderer->createMaterial());
  }

  std::vector<float> vertices_;
  std::vector<float> normals_;
  for (auto &v : vertices) {
    vertices_.push_back(v.x);
    vertices_.push_back(v.y);
    vertices_.push_back(v.z);
  }
  for (auto &n : normals) {
    normals_.push_back(n.x);
    normals_.push_back(n.y);
    normals_.push_back(n.z);
  }

  auto mesh = svulkan2::resource::SVMesh::Create(vertices_, indices);
  mesh->setVertexAttribute("normal", normals_);
  auto shape = svulkan2::resource::SVShape::Create(mesh, mat->getMaterial());
  auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData({shape}));
  obj.setScale({scale.x, scale.y, scale.z});

  mBodies.push_back(
      std::make_unique<SVulkan2Rigidbody>(this, std::vector<svulkan2::scene::Object *>{&obj}));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::cloneRigidbody(SVulkan2Rigidbody *other) {
  auto &otherObjs = other->getVisualObjects();
  std::vector<svulkan2::scene::Object *> objs;
  for (auto &obj : otherObjs) {
    objs.push_back(&getScene()->addObject(obj->getParent(), obj->getModel()));
    objs.back()->setTransform(obj->getTransform());
  }
  mBodies.push_back(std::make_unique<SVulkan2Rigidbody>(this, objs));
  auto body = mBodies.back().get();
  body->setInitialPose(other->getInitialPose());
  return body;
}

void SVulkan2Scene::removeRigidbody(IPxrRigidbody *body) {
  for (auto it = mBodies.begin(); it != mBodies.end(); ++it) {
    if (it->get() == body) {
      it->get()->destroyVisualObjects();
      mBodies.erase(it);
      return;
    }
  }
}

ICamera *SVulkan2Scene::addCamera(std::string const &name, uint32_t width, uint32_t height,
                                  float fovx, float fovy, float near, float far,
                                  std::string const &shaderDir) {
  if (fovx != 0) {
    spdlog::get("SAPIEN")->warn(
        "Current camera implementation does not support non-square"
        "pixels, and fovy will be used. Set fovx to 0 to suppress this warning");
  }
  std::string shader = shaderDir.length() ? shaderDir : gDefaultCameraShaderDirectory;
  auto cam = std::make_unique<SVulkan2Camera>(name, width, height, fovy, near, far, this, shader);
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void SVulkan2Scene::removeCamera(ICamera *camera) {
  auto cam = dynamic_cast<SVulkan2Camera *>(camera);
  if (!cam) {
    return;
  }
  mScene->removeNode(*cam->getCamera());
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [camera](auto &c) { return camera == c.get(); }),
                 mCameras.end());
}

std::vector<ICamera *> SVulkan2Scene::getCameras() {
  std::vector<ICamera *> cams;
  for (auto &cam : mCameras) {
    cams.push_back(cam.get());
  }
  return cams;
}

void SVulkan2Scene::updateRender() { mScene->updateModelMatrices(); }

} // namespace Renderer
} // namespace sapien
