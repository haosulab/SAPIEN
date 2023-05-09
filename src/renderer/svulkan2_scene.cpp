#include "sapien/renderer/svulkan2_scene.h"
#include "sapien/renderer/render_config.h"
#include "sapien/renderer/svulkan2_renderer.h"
#include "sapien/renderer/svulkan2_rigidbody.h"
#include "sapien/renderer/svulkan2_shape.h"
#include <filesystem>
#include <spdlog/spdlog.h>

namespace sapien {
namespace Renderer {

SVulkan2Scene::SVulkan2Scene(SVulkan2Renderer *renderer, std::string const &name)
    : mParentRenderer(renderer), mName(name) {
  mScene = std::make_shared<svulkan2::scene::Scene>();
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
                                                 float shadowFar, uint32_t shadowMapSize) {
  auto &light = mScene->addPointLight();
  light.setColor({color[0], color[1], color[2]});
  light.setTransform({.position = glm::vec4(position[0], position[1], position[2], 1.f)});
  if (enableShadow) {
    light.enableShadow(true);
    light.setShadowParameters(shadowNear, shadowFar, shadowMapSize);
  }
  auto l = std::make_unique<SVulkan2PointLight>(light);
  auto result = l.get();
  mLights.push_back(std::move(l));
  return result;
}

SVulkan2DirectionalLight *
SVulkan2Scene::addDirectionalLight(std::array<float, 3> const &direction,
                                   std::array<float, 3> const &color, bool enableShadow,
                                   std::array<float, 3> const &position, float shadowScale,
                                   float shadowNear, float shadowFar, uint32_t shadowMapSize) {
  auto &light = mScene->addDirectionalLight();
  light.setDirection({direction[0], direction[1], direction[2]});
  light.setColor({color[0], color[1], color[2]});
  if (enableShadow) {
    light.enableShadow(true);
    light.setPosition({position[0], position[1], position[2]});
    light.setShadowParameters(shadowNear, shadowFar, shadowScale, shadowMapSize);
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
                                               float shadowFar, uint32_t shadowMapSize) {
  auto &light = mScene->addSpotLight();
  light.setPosition({position[0], position[1], position[2]});
  light.setDirection({direction[0], direction[1], direction[2]});
  light.setFov(fovOuter);
  light.setFovSmall(fovInner);
  light.setColor({color[0], color[1], color[2]});
  if (enableShadow) {
    light.enableShadow(true);
    light.setShadowParameters(shadowNear, shadowFar, shadowMapSize);
  }
  auto l = std::make_unique<SVulkan2SpotLight>(light);
  auto result = l.get();
  mLights.push_back(std::move(l));
  return result;
}

SVulkan2ParallelogramLight *SVulkan2Scene::addParallelogramLight(
    physx::PxTransform const &pose, std::array<float, 3> const &color,
    std::array<float, 3> const &edge0, std::array<float, 3> const &edge1) {
  auto &light = mScene->addParallelogramLight();
  light.setTransform({.position = {pose.p.x, pose.p.y, pose.p.z},
                      .rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z}});
  light.setShape({edge0[0], edge0[1], edge0[2]}, {edge1[0], edge1[1], edge1[2]});
  light.setColor({color[0], color[1], color[2]});
  auto l = std::make_unique<SVulkan2ParallelogramLight>(light);
  auto result = l.get();
  mLights.push_back(std::move(l));
  return result;
}

IActiveLight *SVulkan2Scene::addActiveLight(physx::PxTransform const &pose,
                                            std::array<float, 3> const &color, float fov,
                                            std::string_view texPath, float shadowNear,
                                            float shadowFar, uint32_t shadowMapSize) {
  auto &light = mScene->addTexturedLight();
  light.setPosition({pose.p.x, pose.p.y, pose.p.z});
  light.setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
  light.setFov(fov);
  light.setFovSmall(fov);
  light.setColor({color[0], color[1], color[2]});
  light.enableShadow(true);
  light.setShadowParameters(shadowNear, shadowFar, shadowMapSize);
  auto tex = mParentRenderer->getContext()->getResourceManager()->CreateTextureFromFile(
      std::string(texPath), 1, vk::Filter::eLinear, vk::Filter::eLinear,
      vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder, true);
  light.setTexture(tex);
  auto l = std::make_unique<SVulkan2ActiveLight>(light);
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
  } else if (auto l = dynamic_cast<SVulkan2ActiveLight *>(light)) {
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
    spdlog::get("SAPIEN")->error("Failed to load visual mesh: " + meshFile);
    mBodies.push_back(
        std::make_unique<SVulkan2Rigidbody>(this, std::vector<svulkan2::scene::Object *>{},
                                            physx::PxGeometryType::eTRIANGLEMESH, scale));
    return mBodies.back().get();
  }

  auto model = mParentRenderer->getContext()->getResourceManager()->CreateModelFromFile(meshFile);
  std::vector<svulkan2::scene::Object *> objects2;
  auto &obj = mScene->addObject(model);
  obj.setScale({scale.x, scale.y, scale.z});
  objects2.push_back(&obj);
  mBodies.push_back(std::make_unique<SVulkan2Rigidbody>(
      this, objects2, physx::PxGeometryType::eTRIANGLEMESH, scale));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale,
                                           std::shared_ptr<IPxrMaterial> material) {
  if (!material) {
    return addRigidbody(meshFile, scale);
  }

  auto mat = std::dynamic_pointer_cast<SVulkan2Material>(material);

  if (!std::filesystem::exists(meshFile)) {
    spdlog::get("SAPIEN")->error("Failed to load visual mesh: " + meshFile);
    mBodies.push_back(
        std::make_unique<SVulkan2Rigidbody>(this, std::vector<svulkan2::scene::Object *>{},
                                            physx::PxGeometryType::eTRIANGLEMESH, scale));
    return mBodies.back().get();
  }
  auto model = mParentRenderer->getContext()->getResourceManager()->CreateModelFromFile(meshFile);
  model->loadAsync().get();

  std::vector<std::shared_ptr<svulkan2::resource::SVShape>> shapes;
  for (auto s : model->getShapes()) {
    auto shape = svulkan2::resource::SVShape::Create(s->mesh, mat->getMaterial());
    shapes.push_back(shape);
  }
  auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData(shapes));
  obj.setScale({scale.x, scale.y, scale.z});
  std::vector<svulkan2::scene::Object *> objects2;
  objects2.push_back(&obj);
  mBodies.push_back(std::make_unique<SVulkan2Rigidbody>(
      this, objects2, physx::PxGeometryType::eTRIANGLEMESH, scale));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(std::shared_ptr<IRenderMesh> mesh,
                                           const physx::PxVec3 &scale,
                                           std::shared_ptr<IPxrMaterial> material) {
  auto rmesh = std::dynamic_pointer_cast<SVulkan2Mesh>(mesh);
  auto mat = std::dynamic_pointer_cast<SVulkan2Material>(material);
  auto shape = svulkan2::resource::SVShape::Create(rmesh->getMesh(), mat->getMaterial());
  auto &obj = mScene->addObject(svulkan2::resource::SVModel::FromData({shape}));
  obj.setScale({scale.x, scale.y, scale.z});
  mBodies.push_back(std::make_unique<SVulkan2Rigidbody>(
      this, std::vector({&obj}), physx::PxGeometryType::eTRIANGLEMESH, scale));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(physx::PxGeometryType::Enum type,
                                           const physx::PxVec3 &scale,
                                           const physx::PxVec3 &color) {
  auto material = std::make_shared<svulkan2::resource::SVMetallicMaterial>(
      glm::vec4{0.f, 0.f, 0.f, 1.f}, glm::vec4{color.x, color.y, color.z, 1.f});
  return addRigidbody(type, scale, std::make_shared<SVulkan2Material>(material, mParentRenderer));
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

  mBodies.push_back(std::make_unique<SVulkan2Rigidbody>(
      this, std::vector<svulkan2::scene::Object *>{object}, type, scale));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                           std::vector<physx::PxVec3> const &normals,
                                           std::vector<uint32_t> const &indices,
                                           const physx::PxVec3 &scale,
                                           const physx::PxVec3 &color) {
  auto material = std::make_shared<svulkan2::resource::SVMetallicMaterial>(
      glm::vec4{0.f, 0.f, 0.f, 1.0f}, glm::vec4{color.x, color.y, color.z, 1.f});
  return addRigidbody(vertices, normals, indices, scale,
                      std::make_shared<SVulkan2Material>(material, getParentRenderer()));
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
      std::make_unique<SVulkan2Rigidbody>(this, std::vector<svulkan2::scene::Object *>{&obj},
                                          physx::PxGeometryType::eTRIANGLEMESH, scale));
  return mBodies.back().get();
}

IPxrRigidbody *SVulkan2Scene::cloneRigidbody(SVulkan2Rigidbody *other) {
  auto &otherObjs = other->getVisualObjects();
  std::vector<svulkan2::scene::Object *> objs;
  for (auto &obj : otherObjs) {
    objs.push_back(&getScene()->addObject(obj->getParent(), obj->getModel()));
    objs.back()->setTransform(obj->getTransform());
  }
  mBodies.push_back(
      std::make_unique<SVulkan2Rigidbody>(this, objs, other->getType(), other->getScale()));
  auto body = mBodies.back().get();
  body->setInitialPose(other->getInitialPose());
  return body;
}

IPxrPointBody *SVulkan2Scene::addPointBody(
    Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> positions) {
  auto pointset = std::make_shared<svulkan2::resource::SVPointSet>();
  pointset->setVertexAttribute(
      "position", std::vector<float>(positions.data(), positions.data() + positions.size()));
  auto &obj = mScene->addPointObject(pointset);
  mPointBodies.push_back(std::make_unique<SVulkan2PointBody>(this, &obj));
  return mPointBodies.back().get();
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

void SVulkan2Scene::removePointBody(IPxrPointBody *body) {
  for (auto it = mPointBodies.begin(); it != mPointBodies.end(); ++it) {
    if (it->get() == body) {
      it->get()->destroyVisualObject();
      mPointBodies.erase(it);
      return;
    }
  }
}

ICamera *SVulkan2Scene::addCamera(uint32_t width, uint32_t height, float fovy, float near,
                                  float far, std::string const &shaderDir) {
  std::string shader = shaderDir.length() ? shaderDir : GetRenderConfig().cameraShaderDirectory;
  auto cam = std::make_unique<SVulkan2Camera>(width, height, fovy, near, far, this, shader);
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

void SVulkan2Scene::setEnvironmentMap(std::string_view path) {
  auto tex = mParentRenderer->getContext()->getResourceManager()->CreateCubemapFromKTX(
      std::string(path), 5);
  mScene->setEnvironmentMap(tex);
}

void SVulkan2Scene::setEnvironmentMap(std::array<std::string_view, 6> paths) {
  auto tex = mParentRenderer->getContext()->getResourceManager()->CreateCubemapFromFiles(
      {
          std::string(paths[0]),
          std::string(paths[1]),
          std::string(paths[2]),
          std::string(paths[3]),
          std::string(paths[4]),
          std::string(paths[5]),
      },
      5);
  mScene->setEnvironmentMap(tex);
}

} // namespace Renderer
} // namespace sapien
