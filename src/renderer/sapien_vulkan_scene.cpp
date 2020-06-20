#ifdef _USE_VULKAN
#include "sapien_vulkan_renderer.h"

namespace sapien {
namespace Renderer {

SapienVulkanScene::SapienVulkanScene(SapienVulkanRenderer *renderer, std::string const &name)
    : mParentRenderer(renderer), mName(name) {
  mScene = std::make_unique<svulkan::Scene>(renderer->mContext->createVulkanScene());
}

void SapienVulkanScene::setAmbientLight(std::array<float, 3> const &color) {
  mScene->setAmbientLight({color[0], color[1], color[2], 1.f});
}

void SapienVulkanScene::addPointLight(std::array<float, 3> const &position,
                                      std::array<float, 3> const &color) {
  mScene->addPointLight(
      {{position[0], position[1], position[2], 1.f}, {color[0], color[1], color[2], 1.f}});
}

void SapienVulkanScene::addDirectionalLight(std::array<float, 3> const &direction,
                                            std::array<float, 3> const &color) {
  mScene->addDirectionalLight(
      {{direction[0], direction[1], direction[2], 1.f}, {color[0], color[1], color[2], 1.f}});
}

void SapienVulkanScene::setShadowLight(std::array<float, 3> const &direction,
                                       std::array<float, 3> const &color) {
  mScene->addDirectionalLight(
      {{direction[0], direction[1], direction[2], 1.f}, {color[0], color[1], color[2], 1.f}});
}

void SapienVulkanScene::destroy() { mParentRenderer->removeScene(this); };

// rigidbody

IPxrRigidbody *SapienVulkanScene::addRigidbody(const std::string &meshFile,
                                               const physx::PxVec3 &scale) {
  auto objects = mParentRenderer->mContext->loadObjects(meshFile, {scale.x, scale.y, scale.z});

  std::vector<svulkan::Object *> objects2;
  objects2.reserve(objects.size());
  for (auto &obj : objects) {
    objects2.push_back(obj.get());
    mScene->addObject(std::move(obj));
  }
  mBodies.push_back(std::make_unique<SapienVulkanRigidbody>(this, objects2));
  return mBodies.back().get();
}

IPxrRigidbody *SapienVulkanScene::addRigidbody(physx::PxGeometryType::Enum type,
                                               const physx::PxVec3 &scale,
                                               const PxrMaterial &material) {
  std::unique_ptr<svulkan::Object> obj;
  switch (type) {
  case physx::PxGeometryType::eBOX: {
    obj = mParentRenderer->mContext->loadCube();
    obj->mTransform.scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::eSPHERE: {
    obj = mParentRenderer->mContext->loadSphere();
    obj->mTransform.scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    obj = mParentRenderer->mContext->loadYZPlane();
    obj->mTransform.scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::eCAPSULE: {
    obj = mParentRenderer->mContext->loadCapsule(scale.y, scale.x);
    obj->mTransform.scale = {1, 1, 1};
    break;
  }
  default:
    throw std::runtime_error("Failed to ad rigidbody: unsupported render body type");
  }

  svulkan::PBRMaterialUBO mat;
  mat.baseColor = {material.base_color[0], material.base_color[1], material.base_color[2],
                   material.base_color[3]};
  mat.specular = material.specular;
  mat.roughness = material.roughness;
  mat.metallic = material.metallic;
  obj->updateMaterial(mat);

  mBodies.push_back(std::make_unique<SapienVulkanRigidbody>(this, std::vector{obj.get()}));
  mScene->addObject(std::move(obj));
  return mBodies.back().get();
}

IPxrRigidbody *SapienVulkanScene::addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                               std::vector<physx::PxVec3> const &normals,
                                               std::vector<uint32_t> const &indices,
                                               const physx::PxVec3 &scale,
                                               const PxrMaterial &material) {
  std::shared_ptr<svulkan::VulkanMesh> vulkanMesh{nullptr};
  std::vector<svulkan::Vertex> vs;
  std::vector<uint32_t> idx;
  for (uint32_t i = 0; i < vertices.size(); ++i) {
    vs.push_back({{vertices[i].x, vertices[i].y, vertices[i].z},
                  {normals[i].x, normals[i].y, normals[i].z}});
  }
  for (uint32_t i = 0; i < indices.size(); ++i) {
    idx.push_back(indices[i]);
  }
  vulkanMesh = std::make_shared<svulkan::VulkanMesh>(
      mParentRenderer->mContext->getPhysicalDevice(), mParentRenderer->mContext->getDevice(),
      mParentRenderer->mContext->getCommandPool(), mParentRenderer->mContext->getGraphicsQueue(),
      vs, idx, false);
  std::unique_ptr<svulkan::VulkanObject> vobj = std::make_unique<svulkan::VulkanObject>(
      mParentRenderer->mContext->getPhysicalDevice(), mParentRenderer->mContext->getDevice(),
      mParentRenderer->mContext->getDescriptorPool(),
      mParentRenderer->mContext->getDescriptorSetLayouts().object.get());

  vobj->setMesh(vulkanMesh);
  std::shared_ptr<svulkan::VulkanMaterial> mat = mParentRenderer->mContext->createMaterial();
  vobj->setMaterial(mat);
  auto obj = std::make_unique<svulkan::Object>(std::move(vobj));

  mBodies.push_back(std::make_unique<SapienVulkanRigidbody>(this, std::vector{obj.get()}));
  mScene->addObject(std::move(obj));
  return mBodies.back().get();
}

void SapienVulkanScene::removeRigidbody(IPxrRigidbody *body) {
  auto it = mBodies.begin();
  for (; it != mBodies.end(); ++it) {
    if (it->get() == body) {
      it->get()->destroyVisualObjects();
      mBodies.erase(it);
      return;
    }
  }
}

ICamera *SapienVulkanScene::addCamera(std::string const &name, uint32_t width, uint32_t height,
                                      float fovx, float fovy, float near, float far,
                                      std::string const &shaderDir) {
  if (fovx != 0) {
    spdlog::get("SAPIEN")->warn(
        "Current camera implementation does not support non-square"
        "pixels, and fovy will be used. Set fovx to 0 to suppress this warning");
  }
  auto cam =
      std::make_unique<SapienVulkanCamera>(name, width, height, fovy, near, far, this, shaderDir);
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void SapienVulkanScene::removeCamera(ICamera *camera) {
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [camera](auto &c) { return camera == c.get(); }),
                 mCameras.end());
}

std::vector<ICamera *> SapienVulkanScene::getCameras() {
  std::vector<ICamera *> cams;
  for (auto &cam : mCameras) {
    cams.push_back(cam.get());
  }
  return cams;
}

} // namespace Renderer
} // namespace sapien
#endif
