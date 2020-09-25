#include "optifuser_renderer.h"
#include <objectLoader.h>
#include <spdlog/spdlog.h>

namespace sapien {
namespace Renderer {

static std::vector<std::string> saveNames = {};

enum RenderMode {
  LIGHTING,
  ALBEDO,
  NORMAL,
  DEPTH,
  SEGMENTATION,
  CUSTOM
#ifdef _USE_OPTIX
  ,
  PATHTRACER
#endif
};

constexpr int WINDOW_WIDTH = 1200, WINDOW_HEIGHT = 800;

//======== Begin Rigidbody ========//

OptifuserRigidbody::OptifuserRigidbody(OptifuserScene *scene,
                                       std::vector<Optifuser::Object *> const &objects)
    : mParentScene(scene), mObjects(objects) {}

void OptifuserRigidbody::setUniqueId(uint32_t uniqueId) {
  mUniqueId = uniqueId;
  for (auto obj : mObjects) {
    obj->setObjId(uniqueId);
  }
}

uint32_t OptifuserRigidbody::getUniqueId() const { return mUniqueId; }

void OptifuserRigidbody::setSegmentationId(uint32_t segmentationId) {
  mSegmentationId = segmentationId;
  for (auto obj : mObjects) {
    obj->setSegmentId(segmentationId);
  }
}

uint32_t OptifuserRigidbody::getSegmentationId() const { return mSegmentationId; }

void OptifuserRigidbody::setSegmentationCustomData(const std::vector<float> &customData) {
  if (customData.size() != 16) {
    throw std::invalid_argument("custom data must have length 16");
  }
  for (auto obj : mObjects) {
    obj->setUserData(customData);
  }
}

void OptifuserRigidbody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void OptifuserRigidbody::update(const physx::PxTransform &transform) {
  auto pose = transform * mInitialPose;
  for (auto obj : mObjects) {
    obj->position = {pose.p.x, pose.p.y, pose.p.z};
    obj->setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
  }
}

void OptifuserRigidbody::destroy() { mParentScene->removeRigidbody(this); }
void OptifuserRigidbody::destroyVisualObjects() {
  for (auto obj : mObjects) {
    mParentScene->getScene()->removeObject(obj);
  }
}

void OptifuserRigidbody::setVisibility(float visibility) {
  for (auto obj : mObjects) {
    obj->visibility = visibility;
  }
}

void OptifuserRigidbody::setVisible(bool visible) {
  for (auto obj : mObjects) {
    obj->visibility = visible ? 1.f : 0.f;
  }
}

void OptifuserRigidbody::setRenderMode(uint32_t mode) {
  if (mode == 0) {
    for (auto obj : mObjects) {
      obj->visibility = 1.f;
    }
    return;
  }
  if (mode == 1) {
    for (auto obj : mObjects) {
      obj->visibility = 0.f;
    }
    return;
  }
  if (mode == 2) {
    for (auto obj : mObjects) {
      obj->visibility = 0.5f;
    }
  }
}

//======== End Rigidbody ========//

//======== Begin Scene ========//
OptifuserScene::OptifuserScene(OptifuserRenderer *renderer, std::string const &name)
    : mParentRenderer(renderer), mScene(std::make_unique<Optifuser::Scene>()), mName(name) {}

Optifuser::Scene *OptifuserScene::getScene() { return mScene.get(); }

IPxrRigidbody *OptifuserScene::addRigidbody(const std::string &meshFile,
                                            const physx::PxVec3 &scale) {
  auto objects = Optifuser::LoadObj(meshFile);
  std::vector<Optifuser::Object *> objs;
  if (objects.empty()) {
    spdlog::get("SAPIEN")->error("Failed to load damaged file: {}", meshFile);
    return nullptr;
  }
  for (auto &obj : objects) {
    obj->scale = {scale.x, scale.y, scale.z};
    objs.push_back(obj.get());
    mScene->addObject(std::move(obj));
  }

  mBodies.push_back(std::make_unique<OptifuserRigidbody>(this, objs));

  return mBodies.back().get();
}

IPxrRigidbody *OptifuserScene::addRigidbody(std::vector<physx::PxVec3> const &points,
                                            std::vector<physx::PxVec3> const &normals,
                                            std::vector<uint32_t> const &indices,
                                            const physx::PxVec3 &scale,
                                            const PxrMaterial &material) {
  std::vector<Optifuser::Vertex> vertices;
  for (uint32_t i = 0; i < points.size(); ++i) {
    vertices.push_back(
        {{points[i].x, points[i].y, points[i].z}, {normals[i].x, normals[i].y, normals[i].z}});
  }

  auto obj = Optifuser::NewObject<Optifuser::Object>(
      std::make_shared<Optifuser::TriangleMesh>(vertices, indices, false));
  {
    obj->pbrMaterial->kd = {material.base_color[0], material.base_color[1], material.base_color[2],
                            material.base_color[3]};
    obj->pbrMaterial->ks = material.specular;
    obj->pbrMaterial->roughness = material.roughness;
    obj->pbrMaterial->metallic = material.metallic;
  }

  obj->scale = {scale.x, scale.y, scale.z};

  mBodies.push_back(std::make_unique<OptifuserRigidbody>(this, std::vector{obj.get()}));

  mScene->addObject(std::move(obj));

  return mBodies.back().get();
}

IPxrRigidbody *OptifuserScene::addRigidbody(physx::PxGeometryType::Enum type,
                                            const physx::PxVec3 &scale,
                                            const PxrMaterial &material) {
  std::unique_ptr<Optifuser::Object> obj;
  switch (type) {
  case physx::PxGeometryType::eBOX: {
    obj = Optifuser::NewFlatCube();
    obj->scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::eSPHERE: {
    obj = Optifuser::NewSphere();
    obj->scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    obj = Optifuser::NewYZPlane();
    obj->scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::eCAPSULE: {
    obj = Optifuser::NewCapsule(scale.x, scale.y);
    obj->scale = {1, 1, 1};
    break;
  }
  default:
    spdlog::get("SAPIEN")->error("Failed to add Rididbody: unimplemented shape");
    return nullptr;
  }

  {
    obj->pbrMaterial->kd = {material.base_color[0], material.base_color[1], material.base_color[2],
                            material.base_color[3]};
    obj->pbrMaterial->ks = material.specular;
    obj->pbrMaterial->roughness = material.roughness;
    obj->pbrMaterial->metallic = material.metallic;
  }

  mBodies.push_back(std::make_unique<OptifuserRigidbody>(this, std::vector{obj.get()}));
  mScene->addObject(std::move(obj));

  return mBodies.back().get();
}

void OptifuserScene::removeRigidbody(IPxrRigidbody *body) {
  auto it = mBodies.begin();
  for (; it != mBodies.end(); ++it) {
    if (it->get() == body) {
      it->get()->destroyVisualObjects();
      mBodies.erase(it);
      return;
    }
  }
}

IPxrRigidbody *OptifuserScene::cloneRigidbody(OptifuserRigidbody *other) {
  auto &otherObjs = other->getVisualObjects();

  std::vector<Optifuser::Object *> objs;
  for (auto &obj : otherObjs) {
    auto newobj = obj->clone();
    objs.push_back(newobj.get());
    mScene->addObject(std::move(newobj));
  }
  mBodies.push_back(std::make_unique<OptifuserRigidbody>(this, objs));
  auto body = mBodies.back().get();
  body->setInitialPose(other->getInitialPose());
  return body;
}

ICamera *OptifuserScene::addCamera(std::string const &name, uint32_t width, uint32_t height,
                                   float fovx, float fovy, float near, float far,
                                   std::string const &shaderDir) {
  std::string d;
  if (shaderDir.length()) {
    d = shaderDir;
  } else {
    d = mParentRenderer->mGlslDir;
  }
  if (fovx != 0) {
    spdlog::get("SAPIEN")->warn(
        "Current camera implementation does not support non-square"
        "pixels, and fovy will be used. Set fovx to 0 to suppress this warning");
  }

  auto cam = std::make_unique<OptifuserCamera>(name, width, height, fovy, this, d,
                                               mParentRenderer->mConfig);
  cam->mCameraSpec->near = near;
  cam->mCameraSpec->far = far;
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void OptifuserScene::removeCamera(ICamera *camera) {
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [camera](auto &c) { return camera == c.get(); }),
                 mCameras.end());
}

std::vector<ICamera *> OptifuserScene::getCameras() {
  std::vector<ICamera *> cams;
  for (auto &cam : mCameras) {
    cams.push_back(cam.get());
  }
  return cams;
}

void OptifuserScene::destroy() { mParentRenderer->removeScene(this); }

void OptifuserScene::setAmbientLight(std::array<float, 3> const &color) {
  mScene->setAmbientLight({color[0], color[1], color[2]});
}

void OptifuserScene::setShadowLight(std::array<float, 3> const &direction,
                                    std::array<float, 3> const &color) {
  mScene->setShadowLight(
      {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
}

void OptifuserScene::addPointLight(std::array<float, 3> const &position,
                                   std::array<float, 3> const &color) {
  mScene->addPointLight({{position[0], position[1], position[2]}, {color[0], color[1], color[2]}});
}

void OptifuserScene::addDirectionalLight(std::array<float, 3> const &direction,
                                         std::array<float, 3> const &color) {
  mScene->addDirectionalLight(
      {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
}

//======== End Scene ========//

//======== Begin Renderer ========//

OptifuserRenderer::OptifuserRenderer(const std::string &glslDir, const std::string &glslVersion,
                                     OptifuserConfig const &config) {
  mConfig = config;

  if (glslDir.length()) {
    mGlslDir = glslDir;
  } else {
    mGlslDir = gDefaultGlslDir;
  }

  mContext = &Optifuser::GLFWRenderContext::Get(WINDOW_WIDTH, WINDOW_HEIGHT);
  mContext->initGui(glslVersion.length() ? glslVersion : gDefaultGlslVersion);

  mContext->renderer.enableAxisPass();
  mContext->renderer.enableDisplayPass();

  if (config.useShadow) {
    mContext->renderer.enableShadowPass(true, config.shadowMapSize, config.shadowFrustumSize);
  }
  if (config.useAo) {
    mContext->renderer.enableAOPass();
  }

  if (config.useShadow) {
    mContext->renderer.setShadowShader(mGlslDir + "/shadow.vsh", mGlslDir + "/shadow.fsh");
  }

  mContext->renderer.setGBufferShader(mGlslDir + "/gbuffer.vsh",
                                      mGlslDir + "/gbuffer_segmentation.fsh");
  if (config.useAo) {
    mContext->renderer.setAOShader(mGlslDir + "/ssao.vsh", mGlslDir + "/ssao.fsh");
  }
  mContext->renderer.setDeferredShader(mGlslDir + "/deferred.vsh", mGlslDir + "/deferred.fsh");
  mContext->renderer.setAxisShader(mGlslDir + "/axes.vsh", mGlslDir + "/axes.fsh");
  mContext->renderer.setTransparencyShader(mGlslDir + "/transparency.vsh",
                                           mGlslDir + "/transparency.fsh");
  mContext->renderer.setCompositeShader(mGlslDir + "/composite.vsh", mGlslDir + "/composite.fsh");
  mContext->renderer.setDisplayShader(mGlslDir + "/display.vsh", mGlslDir + "/display_normal.fsh");

  mContext->renderer.enablePicking();

  setLogLevel("warn");
}

IPxrScene *OptifuserRenderer::createScene(std::string const &name) {
  mScenes.push_back(std::make_unique<OptifuserScene>(this, name));
  return mScenes.back().get();
}

void OptifuserRenderer::removeScene(IPxrScene *scene) {
  mScenes.erase(std::remove_if(mScenes.begin(), mScenes.end(),
                               [scene](auto &s) { return scene == s.get(); }),
                mScenes.end());
}

std::string OptifuserRenderer::gDefaultGlslDir = "glsl_shader/130";
std::string OptifuserRenderer::gDefaultGlslVersion = "130";
void OptifuserRenderer::setDefaultShaderConfig(std::string const &glslDir,
                                               std::string const &glslVersion) {
  gDefaultGlslDir = glslDir;
  gDefaultGlslVersion = glslVersion;
}

#ifdef _USE_OPTIX
std::string OptifuserRenderer::gPtxDir = "ptx";
void OptifuserRenderer::setOptixConfig(std::string const &ptxDir) { gPtxDir = ptxDir; }
#endif

void OptifuserRenderer::enableGlobalAxes(bool enable) {
  mContext->renderer.enableGlobalAxes(enable);
}

void OptifuserRenderer::setLogLevel(std::string const &level) {
  if (level == "debug") {
    spdlog::get("Optifuser")->set_level(spdlog::level::debug);
  } else if (level == "info") {
    spdlog::get("Optifuser")->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    spdlog::get("Optifuser")->set_level(spdlog::level::warn);
  } else if (level == "err" || level == "error") {
    spdlog::get("Optifuser")->set_level(spdlog::level::err);
  } else {
    spdlog::get("Optifuser")->error("Invalid log level \"{}\"", level);
  }
}

//======== End Renderer ========//

} // namespace Renderer
} // namespace sapien
