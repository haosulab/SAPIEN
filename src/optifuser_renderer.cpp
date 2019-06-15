#include "optifuser_renderer.h"
#include <objectLoader.h>

constexpr int WINDOW_WIDTH = 1024, WINDOW_HEIGHT = 768;

void OptifuserRenderer::addRigidbody(uint64_t uniqueId,
                                     const std::string &objFile) {
  if (mObjectRegistry.find(uniqueId) != mObjectRegistry.end()) {
    std::cerr << "Object already added" << std::endl;
    exit(1);
  }
  auto objects = Optifuser::LoadObj(objFile);
  mObjectRegistry[uniqueId] = objects;
  for (auto obj : objects) {
    mScene->addObject(obj);
  }

#ifdef _DEBUG
  printf("Adding Object %ld from %s\n", uniqueId, objFile.c_str());
#endif
}

void OptifuserRenderer::addRigidbody(uint64_t uniqueId,
                                     physx::PxGeometryType::Enum type,
                                     physx::PxVec3 scale) {
  if (mObjectRegistry.find(uniqueId) != mObjectRegistry.end()) {
    std::cerr << "Object already added" << std::endl;
    exit(1);
  }

  switch (type) {
  case physx::PxGeometryType::eBOX: {
    auto obj = Optifuser::NewCube();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {1, 1, 1};
    mScene->addObject(obj);
    mObjectRegistry[uniqueId] = {obj};
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    auto obj = Optifuser::NewYZPlane();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {1, 1, 1};
    mScene->addObject(obj);
    mObjectRegistry[uniqueId] = {obj};
    break;
  }
  default:
    std::cerr << "This shape is Not Implemented" << std::endl;
    break;
  }
}

void OptifuserRenderer::removeRigidbody(uint64_t uniqueId) {
  if (mObjectRegistry.find(uniqueId) == mObjectRegistry.end()) {
    std::cerr << "Object does not exist" << std::endl;
    exit(1);
  }
  mObjectRegistry.erase(uniqueId);
}

void OptifuserRenderer::updateRigidbody(uint64_t uniqueId,
                                        const physx::PxTransform &transform) {
  if (mObjectRegistry.find(uniqueId) == mObjectRegistry.end()) {
    std::cerr << "Object does not exist" << std::endl;
    exit(1);
  }
  for (auto obj : mObjectRegistry[uniqueId]) {
    obj->position = {transform.p.x, transform.p.y, transform.p.z};
    obj->rotation = {transform.q.w, transform.q.x, transform.q.y,
                     transform.q.z};
  }
}

void OptifuserRenderer::init() {
  mContext = &Optifuser::GLFWRenderContext::Get(WINDOW_WIDTH, WINDOW_HEIGHT);
  mScene = std::make_shared<Optifuser::Scene>();
  auto cam = Optifuser::NewObject<Optifuser::Camera>();
  cam->position = {0, 4, 8};
  cam->rotatePitch(-0.3);
  cam->fovy = glm::radians(45.f);
  cam->aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;
  mScene->addObject(cam);
  mScene->setMainCamera(cam);
  mScene->addDirectionalLight({{0, -1, -1}, {1, 1, 1}});
  mScene->setAmbientLight(glm::vec3(0.1, 0.1, 0.1));

  mScene->setEnvironmentMap("../assets/ame_desert/desertsky_ft.tga",
                            "../assets/ame_desert/desertsky_bk.tga",
                            "../assets/ame_desert/desertsky_up.tga",
                            "../assets/ame_desert/desertsky_dn.tga",
                            "../assets/ame_desert/desertsky_lf.tga",
                            "../assets/ame_desert/desertsky_rt.tga");

  mContext->renderer.setGBufferShader("../glsl_shader/gbuffer.vsh",
                                      "../glsl_shader/gbuffer.fsh");
  mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                       "../glsl_shader/deferred.fsh");
}

void OptifuserRenderer::destroy() {
  // TODO: check if scene needs special destroy handling
}
void OptifuserRenderer::render() {
  mContext->processEvents();

  float dt = 0.05f;
  if (mScene->getMainCamera()) {
    if (Optifuser::getInput().getKeyState(GLFW_KEY_W)) {
      mScene->getMainCamera()->move(0, 0, 2 * dt);
    } else if (Optifuser::getInput().getKeyState(GLFW_KEY_S)) {
      mScene->getMainCamera()->move(0, 0, -dt);
    }
    if (Optifuser::getInput().getKeyState(GLFW_KEY_A)) {
      mScene->getMainCamera()->move(0, -dt, 0);
    } else if (Optifuser::getInput().getKeyState(GLFW_KEY_D)) {
      mScene->getMainCamera()->move(0, dt, 0);
    }

    if (Optifuser::getInput().getMouseButton(GLFW_MOUSE_BUTTON_RIGHT) ==
        GLFW_PRESS) {
      double dx, dy;
      Optifuser::getInput().getCursor(dx, dy);
      mScene->getMainCamera()->rotateYaw(-dx / 1000.f);
      mScene->getMainCamera()->rotatePitch(-dy / 1000.f);
    }
  }

  mContext->render(mScene);
}
