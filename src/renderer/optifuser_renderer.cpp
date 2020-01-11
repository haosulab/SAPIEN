#include "optifuser_renderer.h"
#include <objectLoader.h>
#include <spdlog/spdlog.h>
#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

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
  for (auto obj : mObjects) {
    obj->setObjId(uniqueId);
  }
}

void OptifuserRigidbody::setSegmentationId(uint32_t segmentationId) {
  for (auto obj : mObjects) {
    obj->setSegmentId(segmentationId);
  }
}

void OptifuserRigidbody::setSegmentationCustomData(const std::vector<float> &customData) {
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
    spdlog::error("Failed to load damaged file: {}", meshFile);
    return nullptr;
  }
  for (auto &obj : objects) {
    obj->scale = {scale.x, scale.y, scale.z};
    // TODO set id
    objs.push_back(obj.get());
    mScene->addObject(std::move(obj));
  }

  mBodies.push_back(std::make_unique<OptifuserRigidbody>(this, objs));

  return mBodies.back().get();
}

IPxrRigidbody *OptifuserScene::addRigidbody(physx::PxGeometryType::Enum type,
                                            const physx::PxVec3 &scale,
                                            const physx::PxVec3 &color) {
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
    obj = Optifuser::NewXYPlane();
    obj->scale = {scale.x, scale.y, scale.z};
    break;
  }
  case physx::PxGeometryType::eCAPSULE: {
    obj = Optifuser::NewCapsule(scale.x, scale.y);
    obj->scale = {1, 1, 1};
    break;
  }
  default:
    spdlog::error("Failed to add Rididbody: unimplemented shape");
    return nullptr;
  }

  obj->material.kd = {color.x, color.y, color.z, 1};

  mBodies.push_back(std::make_unique<OptifuserRigidbody>(this, std::vector{obj.get()}));
  mScene->addObject(std::move(obj));

  return mBodies.back().get();
}

void OptifuserScene::removeRigidbody(IPxrRigidbody *body) {
  auto it = mBodies.begin();
  for (; it != mBodies.end(); ++it) {
    if (it->get() == body) {
      mBodies.erase(it);
      return;
    }
  }
}

ICamera *OptifuserScene::addCamera(std::string const &name, uint32_t width, uint32_t height,
                                   float fovx, float fovy, float near, float far,
                                   std::string const &shaderDir) {
  spdlog::warn("Note: current camera implementation does not support non-square pixels, and fovy "
               "will take precedence.");
  auto cam = std::make_unique<OptifuserCamera>(name, width, height, fovy, this, shaderDir);
  cam->near = near;
  cam->far = far;
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void OptifuserScene::removeCamera(ICamera *camera) {
  std::remove_if(mCameras.begin(), mCameras.end(),
                 [camera](auto &c) { return camera == c.get(); });
}

std::vector<ICamera *> OptifuserScene::getCameras() {
  std::vector<ICamera *> cams;
  for (auto &cam : mCameras) {
    cams.push_back(cam.get());
  }
  return cams;
}

void OptifuserScene::destroy() { mParentRenderer->removeScene(this); }

void OptifuserScene::setAmbientLight(std::array<float, 3> color) {
  mScene->setAmbientLight({color[0], color[1], color[2]});
}

void OptifuserScene::setShadowLight(std::array<float, 3> direction, std::array<float, 3> color) {
  mScene->setShadowLight(
      {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
}

void OptifuserScene::addPointLight(std::array<float, 3> position, std::array<float, 3> color) {
  mScene->addPointLight({{position[0], position[1], position[2]}, {color[0], color[1], color[2]}});
}

void OptifuserScene::addDirectionalLight(std::array<float, 3> direction,
                                         std::array<float, 3> color) {
  mScene->addDirectionalLight(
      {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
}

//======== End Scene ========//

//======== Begin Renderer ========//

OptifuserRenderer::OptifuserRenderer(const std::string &glslDir, const std::string &glslVersion)
    : mGlslDir(glslDir) {
  mContext = &Optifuser::GLFWRenderContext::Get(WINDOW_WIDTH, WINDOW_HEIGHT);
  mContext->initGui(glslVersion);

  mContext->renderer.setShadowShader(glslDir + "/shadow.vsh", glslDir + "/shadow.fsh");
  mContext->renderer.setGBufferShader(glslDir + "/gbuffer.vsh",
                                      glslDir + "/gbuffer_segmentation.fsh");
  mContext->renderer.setDeferredShader(glslDir + "/deferred.vsh", glslDir + "/deferred.fsh");
  mContext->renderer.setAxisShader(glslDir + "/axes.vsh", glslDir + "/axes.fsh");
  mContext->renderer.enablePicking();
  mContext->renderer.enableAxisPass();
}

IPxrScene *OptifuserRenderer::createScene(std::string const &name) {
  mScenes.push_back(std::make_unique<OptifuserScene>(this, name));
  return mScenes.back().get();
}

void OptifuserRenderer::removeScene(IPxrScene *scene) {
  std::remove_if(mScenes.begin(), mScenes.end(), [scene](auto &s) { return scene == s.get(); });
}


//======== End Renderer ========//

// OptifuserRenderer::OptifuserRenderer(const std::string &version) : glslVersion(version) {
// init(); }

// void OptifuserRenderer::resetScene(uint32_t sceneId) {
//   if (mSceneData.size() <= sceneId) {
//     mSceneData.resize(sceneId + 1);
//   }
//   mSceneData[sceneId] = {};
//   mSceneData[sceneId].scene = std::make_unique<Optifuser::Scene>();
// }

// void OptifuserRenderer::addRigidbody(uint32_t sceneId, uint32_t uniqueId,
//                                      const std::string &objFile, const physx::PxVec3 &scale) {
//   auto objects = Optifuser::LoadObj(objFile);
//   mSceneData[sceneId].renderId2Objects[uniqueId] = {};
//   if (objects.empty()) {
//     std::cerr << "Damaged file detected: " << objFile << std::endl;
//   }
//   for (auto &obj : objects) {
//     mSceneData[sceneId].renderId2Objects[uniqueId].push_back(obj.get());
//     obj->scale = {scale.x, scale.y, scale.z};
//     obj->setObjId(uniqueId);
//     mSceneData[sceneId].scene->addObject(std::move(obj));
//   }
// }

// void OptifuserRenderer::addRigidbody(uint32_t sceneId, uint32_t uniqueId,
//                                      physx::PxGeometryType::Enum type, const physx::PxVec3
//                                      &scale, const physx::PxVec3 &color) {
//   switch (type) {
//   case physx::PxGeometryType::eBOX: {
//     auto obj = Optifuser::NewFlatCube();
//     obj->scale = {scale.x, scale.y, scale.z};
//     obj->material.kd = {color.x, color.y, color.z, 1};
//     obj->setObjId(uniqueId);
//     mSceneData[sceneId].renderId2Objects[uniqueId] = {obj.get()};
//     mSceneData[sceneId].scene->addObject(std::move(obj));
//     break;
//   }
//   case physx::PxGeometryType::eSPHERE: {
//     auto obj = Optifuser::NewSphere();
//     obj->scale = {scale.x, scale.y, scale.z};
//     obj->material.kd = {color.x, color.y, color.z, 1};
//     obj->setObjId(uniqueId);
//     mSceneData[sceneId].renderId2Objects[uniqueId] = {obj.get()};
//     mSceneData[sceneId].scene->addObject(std::move(obj));
//     break;
//   }
//   case physx::PxGeometryType::ePLANE: {
//     auto obj = Optifuser::NewYZPlane();
//     obj->scale = {scale.x, scale.y, scale.z};
//     obj->material.kd = {color.x, color.y, color.z, 1};
//     obj->setObjId(uniqueId);
//     mSceneData[sceneId].renderId2Objects[uniqueId] = {obj.get()};
//     mSceneData[sceneId].scene->addObject(std::move(obj));
//     break;
//   }
//   case physx::PxGeometryType::eCAPSULE: {
//     auto obj = Optifuser::NewCapsule(scale.x, scale.y);
//     obj->scale = {1, 1, 1};
//     obj->material.kd = {color.x, color.y, color.z, 1};
//     obj->setObjId(uniqueId);
//     mSceneData[sceneId].renderId2Objects[uniqueId] = {obj.get()};
//     mSceneData[sceneId].scene->addObject(std::move(obj));
//     break;
//   }
//   default:
//     std::cerr << "This shape is not Implemented" << std::endl;
//     break;
//   }
// }

// void OptifuserRenderer::setSegmentationId(uint32_t sceneId, uint32_t uniqueId,
//                                           uint32_t segmentationId) {

//   if (mSceneData[sceneId].renderId2Objects.find(uniqueId) ==
//       mSceneData[sceneId].renderId2Objects.end()) {
//     spdlog::critical("Invalid render id {} specified when setting segmentation id at "
//                      "OptifuserRenderer::setSegmentationId");
//     throw std::runtime_error("Invalid render id specified when setting segmentation id");
//   }
//   for (auto &obj : mSceneData[sceneId].renderId2Objects[uniqueId]) {
//     obj->setSegmentId(segmentationId);
//   }
//   mSceneData[sceneId].segId2RenderId[segmentationId].push_back(uniqueId);
// }

// void OptifuserRenderer::setSegmentationCustomData(uint32_t sceneId, uint32_t segmentationId,
//                                                   std::vector<float> const &customData) {
//   if (mSceneData[sceneId].segId2RenderId.find(segmentationId) ==
//       mSceneData[sceneId].segId2RenderId.end()) {
//     spdlog::critical("Invalid segmentation id {} specified in scene {} at "
//                      "OptifuserRenderer::setSegmentationCustomData",
//                      segmentationId, sceneId);
//     throw std::runtime_error("Invalid segmentation id");
//   }
//   for (uint32_t renderId : mSceneData[sceneId].segId2RenderId[segmentationId]) {
//     for (auto obj : mSceneData[sceneId].renderId2Objects[renderId]) {
//       obj->setUserData(customData);
//     }
//   }
// }

// void OptifuserRenderer::removeRigidbody(uint32_t sceneId, uint32_t uniqueId) {
//   if (mSceneData[sceneId].renderId2Objects.find(uniqueId) ==
//       mSceneData[sceneId].renderId2Objects.end()) {
//     spdlog::error("Failed to remove non-exist object with render id {} in scene {}", uniqueId,
//                   sceneId);
//   }
//   mSceneData[sceneId].renderId2Objects.erase(uniqueId);
// }

// void OptifuserRenderer::updateRigidbody(uint32_t sceneId, uint32_t uniqueId,
//                                         const physx::PxTransform &transform) {
//   if (mSceneData[sceneId].renderId2Objects.find(uniqueId) ==
//       mSceneData[sceneId].renderId2Objects.end()) {
//     spdlog::error("Failed to update non-exist object with render id {} in scene {}", uniqueId,
//                   sceneId);
//   }
//   for (auto obj : mSceneData[sceneId].renderId2Objects[uniqueId]) {
//     obj->position = {transform.p.x, transform.p.y, transform.p.z};
//     obj->setRotation({transform.q.w, transform.q.x, transform.q.y, transform.q.z});
//   }
// }

// void OptifuserRenderer::init() {
//   mContext = &Optifuser::GLFWRenderContext::Get(WINDOW_WIDTH, WINDOW_HEIGHT);
//   mContext->initGui(glslVersion);
//   // mScene = std::make_shared<Optifuser::Scene>();

//   cam.setUp({0, 0, 1});
//   cam.setForward({0, 1, 0});
//   cam.position = {0, 0, 1};
//   cam.rotateYawPitch(0, 0);
//   cam.fovy = glm::radians(45.f);
//   cam.aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;

//   mContext->renderer.setShadowShader("glsl_shader/" + glslVersion + "/shadow.vsh",
//                                      "glsl_shader/" + glslVersion + "/shadow.fsh");
//   mContext->renderer.setGBufferShader("glsl_shader/" + glslVersion + "/gbuffer.vsh",
//                                       "glsl_shader/" + glslVersion +
//                                       "/gbuffer_segmentation.fsh");
//   mContext->renderer.setDeferredShader("glsl_shader/" + glslVersion + "/deferred.vsh",
//                                        "glsl_shader/" + glslVersion + "/deferred.fsh");
//   mContext->renderer.setAxisShader("glsl_shader/" + glslVersion + "/axes.vsh",
//                                    "glsl_shader/" + glslVersion + "/axes.fsh");
//   mContext->renderer.enablePicking();
//   mContext->renderer.enableAxisPass();
// }

// void OptifuserRenderer::destroy() {
//   // TODO: check if scene needs special destroy handling
// }

// Optifuser::Scene *OptifuserRenderer::getScene(uint32_t sceneId) {
//   if (sceneId >= mSceneData.size()) {
//     return nullptr;
//   }
//   return mSceneData[sceneId].scene.get();
// }

// void OptifuserRenderer::render() {
// #ifdef _USE_OPTIX
//   static Optifuser::OptixRenderer *pathTracer = nullptr;
// #endif

//   static int renderMode = 0;
//   static float moveSpeed = 1.f;
//   mContext->processEvents();

//   float dt = 0.005f * moveSpeed;
//   if (Optifuser::getInput().getKeyState(GLFW_KEY_W)) {
//     cam.moveForwardRight(dt, 0);
// #ifdef _USE_OPTIX
//     if (renderMode == PATHTRACER) {
//       pathTracer->invalidateCamera();
//     }
// #endif
//   } else if (Optifuser::getInput().getKeyState(GLFW_KEY_S)) {
//     cam.moveForwardRight(-dt, 0);
// #ifdef _USE_OPTIX
//     if (renderMode == PATHTRACER) {
//       pathTracer->invalidateCamera();
//     }
// #endif
//   } else if (Optifuser::getInput().getKeyState(GLFW_KEY_A)) {
//     cam.moveForwardRight(0, -dt);
// #ifdef _USE_OPTIX
//     if (renderMode == PATHTRACER) {
//       pathTracer->invalidateCamera();
//     }
// #endif
//   } else if (Optifuser::getInput().getKeyState(GLFW_KEY_D)) {
//     cam.moveForwardRight(0, dt);
// #ifdef _USE_OPTIX
//     if (renderMode == PATHTRACER) {
//       pathTracer->invalidateCamera();
//     }
// #endif
//   }

//   cam.aspect =
//       static_cast<float>(mContext->getWidth()) / static_cast<float>(mContext->getHeight());

//   static bool renderGui = true;
//   if (Optifuser::getInput().getKeyDown(GLFW_KEY_E)) {
//     renderGui = !renderGui;
//   }
//   if (Optifuser::getInput().getMouseButton(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
//     double dx, dy;
//     Optifuser::getInput().getCursorDelta(dx, dy);
//     cam.rotateYawPitch(-dx / 1000.f, -dy / 1000.f);
// #ifdef _USE_OPTIX
//     if (renderMode == PATHTRACER) {
//       pathTracer->invalidateCamera();
//     }
// #endif
//   }

//   auto currentScene = getScene(mSelectedSceneId);

//   if (currentScene) {
//     mContext->renderer.renderScene(*currentScene, cam);

//     if (renderMode == SEGMENTATION) {
//       mContext->renderer.displaySegmentation();
//     } else if (renderMode == CUSTOM) {
//       mContext->renderer.displayUserTexture();
// #ifdef _USE_OPTIX
//     } else if (renderMode == PATHTRACER) {
//       // path tracer
//       pathTracer->numRays = 4;
//       pathTracer->max_iterations = 100000;
//       pathTracer->renderScene(*currentScene, cam);
//       pathTracer->display();
// #endif
//     } else {
//       mContext->renderer.displayLighting();
//     }
//   }

//   static GuiInfo pickedInfo;
//   if (currentScene) {
//     if (Optifuser::getInput().getMouseDown(GLFW_MOUSE_BUTTON_LEFT)) {
//       int x, y;
//       Optifuser::getInput().getCursor(x, y);
//       pickedId = mContext->renderer.pickSegmentationId(x, y);
//       if (pickedId) {
//         pickedRenderId = mContext->renderer.pickObjectId(x, y);
//       } else {
//         pickedRenderId = 0;
//       }
//     }
//     if (pickedId) {
//       pickedInfo = queryCallback(pickedId);
//       auto &pos = pickedInfo.linkInfo.transform.p;
//       auto &quat = pickedInfo.linkInfo.transform.q;
//       currentScene->clearAxes();
//       currentScene->addAxes({pos.x, pos.y, pos.z}, {quat.w, quat.x, quat.y, quat.z});
//     }
//   }

//   static const uint32_t imguiWindowSize = 300;
//   static int camIndex = -1;
//   if (renderGui) {
//     ImGui_ImplOpenGL3_NewFrame();
//     ImGui_ImplGlfw_NewFrame();
//     ImGui::NewFrame();

//     ImGui::Begin("Save##window");
//     {
//       if (ImGui::CollapsingHeader("Save")) {
//         static char buf[1000];
//         ImGui::InputText("##input_buffer", buf, 1000);
//         if (ImGui::Button("Save##button")) {
//           std::cout << "save called" << std::endl;
//           saveCallback(saveNames.size(), std::string(buf));
//         }
//       }
//       if (ImGui::CollapsingHeader("Load")) {
//         for (uint32_t i = 0; i < saveNames.size(); ++i) {
//           ImGui::Text("%s", saveNames[i].c_str());
//           ImGui::SameLine(100);
//           if (ImGui::Button(("load##" + std::to_string(i)).c_str())) {
//             saveActionCallback(i, 0);
//           }
//           ImGui::SameLine(150);
//           if (ImGui::Button(("delete##" + std::to_string(i)).c_str())) {
//             saveActionCallback(i, 1);
//           }
//         }
//       }
//     }
//     ImGui::End();

//     ImGui::SetNextWindowPos(ImVec2(0, 0));
//     ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mContext->getHeight()));

//     ImGui::Begin("Render Options");
//     {
//       if (ImGui::CollapsingHeader("Render Mode", ImGuiTreeNodeFlags_DefaultOpen)) {
//         if (ImGui::RadioButton("Lighting", &renderMode, RenderMode::LIGHTING)) {
//           mContext->renderer.setDeferredShader("glsl_shader/deferred.vsh",
//                                                "glsl_shader/deferred.fsh");
//         };
//         if (ImGui::RadioButton("Albedo", &renderMode, RenderMode::ALBEDO)) {
//           mContext->renderer.setDeferredShader("glsl_shader/deferred.vsh",
//                                                "glsl_shader/deferred_albedo.fsh");
//         }
//         if (ImGui::RadioButton("Normal", &renderMode, RenderMode::NORMAL)) {
//           mContext->renderer.setDeferredShader("glsl_shader/deferred.vsh",
//                                                "glsl_shader/deferred_normal.fsh");
//         }
//         if (ImGui::RadioButton("Depth", &renderMode, RenderMode::DEPTH)) {
//           mContext->renderer.setDeferredShader("glsl_shader/deferred.vsh",
//                                                "glsl_shader/deferred_depth.fsh");
//         }
//         if (ImGui::RadioButton("Segmentation", &renderMode, RenderMode::SEGMENTATION)) {
//           mContext->renderer.setGBufferShader("glsl_shader/gbuffer.vsh",
//                                               "glsl_shader/gbuffer_segmentation.fsh");
//         }
//         if (ImGui::RadioButton("Custom", &renderMode, RenderMode::CUSTOM)) {
//           mContext->renderer.setGBufferShader("glsl_shader/gbuffer.vsh",
//                                               "glsl_shader/gbuffer_segmentation.fsh");
//         }
// #ifdef _USE_OPTIX
//         if (ImGui::RadioButton("PathTracer", &renderMode, RenderMode::PATHTRACER)) {
//           if (pathTracer) {
//             delete pathTracer;
//           }
//           pathTracer = new Optifuser::OptixRenderer();
//           //          pathTracer->setBlackBackground();
//           pathTracer->init(mContext->getWidth(), mContext->getHeight());
//         } else {
//         }
// #endif
//       }

// #ifdef _USE_OPTIX
//       if (renderMode == PATHTRACER) {
//         glEnable(GL_FRAMEBUFFER_SRGB);
//       } else {
//         glDisable(GL_FRAMEBUFFER_SRGB);
//       }
// #endif

//       if (ImGui::CollapsingHeader("Main Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
//         ImGui::Text("Position");
//         ImGui::Text("%-4.3f %-4.3f %-4.3f", cam.position.x, cam.position.y, cam.position.z);
//         ImGui::Text("Forward");
//         auto forward = cam.getRotation() * glm::vec3(0, 0, -1);
//         ImGui::Text("%-4.3f %-4.3f %-4.3f", forward.x, forward.y, forward.z);
//         ImGui::Text("Fov");
//         ImGui::SliderAngle("##fov(y)", &cam.fovy, 1.f, 90.f);
//         ImGui::Text("Move speed");
//         ImGui::SliderFloat("##speed", &moveSpeed, 1.f, 100.f);
//         ImGui::Text("Width: %d", mContext->getWidth());
//         ImGui::SameLine();
//         ImGui::Text("Height: %d", mContext->getHeight());
//         ImGui::SameLine();
//         ImGui::Text("Aspect: %.2f", cam.aspect);
//         ImGui::Text("Picked link id: %d", pickedId);
//         ImGui::Text("Picked render id: %d", pickedRenderId);
//       }

//       if (ImGui::CollapsingHeader("Mounted Cameras")) {
//         ImGui::RadioButton("None##camera", &camIndex, -1);

//         if (currentScene) {
//           auto &mountedCameras = mSceneData[mSelectedSceneId].mountedCameras;

//           for (auto &cam : mountedCameras) {
//             ImGui::RadioButton(
//                 (mountedCameras[cam.first]->getName() + "##camera" +
//                 std::to_string(cam.first))
//                     .c_str(),
//                 &camIndex, cam.first);
//           }

//           if (camIndex >= 0) {
//             uint32_t width = mountedCameras[camIndex]->getWidth();
//             uint32_t height = mountedCameras[camIndex]->getHeight();
//             mountedCameras[camIndex]->takePicture();
//             ImGui::Image(
//                 reinterpret_cast<ImTextureID>(
//                     mountedCameras[camIndex]->mRenderContext->renderer.outputtex),
//                 ImVec2(imguiWindowSize, imguiWindowSize / static_cast<float>(width) * height),
//                 ImVec2(0, 1), ImVec2(1, 0));
//           }
//         }
//       }

//       if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {//
//       ImGui::Text("Frame Time: %.3f ms/frame (%.1f FPS)", 1000.0f /
//         ImGui::GetIO().Framerate,
//                     ImGui::GetIO().Framerate);
//       }
//     }
//     ImGui::End();

//     if (pickedId) {
//       ImGui::SetNextWindowPos(ImVec2(mContext->getWidth() - imguiWindowSize, 0));
//       ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mContext->getHeight()));
//       ImGui::Begin("Selected Object");
//       {
//         if (ImGui::CollapsingHeader("Actor", ImGuiTreeNodeFlags_DefaultOpen)) {
//           ImGui::Text("name: %s", pickedInfo.linkInfo.name.c_str());
//         }
//         if (ImGui::CollapsingHeader("Articulation", ImGuiTreeNodeFlags_DefaultOpen)) {
//           ImGui::Text("name: %s", pickedInfo.articulationInfo.name.c_str());
//           ImGui::Text(" dof: %ld", pickedInfo.articulationInfo.jointInfo.size());
//           int i = 0;
//           for (auto &jointInfo : pickedInfo.articulationInfo.jointInfo) {
//             ImGui::Text("%s", jointInfo.name.c_str());
//             if (ImGui::SliderFloat(("##" + std::to_string(++i)).c_str(), &jointInfo.value,
//                                    jointInfo.limits[0], jointInfo.limits[1])) {
//               syncCallback(pickedId, pickedInfo);
//             }
//           }
//         }
//       }
//       ImGui::End();
//     }

//     ImGui::Render();
//     ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
//   }
//   mContext->swapBuffers();
// }

// void OptifuserRenderer::setCurrentScene(OptifuserScene &currentScene) {
//   OptifuserScene *scene = &currentScene;
//   mSceneIndex =
//       std::find_if(mScenes.begin(), mScenes.end(), [scene](auto &s) { return s.get() == scene; }) -
//       mScenes.begin();
//   if (mSceneIndex >= mScenes.size()) {
//     mSceneIndex = -1;
//   }
// }

// void OptifuserRenderer::bindQueryCallback(std::function<GuiInfo(uint32_t)> callback) {
//   queryCallback = callback;
// }

// void OptifuserRenderer::bindSyncCallback(
//     std::function<void(uint32_t, const GuiInfo &info)> callback) {
//   syncCallback = callback;
// }

// std::vector<ICamera *> OptifuserRenderer::getCameras(uint32_t sceneId) {
//   std::vector<ICamera *> output;
//   for (auto &cam : mSceneData[sceneId].mountedCameras) {
//     output.push_back(cam.second.get());
//   }
//   return output;
// }

// void OptifuserRenderer::addCamera(uint32_t sceneId, uint32_t uniqueId, std::string const &name,
//                                   uint32_t width, uint32_t height, float fovx, float fovy,
//                                   float near, float far) {
//   auto scene = getScene(sceneId);
//   if (!scene) {
//     spdlog::error("Failed to add camera to non-exist scene");
//     return;
//   }
//   spdlog::warn("Note: current camera implementation does not support non-square pixels, and fovy
//   "
//                "will take precedence.");
//   auto cam = std::make_unique<MountedCamera>(name, width, height, fovy, scene);
//   cam->near = near;
//   cam->far = far;
//   mSceneData[sceneId].mountedCameras[uniqueId] = std::move(cam);
// }

// void OptifuserRenderer::updateCamera(uint32_t sceneId, uint32_t uniqueId,
//                                      physx::PxTransform const &transform) {
//   auto scene = getScene(sceneId);
//   if (!scene) {
//     spdlog::error("Failed to add camera to non-exist scene {}", sceneId);
//     return;
//   }
//   if (mSceneData[sceneId].mountedCameras.find(uniqueId) ==
//       mSceneData[sceneId].mountedCameras.end()) {
//     spdlog::error("Failed to find camera with id {} for update", uniqueId);
//     return;
//   }

//   mSceneData[sceneId].mountedCameras[uniqueId]->position = {transform.p.x, transform.p.y,
//                                                             transform.p.z};
//   mSceneData[sceneId].mountedCameras[uniqueId]->setRotation(
//       {transform.q.w, transform.q.x, transform.q.y, transform.q.z});
// }

// void OptifuserRenderer::setAmbientLight(uint32_t sceneId, std::array<float, 3> color) {
//   mSceneData[sceneId].scene->setAmbientLight({color[0], color[1], color[2]});
// }

// void OptifuserRenderer::setShadowLight(uint32_t sceneId, std::array<float, 3> direction,
//                                        std::array<float, 3> color) {
//   mSceneData[sceneId].scene->setShadowLight(
//       {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
// }

// void OptifuserRenderer::addDirectionalLight(uint32_t sceneId, std::array<float, 3> direction,
//                                             std::array<float, 3> color) {
//   mSceneData[sceneId].scene->addDirectionalLight(
//       {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
// }

// void OptifuserRenderer::addPointLight(uint32_t sceneId, std::array<float, 3> position,
//                                       std::array<float, 3> color) {

//   mSceneData[sceneId].scene->addPointLight(
//       {{position[0], position[1], position[2]}, {color[0], color[1], color[2]}});
// }

// void OptifuserRenderer::selectScene(uint32_t sceneId) {
//   if (sceneId >= mSceneData.size() || !mSceneData[sceneId].scene) {
//     spdlog::critical("Failed to select non-exist scene");
//     throw std::runtime_error("Failed to select non-exist scene");
//   }
//   mSelectedSceneId = sceneId;
// }

// void OptifuserRenderer::showWindow() { mContext->showWindow(); }

// void OptifuserRenderer::hideWindow() { mContext->hideWindow(); }

// void OptifuserRenderer::setSaveNames(std::vector<std::string> const &names) { saveNames = names;
// }

// void OptifuserRenderer::bindSaveActionCallback(
//     std::function<void(uint32_t index, uint32_t action)> func) {
//   saveActionCallback = func;
// }
// void OptifuserRenderer::bindSaveCallback(
//     std::function<void(uint32_t index, std::string const &name)> func) {
//   saveCallback = func;
// }

} // namespace Renderer
} // namespace sapien
