#include "optifuser_renderer.h"
#include <objectLoader.h>
#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace sapien {
namespace Renderer {

enum RenderMode { LIGHTING, ALBEDO, NORMAL, DEPTH, SEGMENTATION, CUSTOM };

constexpr int WINDOW_WIDTH = 1200, WINDOW_HEIGHT = 800;

OptifuserRenderer::OptifuserRenderer() { init(); }

void OptifuserRenderer::addRigidbody(uint32_t uniqueId, const std::string &objFile,
                                     const physx::PxVec3 &scale) {
  auto objects = Optifuser::LoadObj(objFile);
  mObjectRegistry[uniqueId] = {};
  if (objects.empty()) {
    std::cerr << "Damaged file detected: " << objFile << std::endl;
  }
  for (auto &obj : objects) {
    mObjectRegistry[uniqueId].push_back(obj.get());
    obj->scale = {scale.x, scale.y, scale.z};
    obj->setObjId(uniqueId);
    mScene->addObject(std::move(obj));
  }

#ifdef _VERBOSE
  printf("Adding Object %d from %s\n", uniqueId, objFile.c_str());
#endif
}

void OptifuserRenderer::addRigidbody(uint32_t uniqueId, physx::PxGeometryType::Enum type,
                                     const physx::PxVec3 &scale, const physx::PxVec3 &color) {
  switch (type) {
  case physx::PxGeometryType::eBOX: {
    auto obj = Optifuser::NewFlatCube();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {color.x, color.y, color.z};
    obj->setObjId(uniqueId);
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  case physx::PxGeometryType::eSPHERE: {
    auto obj = Optifuser::NewSphere();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {color.x, color.y, color.z};
    obj->setObjId(uniqueId);
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    auto obj = Optifuser::NewYZPlane();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {color.x, color.y, color.z};
    obj->setObjId(uniqueId);
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  case physx::PxGeometryType::eCAPSULE: {
    auto obj = Optifuser::NewCapsule(scale.x, scale.y);
    obj->scale = {1, 1, 1};
    obj->material.kd = {color.x, color.y, color.z};
    obj->setObjId(uniqueId);
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  default:
    std::cerr << "This shape is not Implemented" << std::endl;
    break;
  }
}

void OptifuserRenderer::setSegmentationId(uint32_t uniqueId, uint32_t segmentationId) {
  if (mObjectRegistry.find(uniqueId) == mObjectRegistry.end()) {
    throw std::runtime_error("Invalid render id specified when setting segmentation id.");
  }
  for (auto &obj : mObjectRegistry[uniqueId]) {
    obj->setSegmentId(segmentationId);
  }
  mSegId2RenderId[segmentationId].push_back(uniqueId);
}

void OptifuserRenderer::setSegmentationCustomData(uint32_t segmentationId,
                                                  std::vector<float> const &customData) {
  if (mSegId2RenderId.find(segmentationId) == mSegId2RenderId.end()) {
    throw std::runtime_error("Invalid segmentation id.");
  }
  for (uint32_t renderId : mSegId2RenderId[segmentationId]) {
    for (auto obj : mObjectRegistry[renderId]) {
      obj->setUserData(customData);
    }
  }
}

void OptifuserRenderer::removeRigidbody(uint32_t uniqueId) {
  if (mObjectRegistry.find(uniqueId) == mObjectRegistry.end()) {
    std::cerr << "Object does not exist" << std::endl;
    exit(1);
  }
  mObjectRegistry.erase(uniqueId);
}

void OptifuserRenderer::updateRigidbody(uint32_t uniqueId, const physx::PxTransform &transform) {
  if (mObjectRegistry.find(uniqueId) == mObjectRegistry.end()) {
    std::cerr << "Object does not exist" << std::endl;
    exit(1);
  }
  for (auto obj : mObjectRegistry[uniqueId]) {
    obj->position = {transform.p.x, transform.p.y, transform.p.z};
    obj->rotation = {transform.q.w, transform.q.x, transform.q.y, transform.q.z};
  }
}

void OptifuserRenderer::init() {
  mContext = &Optifuser::GLFWRenderContext::Get(WINDOW_WIDTH, WINDOW_HEIGHT);
  mContext->initGui();
  mScene = std::make_shared<Optifuser::Scene>();

  cam.setUp({0, 0, 1});
  cam.setForward({0, 1, 0});
  cam.position = {-8, 0, 4};
  cam.rotateYawPitch(0, -0.15);
  cam.fovy = glm::radians(45.f);
  cam.aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;

  mContext->renderer.setShadowShader("glsl_shader/shadow.vsh", "glsl_shader/shadow.fsh");
  mContext->renderer.setGBufferShader("glsl_shader/gbuffer.vsh",
                                      "glsl_shader/gbuffer_segmentation.fsh");
  mContext->renderer.setDeferredShader("glsl_shader/deferred.vsh",
                                       "glsl_shader/deferred.fsh");
  mContext->renderer.setAxisShader("glsl_shader/axes.vsh", "glsl_shader/axes.fsh");
  mContext->renderer.enablePicking();
  mContext->renderer.enableAxisPass();
}

void OptifuserRenderer::destroy() {
  // TODO: check if scene needs special destroy handling
}

void OptifuserRenderer::render() {
  static float moveSpeed = 1.f;
  mContext->processEvents();

  float dt = 0.005f * moveSpeed;
  if (Optifuser::getInput().getKeyState(GLFW_KEY_W)) {
    cam.moveForwardRight(dt, 0);
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_S)) {
    cam.moveForwardRight(-dt, 0);
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_A)) {
    cam.moveForwardRight(0, -dt);
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_D)) {
    cam.moveForwardRight(0, dt);
  }
  cam.aspect =
      static_cast<float>(mContext->getWidth()) / static_cast<float>(mContext->getHeight());

  static bool renderGui = true;
  if (Optifuser::getInput().getKeyDown(GLFW_KEY_E)) {
    renderGui = !renderGui;
  }
  if (Optifuser::getInput().getMouseButton(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
    double dx, dy;
    Optifuser::getInput().getCursorDelta(dx, dy);
    cam.rotateYawPitch(-dx / 1000.f, -dy / 1000.f);
  }

  static int renderMode = 0;
  mContext->renderer.renderScene(*mScene, cam);
  if (renderMode == SEGMENTATION) {
    mContext->renderer.displaySegmentation();
  } else if (renderMode == CUSTOM) {
    mContext->renderer.displayUserTexture();
  } else {
    mContext->renderer.displayLighting();
  }

  static int pickedId = 0, pickedRenderId = 0;
  static GuiInfo pickedInfo;
  if (Optifuser::getInput().getMouseDown(GLFW_MOUSE_BUTTON_LEFT)) {
    int x, y;
    Optifuser::getInput().getCursor(x, y);
    pickedId = mContext->renderer.pickSegmentationId(x, y);
    if (pickedId) {
      pickedRenderId = mContext->renderer.pickObjectId(x, y);
    } else {
      pickedRenderId = 0;
    }
  }
  if (pickedId) {
    pickedInfo = queryCallback(pickedId);
    auto &pos = pickedInfo.linkInfo.transform.p;
    auto &quat = pickedInfo.linkInfo.transform.q;
    mScene->clearAxes();
    mScene->addAxes({pos.x, pos.y, pos.z}, {quat.w, quat.x, quat.y, quat.z});
  }

  static const uint32_t imguiWindowSize = 300;
  static int camIndex = -1;
  if (renderGui) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mContext->getHeight()));

    ImGui::Begin("Render Options");
    {
      if (ImGui::CollapsingHeader("Render Mode", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::RadioButton("Lighting", &renderMode, RenderMode::LIGHTING)) {
          mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                               "../glsl_shader/deferred.fsh");
        };
        if (ImGui::RadioButton("Albedo", &renderMode, RenderMode::ALBEDO)) {
          mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                               "../glsl_shader/deferred_albedo.fsh");
        }
        if (ImGui::RadioButton("Normal", &renderMode, RenderMode::NORMAL)) {
          mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                               "../glsl_shader/deferred_normal.fsh");
        }
        if (ImGui::RadioButton("Depth", &renderMode, RenderMode::DEPTH)) {
          mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                               "../glsl_shader/deferred_depth.fsh");
        }
        if (ImGui::RadioButton("Segmentation", &renderMode, RenderMode::SEGMENTATION)) {
          mContext->renderer.setGBufferShader("../glsl_shader/gbuffer.vsh",
                                              "../glsl_shader/gbuffer_segmentation.fsh");
        }
        if (ImGui::RadioButton("Custom", &renderMode, RenderMode::CUSTOM)) {
          mContext->renderer.setGBufferShader("../glsl_shader/gbuffer.vsh",
                                              "../glsl_shader/gbuffer_segmentation.fsh");
        }
      }

      if (ImGui::CollapsingHeader("Main Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Position");
        ImGui::Text("%-4.3f %-4.3f %-4.3f", cam.position.x, cam.position.y, cam.position.z);
        ImGui::Text("Forward");
        auto forward = cam.rotation * glm::vec3(0, 0, -1);
        ImGui::Text("%-4.3f %-4.3f %-4.3f", forward.x, forward.y, forward.z);
        ImGui::Text("Fov");
        ImGui::SliderAngle("##fov(y)", &cam.fovy, 1.f, 90.f);
        ImGui::Text("Move speed");
        ImGui::SliderFloat("##speed", &moveSpeed, 1.f, 100.f);
        ImGui::Text("Width: %d", mContext->getWidth());
        ImGui::SameLine();
        ImGui::Text("Height: %d", mContext->getHeight());
        ImGui::SameLine();
        ImGui::Text("Aspect: %.2f", cam.aspect);
        ImGui::Text("Picked link id: %d", pickedId);
        ImGui::Text("Picked render id: %d", pickedRenderId);
      }

      if (ImGui::CollapsingHeader("Mounted Cameras")) {
        ImGui::RadioButton("None##camera", &camIndex, -1);
        for (auto &cam : mMountedCameras) {
          ImGui::RadioButton(
              (mMountedCameras[cam.first]->getName() + "##camera" + std::to_string(cam.first))
                  .c_str(),
              &camIndex, cam.first);
        }

        if (camIndex >= 0) {
          uint32_t width = mMountedCameras[camIndex]->getWidth();
          uint32_t height = mMountedCameras[camIndex]->getHeight();
          mMountedCameras[camIndex]->takePicture();
          ImGui::Image(
              reinterpret_cast<ImTextureID>(
                  mMountedCameras[camIndex]->mRenderContext->renderer.outputtex),
              ImVec2(imguiWindowSize, imguiWindowSize / static_cast<float>(width) * height),
              ImVec2(0, 1), ImVec2(1, 0));
        }
      }

      if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Frame Time: %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
      }
    }
    ImGui::End();

    if (pickedId) {
      ImGui::SetNextWindowPos(ImVec2(mContext->getWidth() - imguiWindowSize, 0));
      ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mContext->getHeight()));
      ImGui::Begin("Selected Object");
      {
        if (ImGui::CollapsingHeader("Actor", ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text("name: %s", pickedInfo.linkInfo.name.c_str());
        }
        if (ImGui::CollapsingHeader("Articulation", ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text("name: %s", pickedInfo.articulationInfo.name.c_str());
          ImGui::Text(" dof: %ld", pickedInfo.articulationInfo.jointInfo.size());
          int i = 0;
          for (auto &jointInfo : pickedInfo.articulationInfo.jointInfo) {
            ImGui::Text("%s", jointInfo.name.c_str());
            if (ImGui::SliderFloat(("##" + std::to_string(++i)).c_str(), &jointInfo.value,
                                   jointInfo.limits[0], jointInfo.limits[1])) {
              syncCallback(pickedId, pickedInfo);
            }
          }
        }
      }
      ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }
  mContext->swapBuffers();
}

void OptifuserRenderer::bindQueryCallback(std::function<GuiInfo(uint32_t)> callback) {
  queryCallback = callback;
}

void OptifuserRenderer::bindSyncCallback(
    std::function<void(uint32_t, const GuiInfo &info)> callback) {
  syncCallback = callback;
}

std::vector<ICamera *> OptifuserRenderer::getCameras() {
  std::vector<ICamera *> output;
  for (auto &cam : mMountedCameras) {
    output.push_back(cam.second.get());
  }
  return output;
}

void OptifuserRenderer::addCamera(uint32_t uniqueId, std::string const &name, uint32_t width,
                                  uint32_t height, float fovx, float fovy, float near, float far) {
  std::cout
      << "Note: current camera implementation does not support non-square pixels, and fovy will "
         "take precedence."
      << std::endl;
  mMountedCameras[uniqueId] =
      std::make_unique<MountedCamera>(name, width, height, fovy, mScene.get());
  mMountedCameras[uniqueId]->near = near;
  mMountedCameras[uniqueId]->far = far;
}

void OptifuserRenderer::updateCamera(uint32_t uniqueId, physx::PxTransform const &transform) {
  assert(mMountedCameras.find(uniqueId) != mMountedCameras.end());
  mMountedCameras[uniqueId]->position = {transform.p.x, transform.p.y, transform.p.z};
  mMountedCameras[uniqueId]->rotation = {transform.q.w, transform.q.x, transform.q.y,
                                         transform.q.z};
}

void OptifuserRenderer::setAmbientLight(std::array<float, 3> color) {
  mScene->setAmbientLight({color[0], color[1], color[2]});
}

void OptifuserRenderer::setShadowLight(std::array<float, 3> direction,
                                       std::array<float, 3> color) {
  mScene->setShadowLight(
      {{direction[0], direction[1], direction[2]}, {color[0], color[1], color[2]}});
}
void OptifuserRenderer::addPointLight(std::array<float, 3> position, std::array<float, 3> color) {

  mScene->addPointLight({{position[0], position[1], position[2]}, {color[0], color[1], color[2]}});
}

void OptifuserRenderer::showWindow() { mContext->showWindow(); }

void OptifuserRenderer::hideWindow() { mContext->hideWindow(); }

} // namespace Renderer
} // namespace sapien
