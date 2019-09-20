#include "optifuser_renderer.h"
#include <objectLoader.h>

#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

enum RenderMode { LIGHTING, ALBEDO, NORMAL, DEPTH, SEGMENTATION };

constexpr int WINDOW_WIDTH = 1200, WINDOW_HEIGHT = 800;

void OptifuserRenderer::addRigidbody(uint64_t uniqueId, const std::string &objFile,
                                     const physx::PxVec3 &scale) {
  if (mObjectRegistry.find(uniqueId) != mObjectRegistry.end()) {
    std::cerr << "Object already added" << std::endl;
    exit(1);
  }
  auto objects = Optifuser::LoadObj(objFile);
  for (auto &obj : objects) {
    mObjectRegistry[uniqueId].push_back(obj.get());
    obj->scale = {scale.x, scale.y, scale.z};
    mScene->addObject(std::move(obj));
  }

#ifdef _DEBUG
  printf("Adding Object %ld from %s\n", uniqueId, objFile.c_str());
#endif
}

void OptifuserRenderer::addRigidbody(uint64_t uniqueId, physx::PxGeometryType::Enum type,
                                     const physx::PxVec3 &scale) {
  if (mObjectRegistry.find(uniqueId) != mObjectRegistry.end()) {
    std::cerr << "Object already added" << std::endl;
    exit(1);
  }

  switch (type) {
  case physx::PxGeometryType::eBOX: {
    auto obj = Optifuser::NewCube();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {1, 1, 1};
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  case physx::PxGeometryType::eSPHERE: {
    auto obj = Optifuser::NewSphere();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {1, 1, 1};
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  case physx::PxGeometryType::ePLANE: {
    auto obj = Optifuser::NewYZPlane();
    obj->scale = {scale.x, scale.y, scale.z};
    obj->material.kd = {1, 1, 1};
    mObjectRegistry[uniqueId] = {obj.get()};
    mScene->addObject(std::move(obj));
    break;
  }
  default:
    std::cerr << "This shape is not Implemented" << std::endl;
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

void OptifuserRenderer::updateRigidbody(uint64_t uniqueId, const physx::PxTransform &transform) {
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

  cam.up = {0, 0, 1};
  cam.forward = {0, 1, 0};
  cam.position = {-8, 0, 4};
  cam.rotateYawPitch(0, -0.15);
  cam.fovy = glm::radians(45.f);
  cam.aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;
  mScene->addDirectionalLight({{0, -1, -1}, {1, 1, 1}});
  mScene->setAmbientLight(glm::vec3(0.1, 0.1, 0.1));

  mScene->setEnvironmentMap(
      "../assets/ame_desert/desertsky_ft.tga", "../assets/ame_desert/desertsky_bk.tga",
      "../assets/ame_desert/desertsky_up.tga", "../assets/ame_desert/desertsky_dn.tga",
      "../assets/ame_desert/desertsky_lf.tga", "../assets/ame_desert/desertsky_rt.tga");

  mContext->renderer.setShadowShader("../glsl_shader/shadow.vsh", "../glsl_shader/shadow.fsh");
  mContext->renderer.setGBufferShader("../glsl_shader/gbuffer.vsh", "../glsl_shader/gbuffer.fsh");
  mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                       "../glsl_shader/deferred.fsh");
  mContext->renderer.setAxisShader("../glsl_shader/axes.vsh", "../glsl_shader/axes.fsh");

  mContext->renderer.worldAxesScale = 3.f;
  mContext->renderer.objectAxesScale = 0.4f;
}

void OptifuserRenderer::destroy() {
  // TODO: check if scene needs special destroy handling
}

void OptifuserRenderer::render() {
  mContext->processEvents();

  float dt = 0.05f;
  if (Optifuser::getInput().getKeyState(GLFW_KEY_W)) {
    cam.moveForwardRight(dt, 0);
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_S)) {
    cam.moveForwardRight(-dt, 0);
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_A)) {
    cam.moveForwardRight(0, -dt);
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_D)) {
    cam.moveForwardRight(0, dt);
  }
  static bool renderGui = true;
  if (Optifuser::getInput().getMouseButton(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
    double dx, dy;
    Optifuser::getInput().getCursorDelta(dx, dy);
    cam.rotateYawPitch(-dx / 1000.f, -dy / 1000.f);
  }

  mContext->renderer.renderScene(*mScene, cam);
  mContext->renderer.displayLighting(0);

  static int renderMode = 0;
  if (renderGui) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

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
        ImGui::RadioButton("Segmentation", &renderMode, RenderMode::SEGMENTATION);
      }

      if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Position");
        ImGui::Text("%-4.3f %-4.3f %-4.3f", cam.position.x, cam.position.y, cam.position.z);
        ImGui::Text("Forward");
        auto forward = cam.rotation * glm::vec3(0, 0, -1);
        ImGui::Text("%-4.3f %-4.3f %-4.3f", forward.x, forward.y, forward.z);
      }

      if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Frame Time: %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
      }
    }

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }
  mContext->swapBuffers();
}
