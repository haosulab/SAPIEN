//
// Created by sim on 10/19/19.
//

#include "demonstration_gui.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using namespace sapien::Renderer;

enum RenderMode { LIGHTING, ALBEDO, NORMAL, DEPTH, SEGMENTATION, CUSTOM };

constexpr int WINDOW_WIDTH = 1200, WINDOW_HEIGHT = 800;
sapien::robot::DemonstrationGUI::DemonstrationGUI() : OptifuserRenderer() {
  mContext->renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                       "../glsl_shader/deferred.fsh");
}
void sapien::robot::DemonstrationGUI::render() {
  static float moveSpeed = 1.f;
  mContext->processEvents();

  float dt = 0.02 * moveSpeed;
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

  mContext->renderer.renderScene(*mScene, cam);
  mContext->renderer.displayLighting();

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
  static const uint32_t annotationWindowHeight = 300;
  uint32_t childWindowSize = (mContext->getWidth() - imguiWindowSize * 2) / 2;
  static int camIndex = -1;
  if (renderGui) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mContext->getHeight()));

    ImGui::Begin("Render Options");
    {
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

    ImGui::SetNextWindowPos(
        ImVec2(imguiWindowSize, mContext->getHeight() - annotationWindowHeight));
    ImGui::SetNextWindowSize(ImVec2(childWindowSize, annotationWindowHeight));
    ImGui::Begin("Collected Manipulation");
    {
      if (ImGui::CollapsingHeader("Saved Manipulation")) {
        for(size_t i=0;i<manipulations.size();++i){
//          ImGui::Text("Sequence%d: %s", i, );
        }
      }
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }
  mContext->swapBuffers();
}
physx::PxRigidActor *sapien::robot::DemonstrationGUI::getActorTupleFromSegId(uint32_t segID) {
  if (simulation->mLinkId2Actor.find(segID) == simulation->mLinkId2Actor.end()) {
    return nullptr;
  }
  auto actor = simulation->mLinkId2Actor[segID];
  return actor;
}
