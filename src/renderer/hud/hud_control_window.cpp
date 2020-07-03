#include "hud_control_window.h"
#include "hud_common.h"

namespace sapien {
namespace Renderer {

void HudControl::draw() {
  mStepped = false;
  if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Pause", &mPause);
    if (mPause && ImGui::Button("Single step")) {
      mStepped = true;
    }
    ImGui::Checkbox("Flip X", &mInvertX);
    ImGui::Checkbox("Flip Y", &mInvertY);
    ImGui::SliderFloat("Move speed", &mMoveSpeed, 0.01, 5);
    ImGui::SliderFloat("Rotate speed", &mRotateSpeed, 0.01, 5);
  }
}

void HudRenderMode::draw() {
  mSwitchMode = false;
  if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::RadioButton("Lighting", &mMode, Mode::eLighting)) {
      mSwitchMode = true;
    }
    if (ImGui::RadioButton("Normal", &mMode, Mode::eNormal)) {
      mSwitchMode = true;
    }
    if (ImGui::RadioButton("Depth", &mMode, Mode::eDepth)) {
      mSwitchMode = true;
    }
    if (ImGui::RadioButton("Segmentation", &mMode, Mode::eSegmentation)) {
      mSwitchMode = true;
    }
  }
}

void HudCameraInfo::draw() {
  mUpdate = false;
  if (ImGui::CollapsingHeader("View Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
    PrintPose(mPosition, mRotation);
    ImGui::Text("Size: %d x %d", mWidth, mHeight);
    ImGui::Text("Range: %.4g - %.4g", mNear, mFar);
    if (ImGui::SliderAngle("Fovy (full angle)", &mFovy, 5, 175)) {
      mUpdate = true;
    }
    if (mTextInfo.length()) {
      ImGui::Text("%s", mTextInfo.c_str());
    }
  }
}

void HudStats::draw() {
  mFrameRate = ImGui::GetIO().Framerate;
  if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("FPS: %.1f", mFrameRate);
    ImGui::Text("Frame time: %.3f ms", 1000.f / mFrameRate);
  }
}
  
void HudControlWindow::draw(){
  ImGui::Begin("Control");
  mHudControl.draw();
  mHudRenderMode.draw();
  mHudCameraInfo.draw();
  mHudStats.draw();
  ImGui::End();
}

}
}
