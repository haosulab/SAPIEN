#include <imgui.h>

namespace sapien {
namespace Renderer {

struct HudControl {
  bool mPause {};
  bool mInvertX {};
  bool mInvertY {};

  float mRotateSpeed {1.f};
  float mMoveSpeed {1.f};

  bool mStepped {};

  void draw() {
    mStepped = false;
    if (ImGui::CollapsingHeader("Control")) {
      if (ImGui::Checkbox("Pause", &mPause)) {
        if (ImGui::Button("Single step")) {
          mStepped = true;
        }
      }
      ImGui::Checkbox("Flip X", &mInvertX);
      ImGui::Checkbox("Flip Y", &mInvertY);
      ImGui::SliderFloat("Move speed", &mMoveSpeed, 0.01, 5);
      ImGui::SliderFloat("Rotate speed", &mRotateSpeed, 0.01, 5);
    }
  }
};

struct HudRenderMode {
  enum Mode {
    eLighting=0, eAlbedo=1, eNormal=2, eDepth=3
  };
  int mMode = Mode::eLighting;

  void draw() {
    if (ImGui::CollapsingHeader("Display")) {
      ImGui::RadioButton("Lighting", &mMode, Mode::eLighting);
      ImGui::RadioButton("Albedo", &mMode, Mode::eAlbedo);
      ImGui::RadioButton("Normal", &mMode, Mode::eNormal);
      ImGui::RadioButton("Depth", &mMode, Mode::eDepth);
    }
  }
};

struct HudStats {
  float mFrameRate{};
  void draw() {
    if (ImGui::CollapsingHeader("Stats")) {
      mFrameRate = ImGui::GetIO().Framerate;
      ImGui::Text("FPS: %.1f", mFrameRate);
      ImGui::Text("Frame time: %.3f ms", 1000.f / mFrameRate);
    }
  }
};
  
struct HudControlWindow {
  HudControl mHudControl = {};
  HudRenderMode mHudRenderMode = {};
  HudStats mHudStats = {};

  void draw() {
    ImGui::Begin("Control");
    mHudControl.draw();
    mHudRenderMode.draw();
    mHudStats.draw();
    ImGui::End();
  }
};

}
}
