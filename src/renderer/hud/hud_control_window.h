#pragma once
#include "hud_common.h"
#include <imgui.h>
#include <string>

namespace sapien {
namespace Renderer {

struct HudControl {
  bool mPause{};
  bool mInvertX{};
  bool mInvertY{};

  float mRotateSpeed{3.f};
  float mMoveSpeed{3.f};

  bool mStepped{};

  void draw();
};

struct HudRenderMode {
  enum Mode { eLighting = 0, eNormal = 1, eDepth = 2, eSegmentation = 3, eCustom = 4 };
  int mMode = Mode::eLighting;
  bool mSwitchMode = false;

  void draw();
};

struct HudCameraInfo {
  glm::vec3 mPosition;
  glm::quat mRotation;
  float mNear;
  float mFar;
  float mFovy;
  uint32_t mWidth;
  uint32_t mHeight;
  std::string mTextInfo;
  bool mUpdate;

  void draw();
};

struct HudStats {
  float mFrameRate{};
  void draw();
};

struct HudControlWindow {
  HudControl mHudControl{};
  HudRenderMode mHudRenderMode{};
  HudCameraInfo mHudCameraInfo{};
  HudStats mHudStats{};

  void draw();
};

} // namespace Renderer
} // namespace sapien
