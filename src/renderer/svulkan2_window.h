#pragma once

#include "svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

#ifdef _DEBUG
class FPSCameraController {
  svulkan2::scene::Node *mCamera;
  glm::vec3 mRPY{0, 0, 0};
  glm::vec3 mXYZ{0, 0, 0};

  glm::vec3 mForward;
  glm::vec3 mUp;
  glm::vec3 mLeft;

  glm::quat mInitialRotation;

  void update();

public:
  FPSCameraController(svulkan2::scene::Node &node, glm::vec3 const &forward, glm::vec3 const &up);

  void setRPY(float roll, float pitch, float yaw);
  void setXYZ(float x, float y, float z);

  inline glm::vec3 getRPY() const { return mRPY; };
  inline glm::vec3 getXYZ() const { return mXYZ; };

  void move(float forward, float left, float up);
  void rotate(float roll, float pitch, float yaw);
};
#endif

class SVulkan2Window {
  SVulkan2Renderer *mRenderer{};
  std::unique_ptr<svulkan2::renderer::Renderer> mSVulkanRenderer{};
  SVulkan2Scene *mScene{};

  std::unique_ptr<svulkan2::renderer::GuiWindow> mWindow;

  vk::UniqueSemaphore mSceneRenderSemaphore;
  vk::UniqueFence mSceneRenderFence;
  vk::UniqueCommandBuffer mCommandBuffer;

  bool mClosed{};

#ifdef _DEBUG
  std::unique_ptr<FPSCameraController> mCameraController{};
#endif
public:
  explicit SVulkan2Window(SVulkan2Renderer &renderer);
  ~SVulkan2Window();
  void setScene(SVulkan2Scene *scene);

  void render();
  void show();
  void hide();

  void close();

  bool windowCloseRequested();
};

} // namespace Renderer
} // namespace sapien
