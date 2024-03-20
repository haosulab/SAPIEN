#pragma once
#include "image.h"
#include "material.h"
#include "sapien/math/mat.h"
#include "sapien/scene.h"
#include <functional>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/renderer/rt_renderer.h>
#include <svulkan2/ui/ui.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderEngine;

#ifdef _DEBUG_VIEWER
class FPSCameraControllerDebug {
  svulkan2::scene::Node *pCamera;
  glm::vec3 mRPY{0, 0, 0};
  glm::vec3 mXYZ{0, 0, 0};

  glm::vec3 mForward;
  glm::vec3 mUp;
  glm::vec3 mLeft;

  glm::quat mInitialRotation;

  void update();

public:
  FPSCameraControllerDebug(svulkan2::scene::Node &node, glm::vec3 const &forward,
                           glm::vec3 const &up);

  void setRPY(float roll, float pitch, float yaw);
  void setXYZ(float x, float y, float z);

  inline glm::vec3 getRPY() const { return mRPY; };
  inline glm::vec3 getXYZ() const { return mXYZ; };

  void move(float forward, float left, float up);
  void rotate(float roll, float pitch, float yaw);
};
#endif

class SapienRendererWindow {
public:
#ifdef _DEBUG_VIEWER
  std::unique_ptr<FPSCameraControllerDebug> mCameraController{};
#endif

  SapienRendererWindow(int width, int height, std::string const &shaderDir);
  ~SapienRendererWindow();

  void setScene(std::shared_ptr<Scene> scene);
  void setScenes(std::vector<std::shared_ptr<Scene>> const &scenes,
                 std::vector<Vec3> const &offsets);

  /** updateRender calls updateRender of individual scenes */
  void updateRender();

  void setCameraParameters(float near, float far, float fovy);
  void setCameraIntrinsicParameters(float near, float far, float fx, float fy, float cx, float cy,
                                    float skew);

  void setCameraPose(Pose const &pose);
  Pose getCameraPose();

  void setCameraPosition(Vec3 const &pos);
  void setCameraRotation(Quat const &rot);

  Vec3 getCameraPosition();
  Quat getCameraRotation();

  int getCameraPropertyInt(std::string const &name) const;
  float getCameraPropertyFloat(std::string const &name) const;

  void setCameraProperty(std::string const &name, float property);
  void setCameraProperty(std::string const &name, int property);
  void setCameraTexture(std::string const &name, std::shared_ptr<SapienRenderTexture2D> texture);
  void setCameraTextureArray(std::string const &name,
                             std::vector<std::shared_ptr<SapienRenderTexture2D>> textures);

  float getContentScale();

  float getCameraNear();
  float getCameraFar();
  float getCameraFovy();

  glm::mat4 getCameraProjectionMatrix();
  Mat4 getCameraModelMatrix();

  std::vector<std::string> getDisplayTargetNames() const;
  void render(std::string const &targetName,
              std::vector<std::shared_ptr<svulkan2::ui::Widget>> uiWindows = {});
  void show();
  void hide();

  void resize(int width, int height);
  void rebuild();

  void close();

  bool windowCloseRequested();

  SapienRenderImageCpu getImage(std::string const &name);
  CpuArray getImagePixel(std::string const &name, uint32_t x, uint32_t y);

  std::array<uint32_t, 2> getRenderTargetSize(std::string const &name) const;

  bool isShiftDown();
  bool isCtrlDown();
  bool isAltDown();
  bool isSuperDown();

  bool isKeyDown(std::string const &key);
  bool isKeyPressed(std::string const &key);
  bool isMouseKeyDown(int key);
  bool isMouseKeyClicked(int key);
  std::array<float, 2> getMousePosition();
  std::array<float, 2> getMouseDelta();
  std::array<float, 2> getMouseWheelDelta();

  std::array<int, 2> getWindowSize();

  void setCursorEnabled(bool enabled);
  bool getCursorEnabled() const;

  float getFPS();

  inline svulkan2::renderer::RendererBase *getInternalRenderer() const {
    return mSVulkanRenderer.get();
  }
  inline svulkan2::scene::Scene *getInternalScene() const { return mRenderScene.get(); }

  void setShader(std::string const &shaderDir);

  void setDropCallback(std::function<void(std::vector<std::string>)> callback);
  void unsetDropCallback();

  void setFocusCallback(std::function<void(int)> callback);
  void unsetFocusCallback();

  svulkan2::renderer::RTRenderer::DenoiserType getDenoiserType() const;
  void setDenoiserType(svulkan2::renderer::RTRenderer::DenoiserType type);

private:
  svulkan2::scene::Camera *getCamera();

  std::shared_ptr<SapienRenderEngine> mEngine;

  std::vector<std::shared_ptr<SapienRendererSystem>> mRenderSystems;
  std::shared_ptr<svulkan2::scene::Scene> mRenderScene;

  std::string mShaderDir{};
  std::unique_ptr<svulkan2::renderer::RendererBase> mSVulkanRenderer{};

  std::unique_ptr<svulkan2::renderer::GuiWindow> mWindow;

  std::unordered_map<std::string, std::vector<char>> mImageBuffers;

  vk::UniqueSemaphore mSceneRenderSemaphore;
  vk::UniqueFence mSceneRenderFence;
  int mViewportWidth{};
  int mViewportHeight{};
  bool mRequiresRebuild{true};
  bool mClosed{};
};

} // namespace sapien_renderer
} // namespace sapien
