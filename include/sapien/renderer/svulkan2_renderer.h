#pragma once
#include "render_interface.h"
#include "svulkan2_light.h"
#include "svulkan2_material.h"
#include "svulkan2_scene.h"
#include <memory>
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/renderer/rt_renderer.h>
#include <svulkan2/scene/scene.h>

namespace sapien {
namespace Renderer {
class SVulkan2Renderer;
class SVulkan2Rigidbody;
class SVulkan2Camera;
class SVulkan2Scene;

template <typename Res> class AwaitableSemaphore : public IAwaitable<Res> {
  std::function<Res()> mCallback;
  vk::Semaphore mSemaphore;
  uint64_t mValue;
  vk::Device mDevice;

public:
  AwaitableSemaphore(std::function<Res()> callback, vk::Semaphore sem, uint64_t value,
                     vk::Device device)
      : mCallback(callback), mSemaphore(sem), mValue(value), mDevice(device) {}
  Res wait() override {
    if (mDevice.waitSemaphores(vk::SemaphoreWaitInfo({}, mSemaphore, mValue), UINT64_MAX) !=
        vk::Result::eSuccess) {
      throw std::runtime_error("failed to wait for semaphore");
    }
    return mCallback();
  }
  bool ready() override {
    return mDevice.waitSemaphores(vk::SemaphoreWaitInfo({}, mSemaphore, mValue), 0) ==
           vk::Result::eSuccess;
  }
};

class SVulkan2Renderer : public IRenderer {
public:
  static void setLogLevel(std::string const &level);

  SVulkan2Renderer(bool offscreenOnly, uint32_t maxNumMaterials, uint32_t maxNumTextures,
                   uint32_t defaultMipLevels, std::string const &device,
                   std::string const &culling, bool doNotLoadTexture);
  IRenderScene *createScene(std::string const &name) override;
  void removeScene(IRenderScene *scene) override;

  std::shared_ptr<IRenderMaterial> createMaterial() override;
  std::shared_ptr<IRenderTexture>
  createTexture(std::string_view filename, uint32_t mipLevels = 1,
                IRenderTexture::FilterMode::Enum filterMode = {},
                IRenderTexture::AddressMode::Enum addressMode = {}) override;

  std::shared_ptr<IRenderTexture> createTexture(std::vector<uint8_t> const &data, int width,
                                             int height, uint32_t mipLevels = 1,
                                             IRenderTexture::FilterMode::Enum filterMode = {},
                                             IRenderTexture::AddressMode::Enum addressMode = {},
                                             bool srgb = true) override;

  std::shared_ptr<IRenderTexture>
  createTexture(std::vector<float> const &data, int width, int height, int depth, int dim,
                uint32_t mipLevels = 1, IRenderTexture::FilterMode::Enum filterMode = {},
                IRenderTexture::AddressMode::Enum addressMode = {}) override;

  std::shared_ptr<IRenderMesh> createMesh(std::vector<float> const &vertices,
                                          std::vector<uint32_t> const &indices) override;

  vk::CullModeFlags getCullMode() const;
  inline std::shared_ptr<svulkan2::core::Context> getContext() const { return mContext; };
  inline std::shared_ptr<svulkan2::resource::SVResourceManager> getResourceManager() const {
    return mResourceManager;
  };

  inline std::shared_ptr<svulkan2::RendererConfig> getDefaultRendererConfig() const {
    return mDefaultRendererConfig;
  }

private:
  std::shared_ptr<svulkan2::core::Context> mContext{};
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager{};
  std::vector<std::unique_ptr<SVulkan2Scene>> mScenes;

  std::shared_ptr<svulkan2::RendererConfig> mDefaultRendererConfig{};
};

class SVulkan2Camera : public ICamera {
  uint32_t mWidth, mHeight;
  SVulkan2Scene *mScene;
  std::unique_ptr<svulkan2::renderer::RendererBase> mRenderer;
  svulkan2::scene::Camera *mCamera;
  vk::UniqueSemaphore mSemaphore;
  uint64_t mFrameCounter{0};
  std::unordered_map<std::string, std::shared_ptr<svulkan2::core::Buffer>> mImageBuffers;
  std::unique_ptr<svulkan2::core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBuffer;

  void waitForRender();

public:
  inline uint32_t getWidth() const override { return mWidth; };
  inline uint32_t getHeight() const override { return mHeight; };

  [[nodiscard]] float getPrincipalPointX() const override;
  [[nodiscard]] float getPrincipalPointY() const override;
  [[nodiscard]] float getFocalX() const override;
  [[nodiscard]] float getFocalY() const override;
  [[nodiscard]] float getNear() const override;
  [[nodiscard]] float getFar() const override;
  [[nodiscard]] float getSkew() const override;

  void setPerspectiveCameraParameters(float near, float far, float fx, float fy, float cx,
                                      float cy, float skew) override;

  void setIntProperty(std::string const &name, int property) override;
  void setFloatProperty(std::string const &name, float property) override;
  void setCustomTexture(std::string const &name, std::shared_ptr<IRenderTexture> texture) override;
  void setCustomTextureArray(std::string const &name,
                             std::vector<std::shared_ptr<IRenderTexture>> textures) override;

  SVulkan2Camera(uint32_t width, uint32_t height, float fovy, float near, float far,
                 SVulkan2Scene *scene, std::string const &shaderDir);

  void takePicture() override;

#ifdef SAPIEN_DLPACK
  std::shared_ptr<IAwaitable<std::vector<DLManagedTensor *>>>
  takePictureAndGetDLTensorsAsync(ThreadPool &thread,
                                  std::vector<std::string> const &names) override;
#endif

  std::vector<std::string> getRenderTargetNames();

  std::vector<float> getFloatImage(std::string const &name) override;
  std::vector<uint32_t> getUintImage(std::string const &name) override;
  std::vector<uint8_t> getUint8Image(std::string const &name) override;

  std::string getImageFormat(std::string const &name) override;

#ifdef SAPIEN_DLPACK
  DLManagedTensor *getDLImage(std::string const &name) override;
#endif

  inline IRenderScene *getScene() override { return mScene; }

  glm::mat4 getModelMatrix() const;
  glm::mat4 getProjectionMatrix() const;
  glm::mat4 getCameraMatrix() const;

  // ISensor
  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &pose) override;

  void setOrthographicParameters(float near, float far, float scaling, float width, float height);

  svulkan2::scene::Camera *getCamera() const { return mCamera; }
  std::string getMode() const;

  inline svulkan2::renderer::RendererBase *getInternalRenderer() const { return mRenderer.get(); }
};

} // namespace Renderer
} // namespace sapien
