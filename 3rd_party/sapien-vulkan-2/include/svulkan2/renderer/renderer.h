/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "renderer_base.h"
#include "svulkan2/core/context.h"
#include "svulkan2/core/descriptor_pool.h"
#include "svulkan2/resource/cubemap.h"
#include "svulkan2/resource/render_target.h"
#include "svulkan2/scene/scene.h"
#include "svulkan2/shader/shader_pack_instance.h"
#include <map>
#include <unordered_set>

namespace svulkan2 {
namespace core {
class Context;
class CommandPool;
} // namespace core
namespace renderer {

class Renderer : public RendererBase {
  std::shared_ptr<core::Context> mContext;
  std::shared_ptr<RendererConfig> mConfig;

  vk::UniqueDescriptorPool mDescriptorPool;
  std::unique_ptr<core::DynamicDescriptorPool> mObjectPool;

  // std::unique_ptr<shader::ShaderManager> mShaderManager;

  std::shared_ptr<shader::ShaderPack> mShaderPack;
  std::shared_ptr<shader::ShaderPackInstance> mShaderPackInstance;
  std::unordered_map<std::string, std::shared_ptr<resource::SVRenderTarget>> mRenderTargets;
  std::unordered_map<std::string, std::shared_ptr<resource::SVRenderTarget>> mMultisampledTargets;

  // shadow targets ================================
  std::vector<uint32_t> mPointLightShadowSizes{};
  std::vector<uint32_t> mDirectionalLightShadowSizes{};
  std::vector<uint32_t> mSpotLightShadowSizes{};
  std::vector<uint32_t> mTexturedLightShadowSizes{};

  std::vector<std::shared_ptr<resource::SVRenderTarget>> mDirectionalShadowReadTargets;
  std::vector<std::shared_ptr<resource::SVRenderTarget>> mDirectionalShadowWriteTargets;

  std::vector<std::shared_ptr<resource::SVRenderTarget>> mPointShadowWriteTargets;
  std::vector<std::shared_ptr<resource::SVRenderTarget>> mPointShadowReadTargets;

  std::vector<std::shared_ptr<resource::SVRenderTarget>> mSpotShadowReadTargets;
  std::vector<std::shared_ptr<resource::SVRenderTarget>> mSpotShadowWriteTargets;

  std::vector<std::shared_ptr<resource::SVRenderTarget>> mTexturedLightShadowReadTargets;
  std::vector<std::shared_ptr<resource::SVRenderTarget>> mTexturedLightShadowWriteTargets;

  std::vector<vk::UniqueDescriptorSet> mLightSets;
  std::vector<std::unique_ptr<core::Buffer>> mLightBuffers;

  std::unique_ptr<core::Buffer> mShadowBuffer;
  // ================================================

  std::unordered_map<std::string, vk::ImageLayout> mRenderTargetFinalLayouts;

  std::vector<vk::UniqueFramebuffer> mFramebuffers;
  std::vector<vk::UniqueFramebuffer> mShadowFramebuffers;
  std::vector<uint32_t> mShadowSizes;

  std::map<std::string, std::vector<std::shared_ptr<resource::SVTexture>>> mCustomTextureArray;
  std::map<std::string, std::shared_ptr<resource::SVTexture>> mCustomTextures;
  std::map<std::string, std::shared_ptr<resource::SVCubemap>> mCustomCubemaps;

  std::shared_ptr<resource::SVCubemap> mEnvironmentMap{};

  int mWidth{};
  int mHeight{};
  bool mRequiresRebuild{true};

  std::unique_ptr<core::Buffer> mSceneBuffer;

  std::unique_ptr<core::Buffer> mCameraBufferCpu;
  std::unique_ptr<core::Buffer> mCameraBuffer;

  std::unique_ptr<core::Buffer> mObjectDataBuffer;

  // transform buffer shared with all cameras
  std::shared_ptr<core::Buffer> mObjectTransformBuffer;

  vk::UniqueDescriptorSet mSceneSet;
  vk::UniqueDescriptorSet mCameraSet;
  std::vector<vk::UniqueDescriptorSet> mObjectSet;

  std::vector<vk::UniqueDescriptorSet> mInputTextureSets;

  bool mSpecializationConstantsChanged = true;
  std::map<std::string, SpecializationConstantValue> mSpecializationConstants;

  bool mRequiresRecord{false};
  uint64_t mLastVersion{0};
  std::shared_ptr<scene::Scene> mScene;

#ifdef SVULKAN2_CUDA_INTEROP
  std::map<std::string, std::shared_ptr<core::Buffer>> mTransferBuffers;
#endif
public:
  Renderer(std::shared_ptr<RendererConfig> config);

  template <typename T> void setSpecializationConstant(std::string const &name, T value) {
    if (!mContext->isVulkanAvailable()) {
      return;
    }

    if (!mSpecializationConstants.contains(name)) {
      mSpecializationConstantsChanged = true;
      mSpecializationConstants[name] = value;
      return;
    }

    SpecializationConstantValue newConstant;
    newConstant = value;
    if (mSpecializationConstants[name] != newConstant) {
      mSpecializationConstantsChanged = true;
      mSpecializationConstants[name] = value;
    }
  }

  void resize(int width, int height) override;

  void render(scene::Camera &camera,
              vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
              vk::ArrayProxyNoTemporaries<vk::PipelineStageFlags const> const &waitStageMasks,
              vk::ArrayProxyNoTemporaries<uint64_t const> const &waitValues,
              vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &signalSemaphores,
              vk::ArrayProxyNoTemporaries<uint64_t const> const &signalValues) override;

  /** render may be called on a thread. */
  void render(scene::Camera &camera, std::vector<vk::Semaphore> const &waitSemaphores,
              std::vector<vk::PipelineStageFlags> const &waitStages,
              std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) override;

  void display(std::string const &renderTargetName, vk::Image backBuffer, vk::Format format,
               uint32_t width, uint32_t height, std::vector<vk::Semaphore> const &waitSemaphores,
               std::vector<vk::PipelineStageFlags> const &waitStages,
               std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) override;

  void setScene(std::shared_ptr<scene::Scene> scene) override {
    mScene = scene;
    mRequiresRebuild = true;
    mRequiresRecord = true;
  }

  std::vector<std::string> getDisplayTargetNames() const override;
  std::vector<std::string> getRenderTargetNames() const override;

  std::shared_ptr<resource::SVRenderTarget> getRenderTarget(std::string const &name) const;

  core::Image &getRenderImage(std::string const &name) override {
    return getRenderTarget(name)->getImage();
  }

  vk::ImageLayout getRenderTargetImageLayout(std::string const &name) override {
    return mRenderTargetFinalLayouts.at(name);
  }

  void setCustomTextureArray(std::string const &name,
                             std::vector<std::shared_ptr<resource::SVTexture>> texture) override;
  void setCustomTexture(std::string const &name,
                        std::shared_ptr<resource::SVTexture> texture) override;
  void setCustomCubemap(std::string const &name,
                        std::shared_ptr<resource::SVCubemap> cubemap) override;

  int getCustomPropertyInt(std::string const &name) const override;
  float getCustomPropertyFloat(std::string const &name) const override;
  glm::vec3 getCustomPropertyVec3(std::string const &name) const override;
  glm::vec4 getCustomPropertyVec4(std::string const &name) const override;

  void setCustomProperty(std::string const &name, int p) override;
  void setCustomProperty(std::string const &name, float p) override;
  void setCustomProperty(std::string const &name, glm::vec3 p) override;
  void setCustomProperty(std::string const &name, glm::vec4 p) override;

  core::Buffer &getCameraBuffer();

  void setAutoUploadEnabled(bool enable);
  bool getAutoUploadEnabled() { return mAutoUpload; }

  void forceUploadCameraBuffer(scene::Camera &camera);

  Renderer(Renderer const &other) = delete;
  Renderer &operator=(Renderer const &other) = delete;
  Renderer(Renderer &&other) = default;
  Renderer &operator=(Renderer &&other) = default;

private:
  void prepareRenderTargets(uint32_t width, uint32_t height);
  void prepareShadowRenderTargets();
  void preparePipelines();
  void prepareFramebuffers(uint32_t width, uint32_t height);
  void prepareShadowFramebuffers();

  void prepareSceneBuffer();
  void prepareObjectBuffers(uint32_t numObjects);
  void prepareCameraBuffer();
  void prepareLightBuffers();

  void prepareInputTextureDescriptorSets();

  std::unordered_set<std::shared_ptr<resource::SVModel>> mModelCache;
  std::unordered_set<std::shared_ptr<resource::SVLineSet>> mLineSetCache;
  std::unordered_set<std::shared_ptr<resource::SVPointSet>> mPointSetCache;

  std::unique_ptr<core::CommandPool> mUploadCommandPool{};
  std::unique_ptr<core::CommandPool> mShadowCommandPool{};
  std::unique_ptr<core::CommandPool> mRenderCommandPool{};
  std::unique_ptr<core::CommandPool> mDisplayCommandPool{};

  vk::UniqueCommandBuffer mUploadCommandBuffer{};
  vk::UniqueCommandBuffer mShadowCommandBuffer{};
  vk::UniqueCommandBuffer mRenderCommandBuffer{};
  vk::UniqueCommandBuffer mDisplayCommandBuffer{};

  void prepareObjects();

  // record data upload commands (e.g. camera buffer upload)
  void recordUpload();

  void recordShadows();
  void recordRenderPasses();

  uint32_t mLineObjectIndex{};  // starting object index for line objects
  uint32_t mPointObjectIndex{}; // starting object index for point objects

  bool mAutoUpload{true};

  // load and uplaod required resources
  void prepareRender(scene::Camera &camera);

  // upload resources to gpu, when mAutoUpload is true, it is called immediately after
  // prepareRender
  void uploadGpuResources(scene::Camera &camera);
};

} // namespace renderer
} // namespace svulkan2