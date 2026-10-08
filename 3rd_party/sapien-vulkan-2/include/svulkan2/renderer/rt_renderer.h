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
#include "svulkan2/resource/storage_image.h"
#include "svulkan2/scene/scene.h"
#include <variant>

namespace svulkan2 {
namespace core {
class Context;
class CommandPool;
} // namespace core

namespace shader {
class RayTracingShaderPack;
class RayTracingShaderPackInstance;
}; // namespace shader

namespace renderer {

class RTRenderer : public RendererBase {
public:
  enum class DenoiserType { eNONE, eOPTIX, eOIDN };

  RTRenderer(std::string const &shaderDir);

  void resize(int width, int height) override;
  void setScene(std::shared_ptr<scene::Scene> scene) override;

  void render(scene::Camera &camera, std::vector<vk::Semaphore> const &waitSemaphores,
              std::vector<vk::PipelineStageFlags> const &waitStages,
              std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) override;
  void render(scene::Camera &camera,
              vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
              vk::ArrayProxyNoTemporaries<vk::PipelineStageFlags const> const &waitStageMasks,
              vk::ArrayProxyNoTemporaries<uint64_t const> const &waitValues,
              vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &signalSemaphores,
              vk::ArrayProxyNoTemporaries<uint64_t const> const &signalValues) override;
  void display(std::string const &imageName, vk::Image backBuffer, vk::Format format,
               uint32_t width, uint32_t height, std::vector<vk::Semaphore> const &waitSemaphores,
               std::vector<vk::PipelineStageFlags> const &waitStages,
               std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) override;

  void enableDenoiser(DenoiserType type, std::string const &colorName,
                      std::string const &albedoName, std::string const &normalName);
  DenoiserType getDenoiserType() const;

  bool denoiserEnabled() const {
#ifdef SVULKAN2_CUDA_INTEROP
    return mDenoiser != nullptr;
#else
    return false;
#endif
  }
  void disableDenoiser();

  int getCustomPropertyInt(std::string const &name) const override {
    return mCustomPropertiesInt.at(name);
  }

  float getCustomPropertyFloat(std::string const &name) const override {
    return mCustomPropertiesFloat.at(name);
  }

  glm::vec3 getCustomPropertyVec3(std::string const &name) const override {
    return mCustomPropertiesVec3.at(name);
  }

  glm::vec4 getCustomPropertyVec4(std::string const &name) const override {
    return mCustomPropertiesVec4.at(name);
  }

  void setCustomProperty(std::string const &name, int p) override {
    mCustomPropertiesInt[name] = p;
  }
  void setCustomProperty(std::string const &name, float p) override {
    mCustomPropertiesFloat[name] = p;
  }
  void setCustomProperty(std::string const &name, glm::vec3 p) override {
    mCustomPropertiesVec3[name] = p;
  }
  void setCustomProperty(std::string const &name, glm::vec4 p) override {
    mCustomPropertiesVec4[name] = p;
  }

  std::vector<std::string> getDisplayTargetNames() const override;
  std::vector<std::string> getRenderTargetNames() const override;

  core::Image &getRenderImage(std::string const &name) override {
    return mRenderImages.at(name)->getImage();
  }

  vk::ImageLayout getRenderTargetImageLayout(std::string const &name) override {
    return vk::ImageLayout::eGeneral;
  }

  inline void setCustomTexture(std::string const &name,
                               std::shared_ptr<resource::SVTexture> texture) override {
    mCustomTextures[name] = texture;
  };
  inline void setCustomCubemap(std::string const &name,
                               std::shared_ptr<resource::SVCubemap> cubemap) override {
    mCustomCubemaps[name] = cubemap;
  };

  RTRenderer(RTRenderer const &other) = delete;
  RTRenderer &operator=(RTRenderer const &other) = delete;
  RTRenderer(RTRenderer &&other) = default;
  RTRenderer &operator=(RTRenderer &&other) = default;

  ~RTRenderer();

private:
  std::unordered_map<std::string, int> mCustomPropertiesInt;
  std::unordered_map<std::string, float> mCustomPropertiesFloat;
  std::unordered_map<std::string, glm::vec3> mCustomPropertiesVec3;
  std::unordered_map<std::string, glm::vec4> mCustomPropertiesVec4;

  void prepareObjects();
  void updateObjects();

  void prepareOutput();
  void prepareScene();
  void prepareCamera();
  void preparePostprocessing();

  void recordRender();
  void recordPostprocess();

  void updatePushConstant();

  void prepareRender(scene::Camera &camera);

  std::shared_ptr<core::Context> mContext;
  std::string mShaderDir;
  std::shared_ptr<shader::RayTracingShaderPack> mShaderPack;
  std::shared_ptr<shader::RayTracingShaderPackInstance> mShaderPackInstance;

  StructDataLayout mMaterialBufferLayout;
  StructDataLayout mTextureIndexBufferLayout;
  StructDataLayout mGeometryInstanceBufferLayout;
  StructDataLayout mCameraBufferLayout;
  StructDataLayout mObjectBufferLayout;

  int mWidth{};
  int mHeight{};

  std::shared_ptr<scene::Scene> mScene;
  uint64_t mSceneVersion{0l}; // check for rebuild

  uint64_t mSceneRenderVersion{0l}; // check for updating matrices
  int mFrameCount = 0;

  std::unordered_map<std::string, std::shared_ptr<resource::SVStorageImage>> mRenderImages;
  // subset of render images used in RT
  std::vector<std::shared_ptr<resource::SVStorageImage>> mRTImages;
  // subset of render images used in postprocess
  std::vector<std::shared_ptr<resource::SVStorageImage>> mPostprocessImages;

  std::unique_ptr<core::DynamicDescriptorPool> mOutputPool;
  std::unique_ptr<core::DynamicDescriptorPool> mCameraPool;
  std::unique_ptr<core::DynamicDescriptorPool> mScenePool;

  vk::UniqueDescriptorSet mSceneSet;
  vk::UniqueDescriptorSet mOutputSet;
  vk::UniqueDescriptorSet mCameraSet;

  std::unique_ptr<core::DynamicDescriptorPool> mPostprocessingPool;
  std::vector<vk::UniqueDescriptorSet> mPostprocessingSets;

  std::unique_ptr<core::Buffer> mCameraBuffer;
  std::unique_ptr<core::Buffer> mObjectBuffer;

  std::map<std::string, std::shared_ptr<resource::SVTexture>> mCustomTextures;
  std::map<std::string, std::shared_ptr<resource::SVCubemap>> mCustomCubemaps;
  std::shared_ptr<resource::SVCubemap> mEnvironmentMap{};

  std::unique_ptr<core::CommandPool> mRenderCommandPool;
  vk::UniqueCommandBuffer mRenderCommandBuffer;
  vk::UniqueCommandBuffer mPostprocessCommandBuffer;

  std::unique_ptr<core::CommandPool> mDisplayCommandPool;
  vk::UniqueCommandBuffer mDisplayCommandBuffer;

  std::vector<uint8_t> mPushConstantBuffer;

  bool mRequiresRebuild{true};

  vk::UniqueFence mSceneAccessFence;

#ifdef SVULKAN2_CUDA_INTEROP
  std::unique_ptr<class Denoiser> mDenoiser;
  std::string mDenoiseColorName;
  std::string mDenoiseAlbedoName;
  std::string mDenoiseNormalName;
#endif
};

} // namespace renderer
} // namespace svulkan2