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
#include "svulkan2/core/context.h"
#include "svulkan2/core/image.h"
#include "svulkan2/scene/scene.h"

namespace svulkan2 {
namespace renderer {

class RendererBase {
public:
  static std::unique_ptr<RendererBase>
  Create(std::shared_ptr<RendererConfig> config);

  virtual void resize(int width, int height) = 0;
  virtual void setScene(std::shared_ptr<scene::Scene> scene) = 0;

  virtual void render(scene::Camera &camera,
                      std::vector<vk::Semaphore> const &waitSemaphores,
                      std::vector<vk::PipelineStageFlags> const &waitStages,
                      std::vector<vk::Semaphore> const &signalSemaphores,
                      vk::Fence fence) = 0;

  virtual void render(
      scene::Camera &camera,
      vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
      vk::ArrayProxyNoTemporaries<vk::PipelineStageFlags const> const
          &waitStageMasks,
      vk::ArrayProxyNoTemporaries<uint64_t const> const &waitValues,
      vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &signalSemaphores,
      vk::ArrayProxyNoTemporaries<uint64_t const> const &signalValues) = 0;

  virtual void display(std::string const &imageName, vk::Image backBuffer,
                       vk::Format format, uint32_t width, uint32_t height,
                       std::vector<vk::Semaphore> const &waitSemaphores,
                       std::vector<vk::PipelineStageFlags> const &waitStages,
                       std::vector<vk::Semaphore> const &signalSemaphores,
                       vk::Fence fence) = 0;

  virtual void
  setCustomTextureArray(std::string const &name,
                        std::vector<std::shared_ptr<resource::SVTexture>>
                            texture){}; // TODO: implement
  virtual void
  setCustomTexture(std::string const &name,
                   std::shared_ptr<resource::SVTexture> texture) = 0;
  virtual void
  setCustomCubemap(std::string const &name,
                   std::shared_ptr<resource::SVCubemap> cubemap) = 0;

  virtual int getCustomPropertyInt(std::string const &name) const = 0;
  virtual float getCustomPropertyFloat(std::string const &name) const = 0;
  virtual glm::vec3 getCustomPropertyVec3(std::string const &name) const = 0;
  virtual glm::vec4 getCustomPropertyVec4(std::string const &name) const = 0;

  virtual void setCustomProperty(std::string const &name, int p) {}
  virtual void setCustomProperty(std::string const &name, float p) {}
  virtual void setCustomProperty(std::string const &name, glm::vec3 p) {}
  virtual void setCustomProperty(std::string const &name, glm::vec4 p) {}


  virtual std::vector<std::string> getDisplayTargetNames() const = 0;
  virtual std::vector<std::string> getRenderTargetNames() const = 0;
  virtual core::Image &getRenderImage(std::string const &name) = 0;
  virtual vk::ImageLayout getRenderTargetImageLayout(std::string const &name) = 0;

  template <typename T>
  std::tuple<std::vector<T>, std::array<uint32_t, 3>>
  download(std::string const &targetName) {
    auto &image = getRenderImage(targetName);
    uint32_t width = image.getExtent().width;
    uint32_t height = image.getExtent().height;
    std::vector<T> data = image.download<T>();
    uint32_t channels = data.size() / (width * height);
    if (width * height * channels != data.size()) {
      throw std::runtime_error(
          "download render target failed: internal format error");
    }
    return {data, std::array<uint32_t, 3>{height, width, channels}};
  }

  template <typename T>
  std::tuple<std::vector<T>, std::array<uint32_t, 3>>
  downloadRegion(std::string const &targetName, vk::Offset2D offset,
                 vk::Extent2D extent) {
    auto &image = getRenderImage(targetName);
    uint32_t width = image.getExtent().width;
    uint32_t height = image.getExtent().height;
    if (offset.x < 0 || offset.y < 0 || offset.x + extent.width >= width ||
        offset.y + extent.height >= height) {
      throw std::runtime_error(
          "failed to download region: offset or extent is out of bound");
    }
    std::vector<T> data =
        image.download<T>(vk::Offset3D{offset.x, offset.y, 0},
                          vk::Extent3D{extent.width, extent.height, 1});
    uint32_t channels = data.size() / (extent.width * extent.height);
    if (extent.width * extent.height * channels != data.size()) {
      throw std::runtime_error(
          "failed to download region: internal format error");
    }
    return {data,
            std::array<uint32_t, 3>{extent.height, extent.width, channels}};
  }

  virtual ~RendererBase() = default;
};

} // namespace renderer
} // namespace svulkan2