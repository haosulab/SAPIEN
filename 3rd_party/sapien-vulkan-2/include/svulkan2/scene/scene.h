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
#include "camera.h"
#include "light.h"
#include "node.h"
#include "object.h"
#include "svulkan2/core/as.h"
#include "svulkan2/core/command_pool.h"
#include <memory>
#include <vector>

namespace svulkan2 {

namespace resource {
class SVCubemap;
}

namespace scene {

class Scene {
public:
  inline Node &getRootNode() { return *mRootNode; };

  Node &addNode(Transform const &transform = {});
  Node &addNode(Node &parent, Transform const &transform = {});

  Object &addObject(std::shared_ptr<resource::SVModel> model, Transform const &transform = {});
  Object &addObject(Node &parent, std::shared_ptr<resource::SVModel> model,
                    Transform const &transform = {});

  Object &addDeformableObject(std::shared_ptr<resource::SVModel> model);

  LineObject &addLineObject(std::shared_ptr<resource::SVLineSet> lineSet,
                            Transform const &transform = {});
  LineObject &addLineObject(Node &parent, std::shared_ptr<resource::SVLineSet> lineSet,
                            Transform const &transform = {});
  PointObject &addPointObject(std::shared_ptr<resource::SVPointSet> pointSet,
                              Transform const &transform = {});
  PointObject &addPointObject(Node &parent, std::shared_ptr<resource::SVPointSet> pointSet,
                              Transform const &transform = {});
  Camera &addCamera(Transform const &transform = {});
  Camera &addCamera(Node &parent, Transform const &transform = {});

  PointLight &addPointLight();
  PointLight &addPointLight(Node &parent);

  DirectionalLight &addDirectionalLight();
  DirectionalLight &addDirectionalLight(Node &parent);

  SpotLight &addSpotLight();
  SpotLight &addSpotLight(Node &parent);

  TexturedLight &addTexturedLight();
  TexturedLight &addTexturedLight(Node &parent);

  ParallelogramLight &addParallelogramLight();
  ParallelogramLight &addParallelogramLight(Node &parent);

  void removeNode(Node &node);
  void clearNodes();
  void forceRemove();

  inline void setAmbientLight(glm::vec4 const &color) { mAmbientLight = color; }
  inline glm::vec4 getAmbientLight() const { return mAmbientLight; };

  virtual std::vector<Object *> getObjects();

  std::vector<LineObject *> getLineObjects();
  std::vector<PointObject *> getPointObjects();
  std::vector<Camera *> getCameras();
  std::vector<PointLight *> getPointLights();
  std::vector<DirectionalLight *> getDirectionalLights();
  std::vector<SpotLight *> getSpotLights();
  std::vector<TexturedLight *> getTexturedLights();
  std::vector<ParallelogramLight *> getParallelogramLights();

  void setEnvironmentMap(std::shared_ptr<resource::SVCubemap> map) { mEnvironmentMap = map; }

  std::shared_ptr<resource::SVCubemap> getEnvironmentMap() const { return mEnvironmentMap; }

  Scene();
  Scene(Scene const &other) = delete;
  Scene &operator=(Scene const &other) = delete;
  Scene(Scene &&other) = delete;
  Scene &operator=(Scene &&other) = delete;

  void uploadToDevice(core::Buffer &sceneBuffer, StructDataLayout const &sceneLayout);
  void uploadShadowToDevice(core::Buffer &shadowBuffer,
                            std::vector<std::unique_ptr<core::Buffer>> &lightBuffers,
                            StructDataLayout const &shadowLayout);

  /** call exactly once per time frame to update the object matrices
   *  this function also copies current matrices into the previous matrices
   */
  void updateModelMatrices();

  /** called to order shadow lights before non-shadow lights */
  void reorderLights();

  uint64_t getVersion() const { return mVersion; }
  void updateVersion();

  uint64_t getRenderVersion() const { return mRenderVersion; }

  /** call this function to notify something has changed in the scene but the scene does not know.
   * E.g., some deformable mesh is modified */
  void updateRenderVersion();

  // ray tracing
  void buildRTResources(StructDataLayout const &materialBufferLayout,
                        StructDataLayout const &textureIndexBufferLayout,
                        StructDataLayout const &geometryInstanceBufferLayout);
  void updateRTResources();
  inline core::TLAS *getTLAS() const { return mTLAS.get(); }

  inline std::vector<vk::Buffer> const &getRTPointSetBuffers() { return mPointSetBuffers; }
  inline std::vector<vk::Buffer> const &getRTVertexBuffers() { return mVertexBuffers; }
  inline std::vector<vk::Buffer> const &getRTIndexBuffers() const { return mIndexBuffers; }
  inline std::vector<vk::Buffer> const &getRTMaterialBuffers() const { return mMaterialBuffers; }
  inline std::vector<std::tuple<vk::ImageView, vk::Sampler>> const &getRTTextures() const {
    return mTextures;
  }
  inline vk::Buffer getRTGeometryInstanceBuffer() const {
    return mGeometryInstanceBuffer->getVulkanBuffer();
  }
  inline vk::Buffer getRTPointInstanceBuffer() const {
    return mPointInstanceBuffer->getVulkanBuffer();
  }
  inline vk::Buffer getRTTextureIndexBuffer() const {
    return mTextureIndexBuffer->getVulkanBuffer();
  }
  inline vk::Buffer getRTPointLightBuffer() const {
    return mRTPointLightBuffer->getVulkanBuffer();
  }
  inline vk::Buffer getRTDirectionalLightBuffer() const {
    return mRTDirectionalLightBuffer->getVulkanBuffer();
  }
  inline vk::Buffer getRTSpotLightBuffer() const { return mRTSpotLightBuffer->getVulkanBuffer(); }
  inline vk::Buffer getRTParallelogramLightBuffer() const {
    return mRTParallelogramLightBuffer->getVulkanBuffer();
  }

  void registerAccessFence(vk::Fence fence);
  void unregisterAccessFence(vk::Fence fence);

  size_t getGpuTransformBufferSize();
  void prepareObjectTransformBuffer();
  std::shared_ptr<core::Buffer> getObjectTransformBuffer();
  virtual void uploadObjectTransforms();

protected:
  std::vector<std::unique_ptr<Node>> mNodes{};
  std::vector<std::unique_ptr<Object>> mObjects{};
  std::vector<std::unique_ptr<Object>> mDeformableObjects{};
  std::vector<std::unique_ptr<LineObject>> mLineObjects{};
  std::vector<std::unique_ptr<PointObject>> mPointObjects{};
  std::vector<std::unique_ptr<Camera>> mCameras{};
  std::vector<std::unique_ptr<PointLight>> mPointLights{};
  std::vector<std::unique_ptr<DirectionalLight>> mDirectionalLights{};
  std::vector<std::unique_ptr<SpotLight>> mSpotLights{};
  std::vector<std::unique_ptr<TexturedLight>> mTexturedLights{};
  std::vector<std::unique_ptr<ParallelogramLight>> mParallelogramLights{};

  Node *mRootNode{nullptr};

  bool mRequireForceRemove{};

  // lighting
  glm::vec4 mAmbientLight{};
  std::shared_ptr<resource::SVCubemap> mEnvironmentMap{};

  /** when anything is added or removed, the version changes */
  uint64_t mVersion{1l};

  /** when models in the scene are updated, the version changes */
  uint64_t mRenderVersion{1l};

  // for transform update and AS update
  std::unique_ptr<core::CommandPool> mCommandPool;
  core::CommandPool &getCommandPool();

  // transform buffers
  size_t mGpuTransformBufferSize{0l};
  uint64_t mTransformBufferVersion{0l};
  uint64_t mTransformBufferRenderVersion{0l};
  std::shared_ptr<core::Buffer> mTransformBufferCpu;
  std::shared_ptr<core::Buffer> mTransformBuffer;
  vk::UniqueCommandBuffer mTransformUpdateCommandBuffer;
  vk::UniqueFence mTransformUpdateFence{};

  // ray tracing helpers
  void ensureBLAS();
  void buildTLAS();

  void updateDynamicBLAS();
  void updateTLAS();
  void createRTStorageBuffers(StructDataLayout const &materialBufferLayout,
                              StructDataLayout const &textureIndexBufferLayout,
                              StructDataLayout const &geometryInstanceBufferLayout);
  void updateRTStorageBuffers();

  // ray tracing resources
  std::mutex mRTResourcesLock;
  std::unique_ptr<core::TLAS> mTLAS;
  std::vector<vk::Buffer> mVertexBuffers;
  std::vector<vk::Buffer> mIndexBuffers;
  std::vector<vk::Buffer> mMaterialBuffers;
  std::vector<std::tuple<vk::ImageView, vk::Sampler>> mTextures;
  std::unique_ptr<core::Buffer> mTextureIndexBuffer;
  std::unique_ptr<core::Buffer> mGeometryInstanceBuffer;

  std::vector<vk::Buffer> mPointSetBuffers;
  std::unique_ptr<core::Buffer> mPointInstanceBuffer;

  struct RTPointLight {
    glm::vec3 position;
    float radius;
    glm::vec3 rgb;
    float padding;
  };
  static_assert(sizeof(RTPointLight) == 32);

  struct RTDirectionalLight {
    glm::vec3 direction;
    float softness;
    glm::vec3 rgb;
    float padding;
  };
  static_assert(sizeof(RTDirectionalLight) == 32);

  struct RTSpotLight {
    glm::mat4 viewMat;
    glm::mat4 projMat;
    glm::vec3 rgb;
    int padding0;
    glm::vec3 position;
    int padding1;
    float fovInner;
    float fovOuter;
    int textureId;
    int padding2;
  };
  static_assert(sizeof(RTSpotLight) == 176);

  struct RTParallelogramLight {
    glm::vec3 color;
    float padding0;
    glm::vec3 position;
    float padding1;
    glm::vec3 edge0;
    float padding2;
    glm::vec3 edge1;
    float padding3;
  };

  std::vector<RTPointLight> mRTPointLightBufferHost;
  std::vector<RTDirectionalLight> mRTDirectionalLightBufferHost;
  std::vector<RTSpotLight> mRTSpotLightBufferHost;
  std::vector<RTParallelogramLight> mRTParallelogramLightBufferHost;

  std::unique_ptr<core::Buffer> mRTPointLightBuffer;
  std::unique_ptr<core::Buffer> mRTDirectionalLightBuffer;
  std::unique_ptr<core::Buffer> mRTSpotLightBuffer;
  std::unique_ptr<core::Buffer> mRTParallelogramLightBuffer;

  uint64_t mRTResourcesVersion{0l};
  uint64_t mRTResourcesRenderVersion{0l};

  // indicates whether this scene is being accessed
  std::vector<vk::Fence> mAccessFences;

  // used to update AS
  vk::UniqueCommandBuffer mASUpdateCommandBuffer;
};

} // namespace scene
} // namespace svulkan2