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
#include "sapien/sapien_renderer/batched_render_system.h"
#include "./batched_render_system.cuh"
#include "sapien/sapien_renderer/camera_component.h"
#include "sapien/sapien_renderer/render_body_component.h"
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/renderer/renderer_base.h>

#ifdef SAPIEN_CUDA
#include "sapien/utils/cuda.h"
#include <cuda_runtime.h>
#endif

namespace sapien {
namespace sapien_renderer {

BatchedCamera::BatchedCamera(std::vector<std::shared_ptr<SapienRenderCameraComponent>> cameras,
                             std::vector<std::string> renderTargets)
    : mCameras(cameras) {
  if (cameras.empty()) {
    throw std::runtime_error("failed to create BatchedCamera: empty cameras");
  }
  uint32_t width = cameras.at(0)->getWidth();
  uint32_t height = cameras.at(0)->getHeight();

  for (auto &cam : cameras) {
    if (!cam->getScene()) {
      throw std::runtime_error(
          "failed to create BatchedCamera: some camera is not added to scene");
    }
    if (cam->getWidth() != width || cam->getHeight() != height) {
      throw std::runtime_error(
          "failed to create BatchedCamera: the cameras must have the same width and height");
    }
  }

  auto context = SapienRenderEngine::Get()->getContext();
  mCommandPool = context->createCommandPool();
  mCommandBuffer = mCommandPool->allocateCommandBuffer();

  for (auto &cam : cameras) {
    cam->gpuInit();
  }

  mCommandBuffer->begin(vk::CommandBufferBeginInfo());
  for (uint32_t i = 0; i < cameras.size(); ++i) {
    auto &cam = cameras[i];

    for (auto &name : renderTargets) {
      auto &image = cam->getInternalImage(name);
      auto extent = image.getExtent();
      vk::Format format = image.getFormat();
      vk::DeviceSize imageSize =
          extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);
      if (!mCudaImageBuffers.contains(name)) {
        mCudaImageBuffers[name] = svulkan2::core::Buffer::Create(
            imageSize * cameras.size(),
            vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
            VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
        {
          CudaArrayHandle array;
          int channels = getFormatChannels(format);
          int itemsize = getFormatChannelSize(format);
          array.shape = {static_cast<int>(cameras.size()), static_cast<int>(extent.height),
                         static_cast<int>(extent.width)};
          array.strides = {static_cast<int>(itemsize * channels * width * height),
                           static_cast<int>(itemsize * channels * width),
                           static_cast<int>(itemsize * channels)};
          if (channels != 1) {
            array.shape.push_back(channels);
            array.strides.push_back(itemsize);
          }
          array.type = getFormatTypestr(format);
#ifdef SAPIEN_CUDA
          array.ptr = mCudaImageBuffers[name]->getCudaPtr();
          array.cudaId = mCudaImageBuffers[name]->getCudaDeviceId();
#endif
          mCudaImageHandles[name] = array;
        }
      }
      image.recordCopyToBuffer(mCommandBuffer.get(),
                               cam->getInternalRenderer().getRenderTargetImageLayout(name),
                               mCudaImageBuffers[name]->getVulkanBuffer(), i * imageSize,
                               imageSize, {0, 0, 0}, extent, 0);
    }
  }
  mCommandBuffer->end();

  // setup semaphore for CUDA
  mFrameCounter = 0;
  vk::SemaphoreTypeCreateInfo timelineCreateInfo(vk::SemaphoreType::eTimeline, 0);
  vk::SemaphoreCreateInfo createInfo{};
  vk::ExportSemaphoreCreateInfo exportCreateInfo(
      vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd);
  createInfo.setPNext(&exportCreateInfo);
  exportCreateInfo.setPNext(&timelineCreateInfo);
  auto device = context->getDevice();
  mSemaphore = device.createSemaphoreUnique(createInfo);

  int fd = device.getSemaphoreFdKHR(
      {mSemaphore.get(), vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd});
#ifdef SAPIEN_CUDA
  cudaExternalSemaphoreHandleDesc desc = {};
  desc.flags = 0;
  desc.handle.fd = fd;
  desc.type = cudaExternalSemaphoreHandleTypeTimelineSemaphoreFd;
  checkCudaErrors(cudaImportExternalSemaphore(&mCudaSem, &desc));
#endif
  // TODO clean up cudaSem
}

void BatchedCamera::takePicture() {
  auto context = SapienRenderEngine::Get()->getContext();

  // make sure previous takePicture has finished
  auto result = context->getDevice().waitSemaphores(
      vk::SemaphoreWaitInfo({}, mSemaphore.get(), mFrameCounter), UINT64_MAX);

  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("take picture failed: wait for fence failed");
  }

  for (auto &cam : mCameras) {
    cam->getInternalRenderer().render(cam->getInternalCamera(), {}, {}, {}, {});
  }
  mFrameCounter++;
  context->getQueue().submit(mCommandBuffer.get(), {}, {}, {}, mSemaphore.get(), mFrameCounter,
                             {});
#ifdef SAPIEN_CUDA
  cudaExternalSemaphoreWaitParams waitParams{};
  waitParams.params.fence.value = mFrameCounter;
  cudaWaitExternalSemaphoresAsync(&mCudaSem, &waitParams, 1, mCudaStream);
#endif
}

CudaArrayHandle BatchedCamera::getPictureCuda(std::string const &name) {
  if (!mCudaImageHandles.contains(name)) {
    throw std::runtime_error("Failed to get image with name :" + name +
                             ". Did you forget to specify it in create_camera_group?");
  }
  return mCudaImageHandles.at(name);
}

BatchedCamera::~BatchedCamera() {
  SapienRenderEngine::Get()->getContext()->getDevice().waitIdle();
#ifdef SAPIEN_CUDA
  cudaDestroyExternalSemaphore(mCudaSem);
#endif
}

BatchedRenderSystem::BatchedRenderSystem(
    std::vector<std::shared_ptr<SapienRendererSystem>> systems)
    : mSystems(systems) {
  if (mSystems.empty()) {
    throw std::runtime_error("systems must not be empty");
  }
  init();
}

void BatchedRenderSystem::init() {
  std::vector<RenderShapeData> allShapeData;
  std::vector<void *> sceneTransformRefs;

  mSceneVersions = {};

  // TODO ensure all cameras are valid
  for (uint32_t sceneIndex = 0; sceneIndex < mSystems.size(); ++sceneIndex) {
    auto system = mSystems[sceneIndex];

    // run a step
    system->step();

    // cache current versions
    mSceneVersions.push_back(system->getScene()->getVersion());

    auto transformArray = system->getTransformCudaArray();
    sceneTransformRefs.push_back(transformArray.ptr);

    if (mTransformBufferElementByteOffset == 0) {
      mTransformBufferElementByteOffset = transformArray.strides.at(0);
      if (mTransformBufferElementByteOffset % 4 != 0) {
        throw std::runtime_error("corrupted transform array buffer");
      }
    } else if (mTransformBufferElementByteOffset != transformArray.strides.at(0)) {
      throw std::runtime_error("corrupted transform array buffer");
    }

    for (auto &body : system->getRenderBodyComponents()) {
      for (auto &shape : body->getRenderShapes()) {
        int poseIndex = shape->getGpuBatchedPoseIndex();

        if (poseIndex < 0) {
          continue;
        }

        // TODO do range check
        Pose localPose = shape->getLocalPose();
        Vec3 scale = shape->getGpuScale();
        int transformIndex = shape->getInternalGpuTransformIndex();

        static_assert(sizeof(RenderShapeData) == 4 * 13);
        RenderShapeData data;
        data.localPose = localPose;
        data.scale = scale;

        data.poseIndex = poseIndex;
        data.sceneIndex = sceneIndex;
        data.transformIndex = transformIndex;

        allShapeData.push_back(data);
      }
    }
  }
  mShapeCount = allShapeData.size();

#ifdef SAPIEN_CUDA
  checkCudaErrors(cudaSetDevice(SapienRenderEngine::Get()->getDevice()->cudaId));
#endif
  mCudaShapeDataBuffer = CudaArray::FromData(allShapeData);
  mCudaSceneTransformRefBuffer = CudaArray::FromData(sceneTransformRefs);

  // create semaphore
  auto context = SapienRenderEngine::Get()->getContext();
  if (!mSem) {
    vk::SemaphoreTypeCreateInfo timelineCreateInfo(vk::SemaphoreType::eTimeline, 0);
    vk::SemaphoreCreateInfo createInfo{};
    vk::ExportSemaphoreCreateInfo exportCreateInfo(
        vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd);
    createInfo.setPNext(&exportCreateInfo);
    exportCreateInfo.setPNext(&timelineCreateInfo);
    mSem = context->getDevice().createSemaphoreUnique(createInfo);

    int fd = context->getDevice().getSemaphoreFdKHR(
        {mSem.get(), vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd});
#ifdef SAPIEN_CUDA
    cudaExternalSemaphoreHandleDesc desc = {};
    desc.flags = 0;
    desc.handle.fd = fd;
    desc.type = cudaExternalSemaphoreHandleTypeTimelineSemaphoreFd;
    checkCudaErrors(cudaImportExternalSemaphore(&mCudaSem, &desc));
#endif
  }
}

void BatchedRenderSystem::setPoseSource(CudaArrayHandle const &poses) {
  poses.checkCongiguous();
  poses.checkShape({-1, -1});
  poses.checkStride({-1, sizeof(float)});

  mCudaPoseHandle = poses;
}

std::shared_ptr<BatchedCamera> BatchedRenderSystem::createCameraBatch(
    std::vector<std::shared_ptr<SapienRenderCameraComponent>> cameras,
    std::vector<std::string> renderTargets) {
  auto cameraBatch = std::make_shared<BatchedCamera>(cameras, renderTargets);
  cameraBatch->setCudaStream(mCudaStream);

  mCameraBatches.push_back(cameraBatch);

  std::vector<CameraData> allCamData;
  mCameraCount = 0;
  for (auto &cb : mCameraBatches) {
    for (auto &cam : cb->getCameras()) {
      int index = cam->getGpuBatchedPoseIndex();
      if (index < 0) {
        // this camera does not need to be updated
        continue;
      }
      CameraData data;
      data.buffer = cam->getCudaBuffer().ptr;
      auto pose = cam->getLocalPose() * POSE_GL_TO_ROS;
      data.localPose = pose;

      data.poseIndex = index;

      allCamData.push_back(data);
      mCameraCount++;
    }
  }
#ifdef SAPIEN_CUDA
  checkCudaErrors(cudaSetDevice(SapienRenderEngine::Get()->getDevice()->cudaId));
#endif
  mCudaCameraDataBuffer = CudaArray::FromData(allCamData);
  return cameraBatch;
}

void BatchedRenderSystem::update() {
  // check pose handle
  if (!mCudaPoseHandle.ptr) {
    throw std::runtime_error("the data source for pose has not been set.");
  }

  // check scene versions
  for (uint32_t i = 0; i < mSystems.size(); ++i) {
    if (mSystems.at(i)->getScene()->getVersion() != mSceneVersions.at(i)) {
      throw std::runtime_error("Modifying a scene (add/remove object/camera) is not allowed after "
                               "creating the batched render system.");
    }
  }

  if (mCudaSceneTransformRefBuffer.cudaId != mCudaPoseHandle.cudaId) {
    throw std::runtime_error("failed to update render system group: cuda pose buffer (cuda:" +
                             std::to_string(mCudaPoseHandle.cudaId) +
                             ") and the "
                             "renderer (cuda:" +
                             std::to_string(mCudaSceneTransformRefBuffer.cudaId) +
                             ") are on different cuda devices.");
  }

  // upload data
#ifdef SAPIEN_CUDA
  update_object_transforms(
      (float **)mCudaSceneTransformRefBuffer.ptr, mTransformBufferElementByteOffset / 4,
      (RenderShapeData *)mCudaShapeDataBuffer.ptr, (float *)mCudaPoseHandle.ptr,
      mCudaPoseHandle.shape.at(1), mShapeCount, mCudaStream);

  update_camera_transforms((CameraData *)mCudaCameraDataBuffer.ptr, (float *)mCudaPoseHandle.ptr,
                           mCudaPoseHandle.shape.at(1), mCameraCount, mCudaStream);
#endif
  // TODO: uplaod camera

  // sync with renderer
  notifyUpdate();
}

void BatchedRenderSystem::notifyUpdate() {
#ifdef SAPIEN_CUDA
  cudaExternalSemaphoreSignalParams sigParams{};
  sigParams.flags = 0;
  sigParams.params.fence.value = ++mSemValue;
  cudaSignalExternalSemaphoresAsync(&mCudaSem, &sigParams, 1, (cudaStream_t)mCudaStream);
#endif
  vk::PipelineStageFlags stage = vk::PipelineStageFlagBits::eAllCommands;
  SapienRenderEngine::Get()->getContext()->getQueue().submit({}, mSem.get(), stage, mSemValue, {},
                                                             {}, {});
}

void BatchedRenderSystem::setCudaStream(uintptr_t stream) {
  mCudaStream = (cudaStream_t)stream;
  for (auto &c : mCameraBatches) {
    c->setCudaStream(mCudaStream);
  }
}

BatchedRenderSystem ::~BatchedRenderSystem() {
  SapienRenderEngine::Get()->getContext()->getDevice().waitIdle();
#ifdef SAPIEN_CUDA
  cudaDestroyExternalSemaphore(mCudaSem);
#endif
}

} // namespace sapien_renderer
} // namespace sapien
