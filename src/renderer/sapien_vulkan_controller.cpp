#ifdef _USE_VULKAN
#ifdef ON_SCREEN
#include "sapien_vulkan_controller.h"
#include "sapien_scene.h"
#include "sapien_actor_base.h"
#include "articulation/sapien_link.h"

namespace sapien {
namespace Renderer {

SapienVulkanController::SapienVulkanController(SapienVulkanRenderer *renderer)
    : mRenderer(renderer), mWidth(0), mHeight(0) {
  mCamera = mRenderer->mContext->createCamera();
  mVulkanRenderer = mRenderer->mContext->createVulkanRendererForEditor();
  mFPSController = std::make_unique<svulkan::FPSCameraController>(
      *mCamera, glm::vec3{1.f, 0.f, 0.f}, glm::vec3{0.f, 0.f, 1.f});
  mWindow = mRenderer->mContext->createWindow(1280, 720);
  mWindow->initImgui(mRenderer->mContext->getDescriptorPool(),
                     mRenderer->mContext->getCommandPool());

  mCommandBuffer = svulkan::createCommandBuffer(mRenderer->mContext->getDevice(),
                                                mRenderer->mContext->getCommandPool(),
                                                vk::CommandBufferLevel::ePrimary);
  mFence =
      mRenderer->mContext->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  mSemaphore = mRenderer->mContext->getDevice().createSemaphoreUnique({});

  mFPSController->setXYZ(0, 0, 0);
}

void SapienVulkanController::render() {
  if (mWindow->isClosed()) {
    return;
  }
  if (!mScene) {
    return;
  }

  do {
    int width, height;
    glfwGetFramebufferSize(mWindow->getWindow(), &width, &height);
    if (static_cast<uint32_t>(width) != mWidth || static_cast<uint32_t>(height) != mHeight) {
      mRenderer->mContext->getDevice().waitIdle();
      mWindow->updateSize(width, height);
      mVulkanRenderer->resize(mWindow->getWidth(), mWindow->getHeight());
      mWidth = mWindow->getWidth();
      mHeight = mWindow->getHeight();
      mCamera->aspect = static_cast<float>(mWidth) / mHeight;
    }

    // get the next image
    try {
      mWindow->newFrame();
    } catch (vk::OutOfDateKHRError &e) {
      mRenderer->mContext->getDevice().waitIdle();
      return;
    }

    {
      ImGui::NewFrame();
      mHudControlWindow.draw();
      mHudObjectWindow.draw(mScene, mSelectedId);
      ImGui::Render();
    }

    // wait for previous frame to finish
    {
      mRenderer->mContext->getDevice().waitForFences(mFence.get(), VK_TRUE, UINT64_MAX);
      mRenderer->mContext->getDevice().resetFences(mFence.get());
    }

    auto scene = static_cast<SapienVulkanScene *>(mScene->getRendererScene());
    // draw
    {
      mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
      mVulkanRenderer->render(mCommandBuffer.get(), *scene->getScene(), *mCamera);
      mVulkanRenderer->display(mCommandBuffer.get(), mWindow->getBackBuffer(),
                               mWindow->getBackBufferFormat(), mWindow->getWidth(),
                               mWindow->getHeight());
      mCommandBuffer->end();

      auto imageAcquiredSemaphore = mWindow->getImageAcquiredSemaphore();
      vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
      vk::SubmitInfo info(1, &imageAcquiredSemaphore, &waitStage, 1, &mCommandBuffer.get(), 1,
                          &mSemaphore.get());
      mRenderer->mContext->getGraphicsQueue().submit(info, {});
    }

    try {
      mWindow->presentFrameWithImgui(mRenderer->mContext->getGraphicsQueue(),
                                     mWindow->getPresentQueue(), mSemaphore.get(), mFence.get());
    } catch (vk::OutOfDateKHRError &e) {
      return;
    }
    // wait idle

    if (mDefaultKeyPressBehavior) {
      if (mWindow->isKeyDown('q')) {
        mWindow->close();
      }
      float r = mHudControlWindow.mHudStats.mFrameRate > 0
                ? std::clamp(1 / mHudControlWindow.mHudStats.mFrameRate, 0.f, 1.f)
                : 0.f;

      r *= mHudControlWindow.mHudControl.mMoveSpeed;
      if (mWindow->isKeyDown('w')) {
        mFPSController->move(r, 0, 0);
      }
      if (mWindow->isKeyDown('s')) {
        mFPSController->move(-r, 0, 0);
      }
      if (mWindow->isKeyDown('a')) {
        mFPSController->move(0, r, 0);
      }
      if (mWindow->isKeyDown('d')) {
        mFPSController->move(0, -r, 0);
      }
    }

    if (mDefaultMouseClickBehavior) {
      float r = mHudControlWindow.mHudStats.mFrameRate > 0
                ? std::clamp(1 / mHudControlWindow.mHudStats.mFrameRate, 0.f, 1.f)
                : 0.f;
      if (mWindow->isMouseKeyDown(1)) {
        auto [x, y] = mWindow->getMouseDelta();
        float r1 = mHudControlWindow.mHudControl.mInvertX ? -r : r;
        float r2 = mHudControlWindow.mHudControl.mInvertY ? -r : r;
        r1 *= mHudControlWindow.mHudControl.mRotateSpeed;
        r2 *= mHudControlWindow.mHudControl.mRotateSpeed;
        mFPSController->rotate(0, -r2 * y, -r1 * x);
      }

      if (!ImGui::GetIO().WantCaptureMouse && ImGui::IsMouseClicked(0)) {
        auto [x, y] = mWindow->getMousePosition();
        auto pixel = mVulkanRenderer->getRenderTargets().segmentation->downloadPixel<uint32_t>(
            mRenderer->mContext->getPhysicalDevice(),
            mRenderer->mContext->getDevice(),
            mRenderer->mContext->getCommandPool(),
            mRenderer->mContext->getGraphicsQueue(),
            static_cast<int>(x), static_cast<int>(y));
        if (pixel.size()) {
          auto actorId = pixel[1];
          selectActor(actorId);
        } else {
          std::cout << "None" << std::endl;
        }
      }
    }

    if (mHudControlWindow.mHudRenderMode.mSwitchMode) {
      switch (mHudControlWindow.mHudRenderMode.mMode) {
        case 0:
          mVulkanRenderer->switchToLighting();
          break;
        case 1:
          mVulkanRenderer->switchToNormal();
          break;
        case 2:
          mVulkanRenderer->switchToDepth();
          break;
        default:
          break;
      }
    }

    if (mHudObjectWindow.mHudWorld.mSelect) {
      selectActor(mHudObjectWindow.mHudWorld.mSelectedId);
    } else if (mHudObjectWindow.mHudArticulation.mSelect) {
      selectActor(mHudObjectWindow.mHudArticulation.mSelectedId);
    }

    if (mSelectedId) {
      mVulkanRenderer->clearAxes();
      auto actor = mScene->findActorById(mSelectedId);
      if (!actor) {
        actor = mScene->findArticulationLinkById(mSelectedId);
      }
      if (actor) {
        auto pose =  actor->getPose();
        if (mHudObjectWindow.mHudActor.mShowCenterOfMass &&
            actor->getType() != EActorType::STATIC) {
          pose = pose * static_cast<SActorDynamicBase*>(actor)->getCMassLocalPose();
        }
        glm::mat4 mat(1);
        mat[0][0] *= 0.1;
        mat[1][1] *= 0.1;
        mat[2][2] *= 0.1;
        mat = glm::mat4(glm::quat(pose.q.w, pose.q.x, pose.q.y, pose.q.z)) * mat;
        mat[3][0] = pose.p.x;
        mat[3][1] = pose.p.y;
        mat[3][2] = pose.p.z;
        mVulkanRenderer->addAxes(mat);
      }
    }

  } while(mHudControlWindow.mHudControl.mPause);
}


void SapienVulkanController::selectActor(physx_id_t actorId) {
  mSelectedId = actorId;
}

} // namespace Renderer
} // namespace sapien
#endif
#endif
