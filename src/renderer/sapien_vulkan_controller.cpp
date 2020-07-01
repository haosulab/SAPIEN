#ifdef _USE_VULKAN
#ifdef ON_SCREEN
#include "sapien_vulkan_controller.h"
#include "articulation/sapien_link.h"
#include "sapien_actor.h"
#include "sapien_actor_base.h"
#include "sapien_drive.h"
#include "sapien_scene.h"

#include "ImGuizmo.h"

namespace sapien {
namespace Renderer {

glm::mat4 PxTransform2Mat4(physx::PxTransform const &t) {
  glm::mat4 rot = glm::toMat4(glm::quat(t.q.w, t.q.x, t.q.y, t.q.z));
  glm::mat4 pos = glm::translate(glm::mat4(1.f), {t.p.x, t.p.y, t.p.z});
  return pos * rot;
}

static physx::PxTransform Mat42PxTransform(glm::mat4 const &m) {
  glm::vec3 scale;
  glm::quat rot;
  glm::vec3 pos;
  glm::vec3 skew;
  glm::vec4 perspective;
  glm::decompose(m, scale, rot, pos, skew, perspective);
  return {{pos.x, pos.y, pos.z}, {rot.x, rot.y, rot.z, rot.w}};
}

void SapienVulkanController::editTransform() {
  static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);
  static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
  static bool useSnap = false;
  static float snap[3] = {1.f, 1.f, 1.f};
  static float bounds[] = {-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};
  static float boundsSnap[] = {0.1f, 0.1f, 0.1f};
  static bool boundSizing = false;
  static bool boundSizingSnap = false;

  // ImGui::SetNextWindowPos(ImVec2(imguiWindowSize, 0));
  // ImGui::SetNextWindowSize(ImVec2(mRenderer->mContext->getWidth() - 2 * imguiWindowSize, 0));

  ImGui::Begin("Gizmo");
  if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
    mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
  ImGui::SameLine();
  if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
    mCurrentGizmoOperation = ImGuizmo::ROTATE;
  float matrixTranslation[3], matrixRotation[3], matrixScale[3] = {1, 1, 1};
  ImGuizmo::DecomposeMatrixToComponents(&mGizmoTransform[0][0], matrixTranslation, matrixRotation,
                                        matrixScale);
  ImGui::InputFloat3("Tr", matrixTranslation, 3);
  ImGui::InputFloat3("Rt", matrixRotation, 3);
  ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale,
                                          &mGizmoTransform[0][0]);

  if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
    mCurrentGizmoMode = ImGuizmo::LOCAL;
  if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
    mCurrentGizmoMode = ImGuizmo::WORLD;
  ImGui::Checkbox("", &useSnap);
  ImGui::SameLine();

  switch (mCurrentGizmoOperation) {
  case ImGuizmo::TRANSLATE:
    ImGui::InputFloat3("Snap", &snap[0]);
    break;
  case ImGuizmo::ROTATE:
    ImGui::InputFloat("Angle Snap", &snap[0]);
    break;
  default:
    break;
  }

  if (ImGui::Button("Reset")) {
    mGizmoTransform = glm::mat4(1);
    createGizmoVisual(nullptr);
  }
  auto pose = Mat42PxTransform(mGizmoTransform);
  for (auto b : mGizmoBody) {
    b->update(pose);
  }
  if (mSelectedId) {
    SActorBase *actor = mScene->findActorById(mSelectedId);
    if (!actor) {
      actor = mScene->findArticulationLinkById(mSelectedId);
    }
    if (actor &&
        (actor->getType() == EActorType::DYNAMIC || actor->getType() == EActorType::KINEMATIC)) {
      ImGui::SameLine();
      if (ImGui::Button("Teleport Actor")) {
        static_cast<SActor *>(actor)->setPose(pose);
      }
    }

    if (actor && (actor->getType() == EActorType::DYNAMIC ||
                  actor->getType() == EActorType::ARTICULATION_LINK)) {
      ImGui::SameLine();
      if (ImGui::Button("Drive Actor")) {
        SDrive *validDrive = nullptr;
        auto drives = actor->getDrives();
        for (SDrive *d : drives) {
          if (d->getActor1() == nullptr &&
              d->getLocalPose1() == PxTransform({{0, 0, 0}, PxIdentity}) &&
              d->getLocalPose2() == PxTransform({{0, 0, 0}, PxIdentity})) {
            validDrive = d;
          }
        }
        if (!validDrive) {
          validDrive = mScene->createDrive(nullptr, {{0, 0, 0}, PxIdentity}, actor,
                                           {{0, 0, 0}, PxIdentity});
          validDrive->setProperties(10000, 10000, PX_MAX_F32, false);
        }
        validDrive->setTarget(pose);
      }
    }

    if (actor) {
      if (ImGui::Button("Gizmo to actor")) {
        mGizmoTransform = PxTransform2Mat4(actor->getPose());
        createGizmoVisual(actor);
      }
    }
  }

  ImGuiIO &io = ImGui::GetIO();
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
  auto view = mCamera->getViewMat();
  auto proj = mCamera->getProjectionMat();
  proj[1][1] *= -1;
  ImGuizmo::Manipulate(&view[0][0], &proj[0][0], mCurrentGizmoOperation, mCurrentGizmoMode,
                       &mGizmoTransform[0][0], NULL, useSnap ? &snap[0] : NULL,
                       boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);
  ImGui::End();
}

void SapienVulkanController::createGizmoVisual(SActorBase *actor) {
  if (!mScene) {
    return;
  }
  for (auto b : mGizmoBody) {
    b->destroy();
  }
  mGizmoBody.clear();
  if (!actor) {
    return;
  }
  for (auto b : actor->getRenderBodies()) {
    auto b2 = static_cast<SapienVulkanScene *>(mScene->getRendererScene())
                  ->cloneRigidbody(static_cast<SapienVulkanRigidbody *>(b));
    b2->setVisibility(0.5);
    mGizmoBody.push_back(b2);
    mGizmoBody.back()->setUniqueId(0);
  }
}

SapienVulkanController::SapienVulkanController(SapienVulkanRenderer *renderer)
    : mRenderer(renderer), mWidth(0), mHeight(0) {
  mCamera = mRenderer->mContext->createCamera();
  mVulkanRenderer = mRenderer->mContext->createVulkanRendererForEditor();
  mFPSController = std::make_unique<svulkan::FPSCameraController>(
      *mCamera, glm::vec3{1.f, 0.f, 0.f}, glm::vec3{0.f, 0.f, 1.f});
  mArcRotateController = std::make_unique<svulkan::ArcRotateCameraController>(
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

    mHudControlWindow.mHudCameraInfo.mPosition = mCamera->position;
    mHudControlWindow.mHudCameraInfo.mRotation = mCamera->rotation;
    mHudControlWindow.mHudCameraInfo.mNear = mCamera->near;
    mHudControlWindow.mHudCameraInfo.mFar = mCamera->far;
    mHudControlWindow.mHudCameraInfo.mFovy = mCamera->fovy;
    mHudControlWindow.mHudCameraInfo.mWidth = mWidth;
    mHudControlWindow.mHudCameraInfo.mHeight = mHeight;
    if (mCameraView) {
      mHudControlWindow.mHudCameraInfo.mTextInfo = "Viewing from a mounted camera";
    } else if (mFocusedId) {
      mHudControlWindow.mHudCameraInfo.mTextInfo = "Focusing on an actor";
    } else {
      mHudControlWindow.mHudCameraInfo.mTextInfo = "Free";
    }

    {
      ImGui::NewFrame();
      ImGuizmo::BeginFrame();

      ImGui::SetNextWindowPos(ImVec2(0, 0));
      ImGui::SetNextWindowSize(ImVec2(mWidth, 60));
      ImGui::Begin("Menu");
      static bool control{}, object{}, gizmo{}, contact{};
      ImGui::Checkbox("Control", &control);
      ImGui::SameLine();
      ImGui::Checkbox("World", &object);
      ImGui::SameLine();
      ImGui::Checkbox("Gizmo", &gizmo);
      ImGui::SameLine();
      ImGui::Checkbox("Contact", &contact);
      ImGui::End();

      if (gizmo) {
        editTransform();
      } else {
        if (mGizmoBody.size()) {
          createGizmoVisual(nullptr);
        }
      }

      if (control) {
        mHudControlWindow.draw();
      }

      if (object) {
        mHudObjectWindow.draw(mScene, mSelectedId);
      }

      ImGui::Render();
    }

    if (mHudControlWindow.mHudCameraInfo.mUpdate) {
      mCamera->fovy = mHudControlWindow.mHudCameraInfo.mFovy;
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

    float frameRate = ImGui::GetIO().Framerate;

    if (mDefaultKeyPressBehavior) {
      if (mWindow->isKeyDown('q')) {
        mWindow->close();
      }
      float r = frameRate > 0 ? std::clamp(1 / frameRate, 0.f, 1.f) : 0.f;

      r *= mHudControlWindow.mHudControl.mMoveSpeed;
      if (mWindow->isKeyDown('w')) {
        viewFromCamera(0);
        focusActor(0);
        mFPSController->move(r, 0, 0);
      }
      if (mWindow->isKeyDown('s')) {
        viewFromCamera(0);
        focusActor(0);
        mFPSController->move(-r, 0, 0);
      }
      if (mWindow->isKeyDown('a')) {
        viewFromCamera(0);
        focusActor(0);
        mFPSController->move(0, r, 0);
      }
      if (mWindow->isKeyDown('d')) {
        viewFromCamera(0);
        focusActor(0);
        mFPSController->move(0, -r, 0);
      }
      if (mWindow->isKeyPressed('f')) {
        viewFromCamera(0);
        focusActor(0);
        focusActor(mSelectedId);
      }
    }

    if (mDefaultMouseClickBehavior) {
      float r = frameRate > 0 ? std::clamp(1 / frameRate, 0.f, 1.f) : 0.f;

      SActorBase *focusedActor{};
      if (mFocusedId) {
        focusedActor = mScene->findActorById(mFocusedId);
        if (!focusedActor) {
          focusedActor = mScene->findArticulationLinkById(mFocusedId);
        }
        if (!focusedActor) {
          focusActor(0);
          return;
        }

        auto p = focusedActor->getPose().p;
        mArcRotateController->setCenter(p.x, p.y, p.z);

        mArcRotateController->zoom(mWindow->getMouseWheelDelta().x);
      }

      if (mWindow->isMouseKeyDown(1)) {
        viewFromCamera(0);
        auto [x, y] = mWindow->getMouseDelta();
        float r1 = mHudControlWindow.mHudControl.mInvertX ? -r : r;
        float r2 = mHudControlWindow.mHudControl.mInvertY ? -r : r;
        r1 *= mHudControlWindow.mHudControl.mRotateSpeed;
        r2 *= mHudControlWindow.mHudControl.mRotateSpeed;

        if (mFocusedId) {
          mArcRotateController->rotateYawPitch(-r1 * x, r2 * y);
        } else {
          mFPSController->rotate(0, -r2 * y, -r1 * x);
        }
      }

      if (mWindow->isMouseKeyDown(0)) {
        auto [x, y] = mWindow->getMousePosition();
        auto pixel = mVulkanRenderer->getRenderTargets().segmentation->downloadPixel<uint32_t>(
            mRenderer->mContext->getPhysicalDevice(), mRenderer->mContext->getDevice(),
            mRenderer->mContext->getCommandPool(), mRenderer->mContext->getGraphicsQueue(),
            static_cast<int>(x), static_cast<int>(y));
        if (pixel.size()) {
          auto actorId = pixel[1];
          selectActor(actorId);
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
      case 3:
        mVulkanRenderer->switchToSegmentation();
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
        auto pose = actor->getPose();
        if (mHudObjectWindow.mHudActor.mShowCenterOfMass &&
            actor->getType() != EActorType::STATIC) {
          pose = pose * static_cast<SActorDynamicBase *>(actor)->getCMassLocalPose();
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

    if (mHudObjectWindow.mHudCamera.mRequestCameraView) {
      viewFromCamera(mHudObjectWindow.mHudCamera.mRequestIndex);
    }

    if (mCameraView) {
      uint32_t idx = mCameraView - 1;
      auto cameras = mScene->getMountedCameras();
      if (idx >= cameras.size()) {
        viewFromCamera(0);
      } else {
        auto pose = cameras[idx]->getPose();
        mCamera->position = {pose.p.x, pose.p.y, pose.p.z};
        mCamera->rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z};
        mCamera->fovy = cameras[idx]->getFovy();
        mCamera->near = cameras[idx]->getNear();
        mCamera->far = cameras[idx]->getFar();
      }
    }

  } while (mHudControlWindow.mHudControl.mPause);
}

void SapienVulkanController::selectActor(physx_id_t actorId) { mSelectedId = actorId; }

void SapienVulkanController::focusActor(physx_id_t actorId) {
  if (mFocusedId != actorId) {
    auto actor = mScene->findActorById(actorId);
    if (!actor) {
      actor = mScene->findArticulationLinkById(actorId);
    }
    if (!actor) {
      mFocusedId = 0;
      auto p = mCamera->position;
      auto yp = mArcRotateController->getYawPitch();
      mFPSController->setXYZ(p.x, p.y, p.z);
      mFPSController->setRPY(0, -yp.y, yp.x);
    } else if (mFocusedId == 0) {
      auto [x, y, z] = actor->getPose().p;

      auto p = mFPSController->getXYZ();
      auto rpy = mFPSController->getRPY();
      float r = glm::length(glm::vec3(x, y, z) - p);

      mArcRotateController->setCenter(x, y, z);
      mArcRotateController->setYawPitch(rpy.z, -rpy.y);
      mArcRotateController->setRadius(r);
      mFocusedId = actorId;
    }
  }
}

void SapienVulkanController::viewFromCamera(uint32_t camera) {
  if (camera) {
    focusActor(0);
  }
  mCameraView = camera;
}

void SapienVulkanController::pause(bool p) { mHudControlWindow.mHudControl.mPause = p; }

void SapienVulkanController::setFreeCameraPosition(float x, float y, float z) {
  focusActor(0);
  viewFromCamera(0);
  mFPSController->setXYZ(x, y, z);
}

void SapienVulkanController::setFreeCameraRotation(float yaw, float pitch, float roll) {
  focusActor(0);
  viewFromCamera(0);
  mFPSController->setRPY(roll, pitch, yaw);
}

void SapienVulkanController::close() { mWindow->close(); }

bool SapienVulkanController::keyPressed(char k) { return mWindow->isKeyPressed(k); }
bool SapienVulkanController::keyDown(char k) { return mWindow->isKeyDown(k); }
bool SapienVulkanController::mouseClick(int key) { return mWindow->isMouseKeyClicked(key); }
bool SapienVulkanController::mouseDown(int key) { return mWindow->isMouseKeyDown(key); }

std::tuple<int, int> SapienVulkanController::getMousePos() {
  auto [x, y] = mWindow->getMousePosition();
  return {x, y};
}

std::tuple<float, float> SapienVulkanController::getMouseDelta() {
  auto [x, y] = mWindow->getMouseDelta();
  return {x, y};
}

std::tuple<float, float> SapienVulkanController::getMouseWheelDelta() {
  auto [x, y] = mWindow->getMouseWheelDelta();
  return {x, y};
}

} // namespace Renderer
} // namespace sapien
#endif
#endif
