#ifdef _USE_VULKAN
#ifdef ON_SCREEN
#include "sapien_vulkan_controller.h"
#include "articulation/sapien_link.h"
#include "sapien_actor.h"
#include "sapien_actor_base.h"
#include "sapien_contact.h"
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

void SapienVulkanController::editGizmoTransform() {
  static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);
  static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
  static bool useSnap = false;
  static float snap[3] = {1.f, 1.f, 1.f};
  static float bounds[] = {-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};
  static float boundsSnap[] = {0.1f, 0.1f, 0.1f};
  static bool boundSizing = false;
  static bool boundSizingSnap = false;

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

#ifdef _USE_PINOCCHIO
    if (mAutoIKMode && actor &&
        (actor->getType() == EActorType::ARTICULATION_LINK ||
         actor->getType() == EActorType::KINEMATIC_ARTICULATION_LINK)) {
      computeIK(static_cast<SLinkBase *>(actor), pose);
    }
#endif

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

#ifdef _USE_PINOCCHIO
    if (actor && (actor->getType() == EActorType::ARTICULATION_LINK ||
                  actor->getType() == EActorType::KINEMATIC_ARTICULATION_LINK)) {
      ImGui::SameLine();
      if (ImGui::Button("Compute IK")) {
        computeIK(static_cast<SLinkBase *>(actor), pose);
      }
    }
#endif

    if (actor) {
      ImGui::Checkbox("Auto Gizmo mode", &mAutoGizmoMode);
      if (!mAutoGizmoMode) {
        ImGui::SameLine();
        if (ImGui::Button("Gizmo to actor")) {
          mGizmoTransform = PxTransform2Mat4(actor->getPose());
          createGizmoVisual(actor);
        }
      }
#ifdef _USE_PINOCCHIO
      if (mAutoGizmoMode) {
        ImGui::Checkbox("Auto IK mode", &mAutoIKMode);
      }
#endif
    }
#ifdef _USE_PINOCCHIO
    if (actor && (actor->getType() == EActorType::ARTICULATION_LINK ||
                  actor->getType() == EActorType::KINEMATIC_ARTICULATION_LINK)) {
      auto articulation = static_cast<SLinkBase *>(actor)->getArticulation();
      if (articulation == mIKArticulation) {
        if (mLastIKResult.size()) {
          ImGui::Text("IK %s", mLastIKSuccess ? "succeeded" : "failed");
          if (mLastIKSuccess) {
            ImGui::SameLine();
            if (ImGui::Button("Teleport##ik_teleport")) {
              std::vector<PxReal> q;
              for (uint32_t i = 0; i < mLastIKResult.size(); ++i) {
                q.push_back(mLastIKResult[i]);
              }
              articulation->setQpos(q);
            }
            if (actor->getType() == EActorType::ARTICULATION_LINK) {
              ImGui::SameLine();
              if (ImGui::Button("Drive##ik_drive")) {
                std::vector<PxReal> q;
                for (uint32_t i = 0; i < mLastIKResult.size(); ++i) {
                  q.push_back(mLastIKResult[i]);
                }
                static_cast<SArticulation*>(articulation)->setDriveTarget(q);
              }
            }
          }
        }
      }
    }
#endif
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

void SapienVulkanController::computeIK(SLinkBase *actor, PxTransform const &pose) {
  auto articulation = actor->getArticulation();
  auto idx = (actor)->getIndex();
  if (mIKArticulation != articulation) {
    mPinocchioModel = articulation->createPinocchioModel();
    mIKArticulation = articulation;
    createIKVisual(articulation);
  }
  auto qpos = articulation->getQpos();
  Eigen::VectorXd q2(qpos.size());
  for (uint32_t i = 0; i < qpos.size(); ++i) {
    q2[i] = qpos[i];
  }
  auto rootPose = articulation->getRootPose();
  auto [ikqpos, success, err] =
      mPinocchioModel->computeInverseKinematics(idx, rootPose.getInverse() * pose, q2, 1e-4, 100);
  mLastIKResult = ikqpos;
  mLastIKSuccess = success;
  spdlog::get("SAPIEN")->warn("Solving IK {}", success ? "succeeded" : "failed");
  mPinocchioModel->computeForwardKinematics(ikqpos);
  for (uint32_t i = 0; i < articulation->getBaseLinks().size(); ++i) {
    auto ikLinkPose = mPinocchioModel->getLinkPose(i);
    for (auto b : mIKBodies[i]) {
      b->update(rootPose * ikLinkPose);
    }
  }
}

void SapienVulkanController::editContactVisualization() {
  if (!mScene) {
    return;
  }
  ImGui::Begin("Contact");

  if (ImGui::Button("Make everything transparent")) {
    for (auto a : mScene->getAllActors()) {
      a->setDisplayVisibility(0.5);
    }
    for (auto a : mScene->getAllArticulations()) {
      for (auto b : a->getBaseLinks()) {
        b->setDisplayVisibility(0.5);
      }
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Make everything normal")) {
    for (auto a : mScene->getAllActors()) {
      a->setDisplayVisibility(1);
    }
    for (auto a : mScene->getAllArticulations()) {
      for (auto b : a->getBaseLinks()) {
        b->setDisplayVisibility(1);
      }
    }
  }

  auto actor = mScene->findActorById(mSelectedId);
  if (!actor) {
    actor = mScene->findArticulationLinkById(mSelectedId);
  }
  if (!actor) {
    ImGui::Text("No actor selected");
  } else {
    static bool showContacts{true}, showImpulses{true};
    static float scale = 0.1f;
    ImGui::Checkbox("Show contacts", &showContacts);
    if (showContacts) {
      ImGui::SliderFloat("Contact frame scale", &scale, 0.01f, 1.f);
    }
    static float impulseScale{1.f}, maxImpulse{1.f};
    ImGui::Checkbox("Show Impulses", &showImpulses);
    if (showImpulses) {
      ImGui::SliderFloat("Max impulse display length", &impulseScale, 0.1f, 10.f);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("The longest length of the impulse display");
      }
      ImGui::SliderFloat("Max impulse", &maxImpulse, 1e-4f, 1e4f, "%f", 10);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Impulse greater than this number will be displayed at maximum display length");
      }
    }

    mVulkanRenderer->clearSticks();
    bool hovered = false;

    uint32_t pair_count = 0;
    for (auto contact : mScene->getContacts()) {
      if (contact->actors[0] == actor || contact->actors[1] == actor) {
        uint32_t point_count = 0;
        for (auto point : contact->points) {
          ImGui::Selectable(
              ("Pair " + std::to_string(pair_count) + " Point " + std::to_string(point_count++))
                  .c_str());
          if (ImGui::IsItemHovered()) {
            hovered = true;
            {
              glm::mat4 s(1);
              s[0][0] = s[1][1] = s[2][2] = scale;

              // invert x axis if current actor is the first actor
              // this makes the x axis pointing outwards
              if (contact->actors[0] == actor) {
                s[0][0] *= -1;
              }

              glm::vec3 v = {point.normal.x, point.normal.y, point.normal.z};
              glm::vec3 t1 = glm::cross(v, glm::vec3(1, 0, 0));
              if (glm::length2(t1) < 0.01) {
                t1 = glm::cross(v, glm::vec3(0, 1, 0));
              }
              t1 = glm::normalize(t1);
              glm::vec3 t2 = glm::cross(v, t1);
              glm::mat3 m(v, t1, t2);
              glm::mat4 transform = glm::mat4(m) * s;
              transform[3][0] = point.position.x;
              transform[3][1] = point.position.y;
              transform[3][2] = point.position.z;
              transform[3][3] = 1.f;
              mVulkanRenderer->addAxes(transform);
            }
            {
              glm::mat4 s(1);
              glm::vec3 v = {point.impulse.x, point.impulse.y, point.impulse.z};
              float mag = glm::length(v);
              s[0][0] = s[1][1] = s[2][2] = impulseScale * std::min(1.f, mag / maxImpulse);

              // invert x axis if current actor is the first actor
              // this makes the x axis pointing outwards
              if (contact->actors[0] == actor) {
                s[0][0] *= -1;
              }

              if (mag != 0.f) {
                v = glm::normalize(v);
                glm::vec3 t1 = glm::cross(v, glm::vec3(1, 0, 0));
                if (glm::length2(t1) < 0.01) {
                  t1 = glm::cross(v, glm::vec3(0, 1, 0));
                }
                t1 = glm::normalize(t1);
                glm::vec3 t2 = glm::cross(v, t1);
                glm::mat3 m(v, t1, t2);
                glm::mat4 transform = glm::mat4(m) * s;
                transform[3][0] = point.position.x;
                transform[3][1] = point.position.y;
                transform[3][2] = point.position.z;
                transform[3][3] = 1.f;
                mVulkanRenderer->addStick(transform);
              }
            }
          }
          ImGui::Text("%s - %s", contact->actors[0]->getName().c_str(),
                      contact->actors[1]->getName().c_str());
          if (contact->starts) {
            ImGui::Text("Type: start");
          } else if (contact->persists) {
            ImGui::Text("Type: persist");
          } else if (contact->ends) {
            ImGui::Text("Type: ends");
          }
          ImGui::Text("Separation: %.4g", point.separation);
          ImGui::Text("Impulse: %.4g %.4g %.4g", point.impulse.x, point.impulse.y,
                      point.impulse.z);
          ImGui::NewLine();
        }
        pair_count++;
      }
    }

    if (!hovered) {
      for (auto contact : mScene->getContacts()) {
        if (contact->actors[0] == actor || contact->actors[1] == actor) {
          for (auto point : contact->points) {
            if (showContacts) {
              glm::mat4 s(1);
              s[0][0] = s[1][1] = s[2][2] = scale;

              // invert x axis if current actor is the first actor
              // this makes the x axis pointing outwards
              if (contact->actors[0] == actor) {
                s[0][0] *= -1;
              }

              glm::vec3 v = {point.normal.x, point.normal.y, point.normal.z};
              glm::vec3 t1 = glm::cross(v, glm::vec3(1, 0, 0));
              if (glm::length2(t1) < 0.01) {
                t1 = glm::cross(v, glm::vec3(0, 1, 0));
              }
              t1 = glm::normalize(t1);
              glm::vec3 t2 = glm::cross(v, t1);
              glm::mat3 m(v, t1, t2);
              glm::mat4 transform = glm::mat4(m) * s;
              transform[3][0] = point.position.x;
              transform[3][1] = point.position.y;
              transform[3][2] = point.position.z;
              transform[3][3] = 1.f;
              mVulkanRenderer->addAxes(transform);
            }
            if (showImpulses) {
              glm::mat4 s(1);
              glm::vec3 v = {point.impulse.x, point.impulse.y, point.impulse.z};
              float mag = glm::length(v);
              s[0][0] = s[1][1] = s[2][2] = impulseScale * std::min(1.f, mag / maxImpulse);

              // invert x axis if current actor is the first actor
              // this makes the x axis pointing outwards
              if (contact->actors[0] == actor) {
                s[0][0] *= -1;
              }

              if (mag != 0.f) {
                v = glm::normalize(v);
                glm::vec3 t1 = glm::cross(v, glm::vec3(1, 0, 0));
                if (glm::length2(t1) < 0.01) {
                  t1 = glm::cross(v, glm::vec3(0, 1, 0));
                }
                t1 = glm::normalize(t1);
                glm::vec3 t2 = glm::cross(v, t1);
                glm::mat3 m(v, t1, t2);
                glm::mat4 transform = glm::mat4(m) * s;
                transform[3][0] = point.position.x;
                transform[3][1] = point.position.y;
                transform[3][2] = point.position.z;
                transform[3][3] = 1.f;
                mVulkanRenderer->addStick(transform);
              }
            }
          }
        }
      }
    }
  }
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
    mGizmoBody.back()->setSegmentationId(actor->getId());
  }
}

void SapienVulkanController::createIKVisual(SArticulationBase *articulation) {
  if (!mScene) {
    return;
  }
  for (auto &l : mIKBodies) {
    for (auto b : l) {
      b->destroy();
    }
  }
  mIKBodies.clear();
  if (!articulation) {
    return;
  }
  for (auto l : articulation->getBaseLinks()) {
    mIKBodies.emplace_back();
    for (auto b : l->getRenderBodies()) {
      auto b2 = static_cast<SapienVulkanScene *>(mScene->getRendererScene())
                    ->cloneRigidbody(static_cast<SapienVulkanRigidbody *>(b));
      b2->setVisibility(0.5);
      b2->setUniqueId(0);
      b2->setSegmentationId(b->getSegmentationId());
      mIKBodies.back().push_back(b2);
    }
  }
}

SapienVulkanController::SapienVulkanController(SapienVulkanRenderer *renderer)
    : mRenderer(renderer), mWidth(0), mHeight(0) {
  mCamera = mRenderer->mContext->createCamera();
  svulkan::VulkanRendererConfig config;
  config.customTextureCount = 1;
  mVulkanRenderer = mRenderer->mContext->createVulkanRendererForEditor(config);
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
      ImGui::Checkbox("Control", &mControlWindow);
      ImGui::SameLine();
      ImGui::Checkbox("Object", &mObjectWindow);
      ImGui::SameLine();
      ImGui::Checkbox("Gizmo", &mGizmoWindow);
      ImGui::SameLine();
      ImGui::Checkbox("Contact", &mContactWindow);
      ImGui::End();

      if (mGizmoWindow) {
        editGizmoTransform();
      } else {
        if (mGizmoBody.size()) {
          createGizmoVisual(nullptr);
        }
      }

      if (mControlWindow) {
        mHudControlWindow.draw();
      }

      if (mObjectWindow) {
        mHudObjectWindow.draw(mScene, mSelectedId);
      }

      if (mContactWindow) {
        editContactVisualization();
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
        r1 = mHudControlWindow.mHudControl.mRotateSpeed * 0.001;
        r2 = mHudControlWindow.mHudControl.mRotateSpeed * 0.001;

        if (mFocusedId) {
          mArcRotateController->rotateYawPitch(-r1 * x, r2 * y);
        } else {
          mFPSController->rotate(0, -r2 * y, -r1 * x);
        }
      }

      if (mWindow->isMouseKeyDown(0)) {
        auto [x, y] = mWindow->getMousePosition();
#ifdef _USE_MACOSX
        int w1, h1, w2, h2;
        glfwGetWindowSize(mWindow->getWindow(), &w1, &h1);
        glfwGetFramebufferSize(mWindow->getWindow(), &w2, &h2);
        float factor = w2 / w1;
        x *= factor;
        y *= factor;
#endif
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
      case 4:
        mVulkanRenderer->switchToCustom();
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

    mVulkanRenderer->clearAxes();
    if (mSelectedId) {
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

  } while (mHudControlWindow.mHudControl.mPause && !mHudControlWindow.mHudControl.mStepped);
}

void SapienVulkanController::selectActor(physx_id_t actorId) {
  if (mSelectedId != actorId) {
    mSelectedId = actorId;

    // auto gizmo mode
    auto actor = mScene->findActorById(actorId);
    if (!actor) {
      actor = mScene->findArticulationLinkById(actorId);
    }
    if (actor) {
      if (mGizmoWindow && mAutoGizmoMode) {
        mGizmoTransform = PxTransform2Mat4(actor->getPose());
        createGizmoVisual(actor);
      }
    }
  }
}

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
