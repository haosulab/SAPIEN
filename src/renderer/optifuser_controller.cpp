#include "optifuser_controller.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_link.h"
#include "render_interface.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <spdlog/spdlog.h>
#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

constexpr int WINDOW_WIDTH = 1200, WINDOW_HEIGHT = 800;

enum RenderMode {
  LIGHTING,
  ALBEDO,
  NORMAL,
  DEPTH,
  SEGMENTATION,
  CUSTOM
#ifdef _USE_OPTIX
  ,
  PATHTRACER
#endif
};

namespace sapien {
namespace Renderer {

static int pickedId = 0;
static int pickedRenderId = 0;

OptifuserController::OptifuserController(OptifuserRenderer *renderer)
    : mRenderer(renderer), mFreeCameraController(mCamera), mArcCameraController(mCamera) {
  mCamera.aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;
  setCameraPosition(0, 0, 1);
  setCameraRotation(0, 0);
}
void OptifuserController::showWindow() { mRenderer->mContext->showWindow(); }
void OptifuserController::hideWindow() { mRenderer->mContext->hideWindow(); }

void OptifuserController::setCurrentScene(SScene *scene) { mScene = scene; }
void OptifuserController::focus(SActorBase *actor) {
  if (actor && !mCurrentFocus) {
    // none -> focus
    mArcCameraController.yaw = mFreeCameraController.yaw;
    mArcCameraController.pitch = mFreeCameraController.pitch;
    auto p = actor->getPose().p;
    mArcCameraController.center = {p.x, p.y, p.z};
    mArcCameraController.r = glm::length(
        glm::vec3(mCamera.position.x - p.x, mCamera.position.y - p.y, mCamera.position.z - p.z));
    actor->EventEmitter<EventActorPreDestroy>::registerListener(*this);
  } else if (!actor && mCurrentFocus) {
    // focus -> none
    mFreeCameraController.yaw = mArcCameraController.yaw;
    mFreeCameraController.pitch = mArcCameraController.pitch;
    auto &p = mArcCameraController.camera.position;
    mFreeCameraController.setPosition(p.x, p.y, p.z);
    mCurrentFocus->EventEmitter<EventActorPreDestroy>::unregisterListener(*this);
  } else if (actor && actor != mCurrentFocus) {
    // focus1 -> focus2
    mCurrentFocus->EventEmitter<EventActorPreDestroy>::unregisterListener(*this);
    actor->EventEmitter<EventActorPreDestroy>::registerListener(*this);
  } // none -> none
  mCurrentFocus = actor;
}

void OptifuserController::setCameraPosition(float x, float y, float z) {
  focus(nullptr);
  mFreeCameraController.setPosition(x, y, z);
}

void OptifuserController::setCameraRotation(float yaw, float pitch) {
  focus(nullptr);
  mFreeCameraController.yaw = yaw;
  mFreeCameraController.pitch = pitch;
  mFreeCameraController.update();
}

physx::PxTransform OptifuserController::getCameraPose() const {
  auto p = mCamera.position;
  auto q = mCamera.getRotation();
  return {{p.x, p.y, p.z}, {q.x, q.y, q.z, q.w}};
}

bool OptifuserController::shouldQuit() { return mShouldQuit; }

void OptifuserController::render() {

#ifdef _USE_OPTIX
  static Optifuser::OptixRenderer *pathTracer = nullptr;
#endif

  static int renderMode = 0;
  static float moveSpeed = 3.f;
  mRenderer->mContext->processEvents();
  float framerate = ImGui::GetIO().Framerate;

  float dt = 1.f * moveSpeed / framerate;

  if (Optifuser::getInput().getKeyState(GLFW_KEY_W)) {
    focus(nullptr);
    mFreeCameraController.moveForwardRight(dt, 0);
#ifdef _USE_OPTIX
    if (renderMode == PATHTRACER) {
      pathTracer->invalidateCamera();
    }
#endif
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_S)) {
    focus(nullptr);
    mFreeCameraController.moveForwardRight(-dt, 0);
#ifdef _USE_OPTIX
    if (renderMode == PATHTRACER) {
      pathTracer->invalidateCamera();
    }
#endif
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_A)) {
    focus(nullptr);
    mFreeCameraController.moveForwardRight(0, -dt);
#ifdef _USE_OPTIX
    if (renderMode == PATHTRACER) {
      pathTracer->invalidateCamera();
    }
#endif
  } else if (Optifuser::getInput().getKeyState(GLFW_KEY_D)) {
    focus(nullptr);
    mFreeCameraController.moveForwardRight(0, dt);
#ifdef _USE_OPTIX
    if (renderMode == PATHTRACER) {
      pathTracer->invalidateCamera();
    }
#endif
  }

  if (mCurrentFocus) {
    auto [x, y, z] = mCurrentFocus->getPose().p;

    double dx, dy;
    Optifuser::getInput().getWheelDelta(dx, dy);
    mArcCameraController.r += dy;
    if (mArcCameraController.r < 1) {
      mArcCameraController.r = 1;
    }
    mArcCameraController.setCenter(x, y, z);
  }

  if (Optifuser::getInput().getKeyDown(GLFW_KEY_Q)) {
    mShouldQuit = true;
  }

  mCamera.aspect = static_cast<float>(mRenderer->mContext->getWidth()) /
                   static_cast<float>(mRenderer->mContext->getHeight());

  mCamera.aspect = static_cast<float>(mRenderer->mContext->getWidth()) /
                   static_cast<float>(mRenderer->mContext->getHeight());

  static bool renderGui = true;
  if (Optifuser::getInput().getKeyDown(GLFW_KEY_E)) {
    renderGui = !renderGui;
  }
  if (Optifuser::getInput().getMouseButton(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
    double dx, dy;
    Optifuser::getInput().getCursorDelta(dx, dy);
    if (!mCurrentFocus) {
      mFreeCameraController.rotateYawPitch(-dx / 1000.f, -dy / 1000.f);
    } else {
      mArcCameraController.rotateYawPitch(-dx / 1000.f, -dy / 1000.f);
    }
#ifdef _USE_OPTIX
    if (renderMode == PATHTRACER) {
      pathTracer->invalidateCamera();
    }
#endif
  }

  OptifuserScene *currentScene = nullptr;
  if (mScene) {
    currentScene = static_cast<OptifuserScene *>(mScene->getRendererScene());
  }

  if (currentScene) {
    mRenderer->mContext->renderer.renderScene(*currentScene->getScene(), mCamera);

    if (renderMode == SEGMENTATION) {
      mRenderer->mContext->renderer.displaySegmentation();
    } else if (renderMode == CUSTOM) {
      mRenderer->mContext->renderer.displayUserTexture();
#ifdef _USE_OPTIX
    } else if (renderMode == PATHTRACER) {
      // path tracer
      pathTracer->numRays = 4;
      pathTracer->max_iterations = 100000;
      pathTracer->renderScene(*currentScene->getScene(), mCamera);
      pathTracer->display();
#endif
    } else {
      mRenderer->mContext->renderer.displayLighting();
    }
  }

  if (Optifuser::getInput().getMouseDown(GLFW_MOUSE_BUTTON_LEFT)) {
    int x, y;
    Optifuser::getInput().getCursor(x, y);
    pickedId = mRenderer->mContext->renderer.pickSegmentationId(x, y);
    mGuiModel.linkId = pickedId;

    pickedRenderId = 0;
    if (pickedId) {
      pickedRenderId = mRenderer->mContext->renderer.pickObjectId(x, y);
    }

    SActorBase *actor = mScene->findActorById(mGuiModel.linkId);
    if (!actor) {
      actor = mScene->findArticulationLinkById(mGuiModel.linkId);
    }

    if (actor != mCurrentSelection) {
      if (mCurrentSelection) {
        mCurrentSelection->EventEmitter<EventActorPreDestroy>::unregisterListener(*this);
      }
      if (actor) {
        actor->EventEmitter<EventActorPreDestroy>::registerListener(*this);
      }
      mCurrentSelection = actor;
    }
  }

  if (mCurrentSelection) {
    SActorBase *actor = mCurrentSelection;
    SArticulationBase *articulation = nullptr;
    auto t = actor->getType();
    if (t == EActorType::ARTICULATION_LINK || t == EActorType::KINEMATIC_ARTICULATION_LINK) {
      articulation = static_cast<SLinkBase *>(actor)->getArticulation();
    }

    mGuiModel.linkModel.name = actor->getName();
    mGuiModel.linkModel.transform = actor->getPxActor()->getGlobalPose();
    mGuiModel.linkModel.col1 = actor->getCollisionGroup1();
    mGuiModel.linkModel.col2 = actor->getCollisionGroup2();
    mGuiModel.linkModel.col3 = actor->getCollisionGroup3();
    mGuiModel.linkModel.renderCollision = actor->getRenderMode() == 1;
    mGuiModel.articulationId = 0;

    if (articulation) {
      mGuiModel.articulationId = 1;
      mGuiModel.articulationModel.name = articulation->getName();

      mGuiModel.articulationModel.jointModel.resize(articulation->dof());
      uint32_t n = 0;
      auto qpos = articulation->getQpos();
      for (auto j : articulation->getBaseJoints()) {
        auto limits = j->getLimits();
        for (uint32_t i = 0; i < j->getDof(); ++i) {
          mGuiModel.articulationModel.jointModel[n].name = j->getName();
          mGuiModel.articulationModel.jointModel[n].limits = limits[i];
          mGuiModel.articulationModel.jointModel[n].value = qpos[n];
          ++n;
        }
      }
    }

    switch (actor->getType()) {
    case EActorType::DYNAMIC:
    case EActorType::KINEMATIC:
    case EActorType::ARTICULATION_LINK:
      if (mGuiModel.linkModel.showCenterOfMass) {
        mGuiModel.linkModel.cmassPose =
            actor->getPxActor()->getGlobalPose() *
            static_cast<PxRigidBody *>(actor->getPxActor())->getCMassLocalPose();
      }
      break;
    default:
      break;
    }

    if (Optifuser::getInput().getKeyDown(GLFW_KEY_F)) {
      focus(actor);
    }

    currentScene->getScene()->clearAxes();

    auto &pos = mGuiModel.linkModel.showCenterOfMass ? mGuiModel.linkModel.cmassPose.p
                                                     : mGuiModel.linkModel.transform.p;
    auto &quat = mGuiModel.linkModel.showCenterOfMass ? mGuiModel.linkModel.cmassPose.q
                                                      : mGuiModel.linkModel.transform.q;
    currentScene->getScene()->addAxes({pos.x, pos.y, pos.z}, {quat.w, quat.x, quat.y, quat.z});
  }

  // if (mGuiModel.linkId) {
  //   SActorBase *actor = mScene->findActorById(mGuiModel.linkId);
  //   SLinkBase *link = mScene->findArticulationLinkById(mGuiModel.linkId);
  //   if (actor) {
  //     mCurrentSelection = actor;
  //     actor->registerListener(*this);

  //     mGuiModel.linkModel.name = actor->getName();
  //     mGuiModel.linkModel.transform = actor->getPxActor()->getGlobalPose();
  //     mGuiModel.linkModel.col1 = actor->getCollisionGroup1();
  //     mGuiModel.linkModel.col2 = actor->getCollisionGroup2();
  //     mGuiModel.linkModel.col3 = actor->getCollisionGroup3();
  //     mGuiModel.linkModel.renderCollision = actor->getRenderMode() == 1;
  //     mGuiModel.articulationId = 0;
  //   } else if (link) {
  //     mCurrentSelection = link;
  //     actor = link;
  //     actor->registerListener(*this);

  //     mGuiModel.linkModel.name = link->getName();
  //     mGuiModel.linkModel.transform = link->getPxActor()->getGlobalPose();
  //     mGuiModel.linkModel.col1 = link->getCollisionGroup1();
  //     mGuiModel.linkModel.col2 = link->getCollisionGroup2();
  //     mGuiModel.linkModel.col3 = link->getCollisionGroup3();
  //     mGuiModel.linkModel.renderCollision = link->getRenderMode() == 1;
  //     auto articulation = link->getArticulation();
  //     mGuiModel.articulationId = 1;
  //     mGuiModel.articulationModel.name = articulation->getName();

  //     mGuiModel.articulationModel.jointModel.resize(articulation->dof());
  //     uint32_t n = 0;
  //     auto qpos = articulation->getQpos();
  //     for (auto j : articulation->getBaseJoints()) {
  //       auto limits = j->getLimits();
  //       for (uint32_t i = 0; i < j->getDof(); ++i) {
  //         mGuiModel.articulationModel.jointModel[n].name = j->getName();
  //         mGuiModel.articulationModel.jointModel[n].limits = limits[i];
  //         mGuiModel.articulationModel.jointModel[n].value = qpos[n];
  //         ++n;
  //       }
  //     }
  //   } else {
  //     spdlog::error(
  //         "User picked an unregistered object {}. There is probably some implementation error!",
  //         mGuiModel.linkId);
  //   }

  //   if (actor) {
  //     switch (actor->getType()) {
  //     case EActorType::DYNAMIC:
  //     case EActorType::KINEMATIC:
  //     case EActorType::ARTICULATION_LINK:
  //       if (mGuiModel.linkModel.showCenterOfMass) {
  //         mGuiModel.linkModel.cmassPose =
  //             link->getPxActor()->getGlobalPose() *
  //             static_cast<PxRigidBody *>(link->getPxActor())->getCMassLocalPose();
  //       }
  //       break;
  //     default:
  //       break;
  //     }
  //   }

  //   if (Optifuser::getInput().getKeyDown(GLFW_KEY_F)) {
  //     focus(actor);
  //   }

  //   currentScene->getScene()->clearAxes();

  //   auto &pos = mGuiModel.linkModel.showCenterOfMass ? mGuiModel.linkModel.cmassPose.p
  //                                                    : mGuiModel.linkModel.transform.p;
  //   auto &quat = mGuiModel.linkModel.showCenterOfMass ? mGuiModel.linkModel.cmassPose.q
  //                                                     : mGuiModel.linkModel.transform.q;
  //   currentScene->getScene()->addAxes({pos.x, pos.y, pos.z}, {quat.w, quat.x, quat.y, quat.z});
  // }

  static const uint32_t imguiWindowSize = 300;
  static int camIndex = -1;
  if (renderGui) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mRenderer->mContext->getHeight()));

    ImGui::Begin("Render Options");
    {
      if (ImGui::CollapsingHeader("Render Mode", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::RadioButton("Lighting", &renderMode, RenderMode::LIGHTING)) {
          mRenderer->mContext->renderer.setDeferredShader(mRenderer->mGlslDir + "/deferred.vsh",
                                                          mRenderer->mGlslDir + "/deferred.fsh");
        };
        if (ImGui::RadioButton("Albedo", &renderMode, RenderMode::ALBEDO)) {
          mRenderer->mContext->renderer.setDeferredShader(
              mRenderer->mGlslDir + "/deferred.vsh", mRenderer->mGlslDir + "/deferred_albedo.fsh");
        }
        if (ImGui::RadioButton("Normal", &renderMode, RenderMode::NORMAL)) {
          mRenderer->mContext->renderer.setDeferredShader(
              mRenderer->mGlslDir + "/deferred.vsh", mRenderer->mGlslDir + "/deferred_normal.fsh");
        }
        if (ImGui::RadioButton("Depth", &renderMode, RenderMode::DEPTH)) {
          mRenderer->mContext->renderer.setDeferredShader(
              mRenderer->mGlslDir + "/deferred.vsh", mRenderer->mGlslDir + "/deferred_depth.fsh");
        }
        if (ImGui::RadioButton("Segmentation", &renderMode, RenderMode::SEGMENTATION)) {
          mRenderer->mContext->renderer.setGBufferShader(mRenderer->mGlslDir + "/gbuffer.vsh",
                                                         mRenderer->mGlslDir +
                                                             "/gbuffer_segmentation.fsh");
        }
        if (ImGui::RadioButton("Custom", &renderMode, RenderMode::CUSTOM)) {
          mRenderer->mContext->renderer.setGBufferShader(mRenderer->mGlslDir + "/gbuffer.vsh",
                                                         mRenderer->mGlslDir +
                                                             "/gbuffer_segmentation.fsh");
        }
#ifdef _USE_OPTIX
        if (ImGui::RadioButton("PathTracer", &renderMode, RenderMode::PATHTRACER)) {
          if (pathTracer) {
            delete pathTracer;
          }
          pathTracer = new Optifuser::OptixRenderer();
          pathTracer->setBlackBackground();
          pathTracer->init(mRenderer->mContext->getWidth(), mRenderer->mContext->getHeight());
        } else {
        }
#endif
      }

#ifdef _USE_OPTIX
      if (renderMode == PATHTRACER) {
        glEnable(GL_FRAMEBUFFER_SRGB);
      } else {
        glDisable(GL_FRAMEBUFFER_SRGB);
      }
#endif

      if (ImGui::CollapsingHeader("Main Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Position");
        ImGui::Text("%-4.3f %-4.3f %-4.3f", mCamera.position.x, mCamera.position.y,
                    mCamera.position.z);
        ImGui::Text("Forward");
        auto forward = mCamera.getRotation() * glm::vec3(0, 0, -1);
        ImGui::Text("%-4.3f %-4.3f %-4.3f", forward.x, forward.y, forward.z);
        ImGui::Text("Fov");
        ImGui::SliderAngle("##fov(y)", &mCamera.fovy, 1.f, 90.f);
        ImGui::Text("Move speed");
        ImGui::SliderFloat("##speed", &moveSpeed, 1.f, 10.f);
        ImGui::Text("Width: %d", mRenderer->mContext->getWidth());
        ImGui::SameLine();
        ImGui::Text("Height: %d", mRenderer->mContext->getHeight());
        ImGui::SameLine();
        ImGui::Text("Aspect: %.2f", mCamera.aspect);
        ImGui::Text("Picked link id: %d", pickedId);
        ImGui::Text("Picked render id: %d", pickedRenderId);
      }

      if (ImGui::CollapsingHeader("Mounted Cameras")) {
        ImGui::RadioButton("None##camera", &camIndex, -1);

        if (currentScene) {
          auto cameras = currentScene->getCameras();
          for (uint32_t i = 0; i < cameras.size(); ++i) {
            ImGui::RadioButton((cameras[i]->getName() + "##camera" + std::to_string(i)).c_str(),
                               &camIndex, i);
          }

          // handle camera deletion
          if (camIndex >= static_cast<int>(cameras.size())) {
            camIndex = -1;
          }

          if (camIndex >= 0) {
            uint32_t width = cameras[camIndex]->getWidth();
            uint32_t height = cameras[camIndex]->getHeight();
            cameras[camIndex]->takePicture();
            ImGui::Image(
                reinterpret_cast<ImTextureID>(static_cast<OptifuserCamera *>(cameras[camIndex])
                                                  ->mRenderContext->renderer.outputtex),
                ImVec2(imguiWindowSize, imguiWindowSize / static_cast<float>(width) * height),
                ImVec2(0, 1), ImVec2(1, 0));
          }
        }
      }

      if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Frame Time: %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
      }
    }
    ImGui::End();

    if (mGuiModel.linkId) {
      ImGui::SetNextWindowPos(ImVec2(mRenderer->mContext->getWidth() - imguiWindowSize, 0));
      ImGui::SetNextWindowSize(ImVec2(imguiWindowSize, mRenderer->mContext->getHeight()));
      ImGui::Begin("Selected Object");
      {
        if (ImGui::CollapsingHeader("Actor", ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text("name: %s", mGuiModel.linkModel.name.c_str());
          ImGui::Text("col1: #%08x, col2: #%08x", mGuiModel.linkModel.col1,
                      mGuiModel.linkModel.col2);
          ImGui::Text("col3: #%08x", mGuiModel.linkModel.col3);

          // toggle collision shape
          if (ImGui::Checkbox("Collision Shape", &mGuiModel.linkModel.renderCollision)) {
            SActorBase *link = mScene->findActorById(mGuiModel.linkId);
            if (!link) {
              link = mScene->findArticulationLinkById(mGuiModel.linkId);
            }
            if (link) {
              link->setRenderMode(mGuiModel.linkModel.renderCollision);
            }
          }

          // toggle center of mass
          if (ImGui::Checkbox("Center of Mass", &mGuiModel.linkModel.showCenterOfMass)) {
            SActorBase *link = mScene->findActorById(mGuiModel.linkId);
            if (!link) {
              link = mScene->findArticulationLinkById(mGuiModel.linkId);
            }
          }
        }
      }
      if (mGuiModel.articulationId) {
        if (ImGui::CollapsingHeader("Articulation", ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text("name: %s", mGuiModel.articulationModel.name.c_str());
          ImGui::Text(" dof: %ld", mGuiModel.articulationModel.jointModel.size());

          int i = 0;
          for (auto &joint : mGuiModel.articulationModel.jointModel) {
            ImGui::Text("%s", joint.name.c_str());
            if (ImGui::SliderFloat(("##" + std::to_string(++i)).c_str(), &joint.value,
                                   std::max(joint.limits[0], -10.f),
                                   std::min(joint.limits[1], 10.f))) {
              std::vector<PxReal> v;
              SLinkBase *link = mScene->findArticulationLinkById(mGuiModel.linkId);
              auto articulation = link->getArticulation();
              for (auto j : mGuiModel.articulationModel.jointModel) {
                v.push_back(j.value);
              }
              articulation->setQpos(v);
            }
          }
        }
      }
      ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }
  mRenderer->mContext->swapBuffers();
}

void OptifuserController::onEvent(EventActorPreDestroy &e) {
  if (e.actor == mCurrentFocus) {
    focus(nullptr);
  }
  if (e.actor == mCurrentSelection) {
    mGuiModel = {};
    mCurrentSelection = nullptr;
  }
}

} // namespace Renderer
} // namespace sapien
