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
    : mRenderer(renderer), mCameraMode(0),
      mCamera(std::make_unique<Optifuser::PerspectiveCameraSpec>()),
      mFreeCameraController(mCamera.get()), mArcCameraController(mCamera.get()) {
  mCamera->aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;
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
    mArcCameraController.r = glm::length(glm::vec3(
        mCamera->position.x - p.x, mCamera->position.y - p.y, mCamera->position.z - p.z));
    actor->EventEmitter<EventActorPreDestroy>::registerListener(*this);
  } else if (!actor && mCurrentFocus) {
    // focus -> none
    mFreeCameraController.yaw = mArcCameraController.yaw;
    mFreeCameraController.pitch = mArcCameraController.pitch;
    auto &p = mArcCameraController.camera->position;
    mFreeCameraController.setPosition(p.x, p.y, p.z);
    if (mCurrentSelection != mCurrentFocus) {
      mCurrentFocus->EventEmitter<EventActorPreDestroy>::unregisterListener(*this);
    }
  } else if (actor && actor != mCurrentFocus) {
    // focus1 -> focus2
    if (mCurrentSelection != mCurrentFocus) {
      mCurrentFocus->EventEmitter<EventActorPreDestroy>::unregisterListener(*this);
    }
    actor->EventEmitter<EventActorPreDestroy>::registerListener(*this);
  } // none -> none
  mCurrentFocus = actor;
}

void OptifuserController::select(SActorBase *actor) {
  if (actor != mCurrentSelection) {
    if (mCurrentSelection) {
      for (auto b : mCurrentSelection->getRenderBodies()) {
        b->setRenderMode(0);
      }
    }
    if (mCurrentSelection && mCurrentSelection != mCurrentFocus) {
      mCurrentSelection->EventEmitter<EventActorPreDestroy>::unregisterListener(*this);
    }
    if (actor) {
      actor->EventEmitter<EventActorPreDestroy>::registerListener(*this);
      for (auto b : actor->getRenderBodies()) {
        b->setRenderMode(2);
      }
    }
    mCurrentSelection = actor;
  }
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

void OptifuserController::setCameraOrthographic(bool ortho) {
  if (ortho) {
    mCameraMode = 1;
    auto cam = std::make_unique<Optifuser::OrthographicCameraSpec>();
    cam->name = mCamera->name;
    cam->aspect = mCamera->aspect;
    cam->position = mCamera->position;
    cam->setRotation(mCamera->getRotation());
    cam->near = mCamera->near;
    cam->far = mCamera->far;
    cam->scaling = 1.f;
    mCamera = std::move(cam);
  } else {
    mCameraMode = 0;
    auto cam = std::make_unique<Optifuser::PerspectiveCameraSpec>();
    cam->name = mCamera->name;
    cam->aspect = mCamera->aspect;
    cam->position = mCamera->position;
    cam->setRotation(mCamera->getRotation());
    cam->near = mCamera->near;
    cam->far = mCamera->far;
    cam->fovy = glm::radians(35.f);
    mCamera = std::move(cam);
  }
  mFreeCameraController.changeCamera(mCamera.get());
  mArcCameraController.changeCamera(mCamera.get());
}

physx::PxTransform OptifuserController::getCameraPose() const {
  auto p = mCamera->position;
  auto q = mCamera->getRotation();
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

#ifdef _USE_OPTIX
    if ((dx != 0 || dy != 0) && renderMode == PATHTRACER) {
      pathTracer->invalidateCamera();
    }
#endif
  }

  if (Optifuser::getInput().getKeyDown(GLFW_KEY_Q)) {
    mShouldQuit = true;
  }

  mCamera->aspect = static_cast<float>(mRenderer->mContext->getWidth()) /
                    static_cast<float>(mRenderer->mContext->getHeight());

  mCamera->aspect = static_cast<float>(mRenderer->mContext->getWidth()) /
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
    mRenderer->mContext->renderer.renderScene(*currentScene->getScene(), *mCamera);

    if (renderMode == LIGHTING) {
      mRenderer->mContext->renderer.displayLighting();
    } else if (renderMode == SEGMENTATION) {
      mRenderer->mContext->renderer.displaySegmentation();
    } else if (renderMode == CUSTOM) {
      mRenderer->mContext->renderer.displayUserTexture();
#ifdef _USE_OPTIX
    } else if (renderMode == PATHTRACER) {
      // path tracer
      pathTracer->numRays = 4;
      pathTracer->max_iterations = 100000;
      pathTracer->renderScene(*currentScene->getScene(), *mCamera);
      pathTracer->display();
#endif
    } else {
      mRenderer->mContext->renderer.display();
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
    select(actor);
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

  static const uint32_t imguiWindowSize = 300;
  static int camIndex = -1;
  int changeShader = 0;
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
        };
        if (ImGui::RadioButton("Albedo", &renderMode, RenderMode::ALBEDO)) {
          changeShader = 1;
        }
        if (ImGui::RadioButton("Normal", &renderMode, RenderMode::NORMAL)) {
          changeShader = 1;
        }
        if (ImGui::RadioButton("Depth", &renderMode, RenderMode::DEPTH)) {
          changeShader = 1;
        }
        if (ImGui::RadioButton("Segmentation", &renderMode, RenderMode::SEGMENTATION)) {
        }
        if (ImGui::RadioButton("Custom", &renderMode, RenderMode::CUSTOM)) {
        }
#ifdef _USE_OPTIX
        if (ImGui::RadioButton("PathTracer", &renderMode, RenderMode::PATHTRACER)) {
          if (pathTracer) {
            delete pathTracer;
          }
          pathTracer = new Optifuser::OptixRenderer(OptifuserRenderer::gPtxDir);
          pathTracer->setBlackBackground();
          pathTracer->init(mRenderer->mContext->getWidth(), mRenderer->mContext->getHeight());
          pathTracer->enableDenoiser();
        } else {
        }
#endif
      }

      if (ImGui::CollapsingHeader("Main Camera", ImGuiTreeNodeFlags_DefaultOpen)) {

        static bool ortho;
        if (ImGui::Checkbox("Orthographic", &ortho)) {
          if (ortho) {
            setCameraOrthographic(true);
          } else {
            setCameraOrthographic(false);
          }
        }

        ImGui::Text("Position");
        ImGui::Text("%-4.3f %-4.3f %-4.3f", mCamera->position.x, mCamera->position.y,
                    mCamera->position.z);
        ImGui::Text("Forward");
        auto forward = mCamera->getRotation() * glm::vec3(0, 0, -1);
        ImGui::Text("%-4.3f %-4.3f %-4.3f", forward.x, forward.y, forward.z);

        if (mCameraMode == 0) {
          ImGui::Text("Fov");
          auto fovy = &static_cast<Optifuser::PerspectiveCameraSpec *>(mCamera.get())->fovy;
          ImGui::SliderAngle("##fov(y)", fovy, 1.f, 90.f);
        } else {
          ImGui::Text("Scaling");
          auto scaling = &static_cast<Optifuser::OrthographicCameraSpec *>(mCamera.get())->scaling;
          ImGui::SliderFloat("##scaling", scaling, 0.1f, 10.f);
        }
        ImGui::Text("Move speed");
        ImGui::SliderFloat("##speed", &moveSpeed, 1.f, 10.f);
        ImGui::Text("Width: %d", mRenderer->mContext->getWidth());
        ImGui::SameLine();
        ImGui::Text("Height: %d", mRenderer->mContext->getHeight());
        ImGui::SameLine();
        ImGui::Text("Aspect: %.2f", mCamera->aspect);
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
                                                  ->mRenderContext->renderer.lightingtex),
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

          ImGui::Text("Position: %.2f %.2f %.2f", mGuiModel.linkModel.transform.p.x,
                      mGuiModel.linkModel.transform.p.y, mGuiModel.linkModel.transform.p.z);

          glm::vec3 angles = glm::eulerAngles(glm::quat(mGuiModel.linkModel.transform.q.w,
                                                        mGuiModel.linkModel.transform.q.x,
                                                        mGuiModel.linkModel.transform.q.y,
                                                        mGuiModel.linkModel.transform.q.z)) /
                             glm::pi<float>() * 180.f;
          ImGui::Text("Euler (degree): %.2f %.2f %.2f", angles.x, angles.y, angles.z);

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
      if (ImGui::CollapsingHeader("Actor Details")) {
        SActorBase *link = mScene->findActorById(mGuiModel.linkId);
        if (!link) {
          link = mScene->findArticulationLinkById(mGuiModel.linkId);
        }
        if (link) {
          auto actor = link->getPxActor();
          std::vector<PxShape *> shapes(actor->getNbShapes());
          actor->getShapes(shapes.data(), shapes.size());
          int primitives = 0;
          int meshes = 0;
          PxReal minDynamicFriction = 100;
          PxReal maxDynamicFriction = -1;
          PxReal minStaticFriction = 100;
          PxReal maxStaticFriction = -1;
          PxReal minRestitution = 100;
          PxReal maxRestitution = -1;
          for (auto s : shapes) {
            if (s->getGeometryType() == PxGeometryType::eCONVEXMESH) {
              meshes += 1;
            } else {
              primitives += 1;
            }
            std::vector<PxMaterial *> mats(s->getNbMaterials());
            s->getMaterials(mats.data(), s->getNbMaterials());
            for (auto m : mats) {
              PxReal sf = m->getStaticFriction();
              minStaticFriction = std::min(minStaticFriction, sf);
              maxStaticFriction = std::max(maxStaticFriction, sf);
              PxReal df = m->getDynamicFriction();
              minDynamicFriction = std::min(minDynamicFriction, df);
              maxDynamicFriction = std::max(maxDynamicFriction, df);
              PxReal r = m->getRestitution();
              minRestitution = std::min(minRestitution, r);
              maxRestitution = std::max(maxRestitution, r);
            }
          }
          ImGui::Text("Primitive Count: %d", primitives);
          ImGui::Text("Convex Mesh Count: %d", meshes);
          if (maxStaticFriction >= 0) {
            ImGui::Text("Static friction: %.2f - %.2f", minStaticFriction, maxStaticFriction);
            ImGui::Text("Dynamic friction: %.2f - %.2f", minDynamicFriction, maxDynamicFriction);
            ImGui::Text("Restitution : %.2f - %.2f", minRestitution, maxRestitution);
          } else {
            ImGui::Text("No Physical Material");
          }
        }
      }

      if (mGuiModel.articulationId) {
        if (ImGui::CollapsingHeader("Articulation", ImGuiTreeNodeFlags_DefaultOpen)) {
          SLinkBase *link = mScene->findArticulationLinkById(mGuiModel.linkId);
          auto articulation = link->getArticulation();

          ImGui::Text("name: %s", mGuiModel.articulationModel.name.c_str());
          ImGui::Text(" dof: %ld", mGuiModel.articulationModel.jointModel.size());
          if (articulation->getType() == EArticulationType::DYNAMIC) {
            ImGui::Text("type: Dynamic");
          } else {
            ImGui::Text("type: Kinematic");
          }

          static bool articulationDetails;
          ImGui::Checkbox("Details##ArticulationDetails", &articulationDetails);

          std::vector<SJoint *> joints;
          if (articulation->getType() == EArticulationType::DYNAMIC) {
            auto a = static_cast<sapien::SArticulation *>(articulation);
            auto js = a->getSJoints();
            for (auto j : js) {
              if (j->getDof()) {
                joints.push_back(j);
              }
            }
          }

          int i = 0;
          for (auto &joint : mGuiModel.articulationModel.jointModel) {
            ImGui::Text("%s", joint.name.c_str());
            if (ImGui::SliderFloat(("##" + std::to_string(i)).c_str(), &joint.value,
                                   std::max(joint.limits[0], -10.f),
                                   std::min(joint.limits[1], 10.f))) {
              std::vector<PxReal> v;
              for (auto j : mGuiModel.articulationModel.jointModel) {
                v.push_back(j.value);
              }
              articulation->setQpos(v);
            }
            if (articulationDetails && joints.size()) {
              auto j = joints[i];
              float friction = j->getFriction();
              float stiffness = j->getDriveStiffness();
              float damping = j->getDriveDamping();
              float maxForce = j->getDriveForceLimit();
              float target = j->getDriveTarget();
              float vtarget = j->getDriveVelocityTarget();
              ImGui::Text("Friction: %.2f", friction);
              ImGui::Text("Damping: %.2f", damping);
              ImGui::Text("Stiffness: %.2f", stiffness);
              if (maxForce > 1e6) {
                ImGui::Text("Max Force: >1e6");
              } else {
                ImGui::Text("Max Force: %.2f", maxForce);
              }
              if (stiffness > 0) {
                ImGui::Text("Drive Position Target: %.2f", target);
                ImGui::Text("Drive Velocity Target: %.2f", vtarget);
              }
            }

            ++i;
          }
        }
      }
      ImGui::End();
    }

    {
      auto err = glGetError();
      if (err != GL_NO_ERROR) {
        spdlog::get("SAPIEN")->error("Error0 {:x}", err);
        throw "";
      }
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    {
      auto err = glGetError();
      if (err != GL_NO_ERROR) {
        spdlog::get("SAPIEN")->error("Error1 {:x}", err);
        throw "";
      }
    }
  }

  mRenderer->mContext->swapBuffers();

  if (changeShader) {
    if (renderMode == ALBEDO) {
      mRenderer->mContext->renderer.setDisplayShader(mRenderer->mGlslDir + "/display.vsh",
                                                     mRenderer->mGlslDir + "/display_albedo.fsh");
    } else if (renderMode == NORMAL) {
      mRenderer->mContext->renderer.setDisplayShader(mRenderer->mGlslDir + "/display.vsh",
                                                     mRenderer->mGlslDir + "/display_normal.fsh");
    } else if (renderMode == DEPTH) {
      mRenderer->mContext->renderer.setDisplayShader(mRenderer->mGlslDir + "/display.vsh",
                                                     mRenderer->mGlslDir + "/display_depth.fsh");
    }
  }
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
