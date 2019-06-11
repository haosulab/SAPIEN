#include <PxPhysicsAPI.h>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxDefaultErrorCallback.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <extensions/PxExtensionsAPI.h>
#include <extensions/PxShapeExt.h>
#include <extensions/PxSimpleFactory.h>
#include <foundation/PxMat33.h>
#include <iostream>
#include <object.h>
#include <optifuser.h>
#include <vector>

using namespace physx;

constexpr int WINDOW_WIDTH = 1024, WINDOW_HEIGHT = 768;

static PxPhysics *gPhysicsSDK = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader =
    PxDefaultSimulationFilterShader;
static PxFoundation *gFoundation = NULL;

PxScene *gScene = NULL;
PxReal myTimestep = 1.0f / 600.0f;
std::vector<PxRigidActor *> boxes;

std::shared_ptr<Optifuser::Scene> gSceneRender = nullptr;

std::map<PxRigidActor *, std::shared_ptr<Optifuser::Object>> gActorRegistry;
void AddActorForRendering(std::shared_ptr<Optifuser::Scene> scene, PxRigidActor *actor, PxGeometryType::Enum type) {
  switch (type) {
    case PxGeometryType::eBOX: {
      auto obj = Optifuser::NewCube();
      obj->material.kd = {1, 1, 1};
      scene->addObject(obj);
      gActorRegistry[actor] = obj;
      break;
    }
      // TODO: fix plane orientation
    case PxGeometryType::ePLANE: {
      auto obj = Optifuser::NewYZPlane();
      obj->scale = {10, 10, 10};
      obj->material.kd = {1, 1, 1};
      scene->addObject(obj);
      gActorRegistry[actor] = obj;
      break;
    }
    default:
      break;
  }
}

void RenderActor(PxRigidActor *actor) {
  PxTransform pT = actor->getGlobalPose();
  gActorRegistry[actor]->position = {pT.p.x, pT.p.y, pT.p.z};
  gActorRegistry[actor]->rotation = {pT.q.w, pT.q.x, pT.q.y, pT.q.z};
}

void RenderActors() {
  for (auto actor : gActorRegistry) {
    RenderActor(actor.first);
  }
}

void StepPhysX() {
  for (int i = 0; i < 10; ++i) {
    gScene->simulate(myTimestep);
    while (!gScene->fetchResults()) {
      // do something useful
    }
  }
}

void InitializePhysX() {

  gFoundation = PxCreateFoundation(
      PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  // Creating instance of PhysX SDK
  gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation,
                                PxTolerancesScale(), true);

  if (gPhysicsSDK == NULL) {
    std::cerr << "Error creating PhysX3 device." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  if (!PxInitExtensions(*gPhysicsSDK, nullptr)) {
    std::cerr << "PxInitExtensions failed!" << std::endl;
  }

  // Create the scene
  PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

  if (!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher *mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if (!mCpuDispatcher)
      std::cerr << "PxDefaultCpuDispatcherCreate failed!" << std::endl;
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if (!sceneDesc.filterShader)
    sceneDesc.filterShader = gDefaultFilterShader;

  gScene = gPhysicsSDK->createScene(sceneDesc);
  if (!gScene)
    std::cerr << "createScene failed!" << std::endl;

  gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,
                                    1.0f);

  PxMaterial *mMaterial = gPhysicsSDK->createMaterial(0.5, 0.5, 0.5);

  PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),
                                 PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));

  PxRigidStatic *plane = gPhysicsSDK->createRigidStatic(pose);
  if (!plane)
    std::cerr << "create plane failed!" << std::endl;

  PxShape *shape = PxRigidActorExt::createExclusiveShape(
      *plane, PxPlaneGeometry(), *mMaterial);

  if (!shape)
    std::cerr << "create shape failed!" << std::endl;
  gScene->addActor(*plane);
  AddActorForRendering(gSceneRender, plane, PxGeometryType::Enum::ePLANE);

  // 2) Create cube
  PxReal density = 1.0f;
  PxTransform transform(PxVec3(0.0f, 5.0, 0.0f), PxQuat(PxIdentity));
  PxVec3 dimensions(1, 1, 1);
  PxBoxGeometry geometry(dimensions);

  for (int i = 0; i < 10; i++) {
    transform.p = PxVec3(0.0f, 5.0f + 5 * i, 0.0f);
    PxRigidDynamic *actor =
        PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
    if (!actor)
      std::cerr << "create actor failed!" << std::endl;
    actor->setAngularDamping(0.75);
    actor->setLinearVelocity(PxVec3(0, 0, 0));
    gScene->addActor(*actor);
    boxes.push_back(actor);

    AddActorForRendering(gSceneRender, actor, PxGeometryType::Enum::eBOX);
  }
}

void ShutdownPhysX() {
  for (PxU32 i = 0; i < boxes.size(); i++) {
    gScene->removeActor(*boxes[i]);
    boxes[i]->release();
  }

  boxes.clear();
  gScene->release();

  gPhysicsSDK->release();
}


constexpr int MAX_PATH = 1000;
char buffer[MAX_PATH];

void BeforeRender() {
  // Update PhysX
  if (gScene) {
    StepPhysX();
  }
  RenderActors();
}

void OnShutdown() { ShutdownPhysX(); }

int main(int argc, char **argv) {
  Optifuser::GLFWRenderContext &context =
      Optifuser::GLFWRenderContext::Get(WINDOW_WIDTH, WINDOW_HEIGHT);
  gSceneRender = std::make_shared<Optifuser::Scene>();
  auto cam = Optifuser::NewObject<Optifuser::Camera>();
  cam->position = {0, 4, 8};
  cam->rotatePitch(-0.3);
  cam->fovy = glm::radians(45.f);
  cam->aspect = WINDOW_WIDTH / (float)WINDOW_HEIGHT;
  gSceneRender->addObject(cam);
  gSceneRender->setMainCamera(cam);
  gSceneRender->addDirectionalLight({ {0, -1, -1}, {1, 1, 1} });
      gSceneRender->setAmbientLight(glm::vec3(0.1, 0.1, 0.1));

  gSceneRender->setEnvironmentMap("../assets/ame_desert/desertsky_ft.tga",
                           "../assets/ame_desert/desertsky_bk.tga",
                           "../assets/ame_desert/desertsky_up.tga",
                           "../assets/ame_desert/desertsky_dn.tga",
                           "../assets/ame_desert/desertsky_lf.tga",
                           "../assets/ame_desert/desertsky_rt.tga");

  context.renderer.setGBufferShader("../glsl_shader/gbuffer.vsh",
                                    "../glsl_shader/gbuffer.fsh");
  context.renderer.setDeferredShader("../glsl_shader/deferred.vsh",
                                     "../glsl_shader/deferred.fsh");

  InitializePhysX();
  while (!context.shouldExit()) {
    context.processEvents();

    float dt = 0.05f;
    if (gSceneRender->getMainCamera()) {
      if (Optifuser::input.getKeyState(GLFW_KEY_W)) {
        gSceneRender->getMainCamera()->move(0, 0, 2 * dt);
      } else if (Optifuser::input.getKeyState(GLFW_KEY_S)) {
        gSceneRender->getMainCamera()->move(0, 0, -dt);
      }
      if (Optifuser::input.getKeyState(GLFW_KEY_A)) {
        gSceneRender->getMainCamera()->move(0, -dt, 0);
      } else if (Optifuser::input.getKeyState(GLFW_KEY_D)) {
        gSceneRender->getMainCamera()->move(0, dt, 0);
      }

      if (Optifuser::input.getMouseButton(GLFW_MOUSE_BUTTON_RIGHT) ==
          GLFW_PRESS) {
        double dx, dy;
        Optifuser::input.getCursor(dx, dy);
        gSceneRender->getMainCamera()->rotateYaw(-dx / 1000.f);
        gSceneRender->getMainCamera()->rotatePitch(-dy / 1000.f);
      }
    }
    BeforeRender();
    context.render(gSceneRender);
  }
}
