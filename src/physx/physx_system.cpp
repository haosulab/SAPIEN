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
#include "sapien/physx/physx_system.h"
#include "../logger.h"
#include "./filter_shader.hpp"
#include "sapien/math/conversion.h"
#include "sapien/physx/articulation.h"
#include "sapien/physx/articulation_link_component.h"
#include "sapien/physx/material.h"
#include "sapien/physx/physx_default.h"
#include "sapien/physx/rigid_component.h"
#include "sapien/profiler.h"
#include <extensions/PxExtensionsAPI.h>

#ifdef SAPIEN_CUDA
#include "./physx_system.cuh"
#include <PxDirectGPUAPI.h>
#include <cuda.h>
#include <cuda_runtime.h>
#endif

using namespace physx;
namespace sapien {
namespace physx {

struct SapienBodyDataTest {
  Pose pose;
  Vec3 v;
  Vec3 w;
};

static_assert(sizeof(SapienBodyDataTest) == 52);

PhysxSystem::PhysxSystem()
    : mSceneConfig(PhysxDefault::getSceneConfig()), mEngine(PhysxEngine::Get()) {}

PhysxSystemCpu::PhysxSystemCpu() {
  if (PhysxDefault::GetGPUEnabled()) {
    logger::warn(
        "A PhysX CPU system is being created while PhysX GPU is enabled. You can safely ignore "
        "this message if it is intended. To use GPU PhysX, create a sapien.physx.PhysxGpuSystem "
        "explicitly and pass it to sapien.Scene constructor.");
  }

  auto &config = mSceneConfig;
  PxSceneDesc sceneDesc(mEngine->getPxPhysics()->getTolerancesScale());
  sceneDesc.gravity = Vec3ToPxVec3(config.gravity);
  sceneDesc.filterShader = TypeAffinityIgnoreFilterShader;
  sceneDesc.solverType = config.enableTGS ? PxSolverType::eTGS : PxSolverType::ePGS;
  sceneDesc.bounceThresholdVelocity = config.bounceThreshold;

  PxSceneFlags sceneFlags;
  if (config.enableEnhancedDeterminism) {
    sceneFlags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
  }
  if (config.enablePCM) {
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }
  if (config.enableCCD) {
    sceneFlags |= PxSceneFlag::eENABLE_CCD;
  }
  if (config.enableFrictionEveryIteration) {
    sceneFlags |= PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION;
  }

  sceneDesc.flags = sceneFlags;

  mPxCPUDispatcher = PxDefaultCpuDispatcherCreate(config.cpuWorkers);
  if (!mPxCPUDispatcher) {
    throw std::runtime_error("PhysX system creation failed: failed to create CPU dispatcher");
  }
  sceneDesc.cpuDispatcher = mPxCPUDispatcher;
  mPxScene = mEngine->getPxPhysics()->createScene(sceneDesc);
  mPxScene->setSimulationEventCallback(&mSimulationCallback);
}

#ifdef SAPIEN_CUDA
PhysxSystemGpu::PhysxSystemGpu(std::shared_ptr<Device> device) {
  if (!PhysxDefault::GetGPUEnabled()) {
    throw std::runtime_error(
        "sapien.physx.enable_gpu() must be called before creating a PhysX GPU system.");
  }

  if (!device) {
    device = findDevice("cuda");
    if (!device) {
      throw std::runtime_error("failed to find a CUDA device for PhysX GPU");
    }
  } else if (!device->isCuda()) {
    throw std::runtime_error(
        "failed to create PhysX GPU system: device provided does not support CUDA");
  }
  mDevice = device;

  auto &config = mSceneConfig;
  PxSceneDesc sceneDesc(mEngine->getPxPhysics()->getTolerancesScale());
  sceneDesc.gravity = Vec3ToPxVec3(config.gravity);
  sceneDesc.filterShader = TypeAffinityIgnoreFilterShaderGpu;
  sceneDesc.solverType = config.enableTGS ? PxSolverType::eTGS : PxSolverType::ePGS;
  sceneDesc.bounceThresholdVelocity = config.bounceThreshold;

  sceneDesc.gpuDynamicsConfig = PhysxDefault::getGpuMemoryConfig();

  PxSceneFlags sceneFlags;
  if (config.enableEnhancedDeterminism) {
    sceneFlags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
  }
  if (config.enablePCM) {
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }
  if (config.enableCCD) {
    sceneFlags |= PxSceneFlag::eENABLE_CCD;
  }
  if (config.enableFrictionEveryIteration) {
    sceneFlags |= PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION;
  }

  sceneFlags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
  sceneFlags |= PxSceneFlag::eENABLE_DIRECT_GPU_API;
  sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
  sceneDesc.cudaContextManager = mEngine->getCudaContextManager(device->cudaId);
  if (!config.enablePCM) {
    logger::warn("PCM must be enabled when using GPU.");
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }

  sceneDesc.flags = sceneFlags;

  mPxCPUDispatcher = PxDefaultCpuDispatcherCreate(config.cpuWorkers);
  if (!mPxCPUDispatcher) {
    throw std::runtime_error("PhysX system creation failed: failed to create CPU dispatcher");
  }
  sceneDesc.cpuDispatcher = mPxCPUDispatcher;
  mPxScene = mEngine->getPxPhysics()->createScene(sceneDesc);
}
#else
PhysxSystemGpu::PhysxSystemGpu(std::shared_ptr<Device> device) {
  throw std::runtime_error(
        "Does not support PhysX GPU system.");
}
#endif

void PhysxSystemCpu::registerComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.insert(component);
}
void PhysxSystemCpu::registerComponent(std::shared_ptr<PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.insert(component);
}
void PhysxSystemCpu::registerComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.insert(component);
}
void PhysxSystemCpu::unregisterComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.erase(component);
}
void PhysxSystemCpu::unregisterComponent(std::shared_ptr<PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.erase(component);
}
void PhysxSystemCpu::unregisterComponent(
    std::shared_ptr<PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.erase(component);
}
std::vector<std::shared_ptr<PhysxRigidDynamicComponent>>
PhysxSystemCpu::getRigidDynamicComponents() const {
  return {mRigidDynamicComponents.begin(), mRigidDynamicComponents.end()};
}
std::vector<std::shared_ptr<PhysxRigidStaticComponent>>
PhysxSystemCpu::getRigidStaticComponents() const {
  return {mRigidStaticComponents.begin(), mRigidStaticComponents.end()};
}
std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
PhysxSystemCpu::getArticulationLinkComponents() const {
  return {mArticulationLinkComponents.begin(), mArticulationLinkComponents.end()};
}

#ifdef SAPIEN_CUDA
void PhysxSystemGpu::registerComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.insert(component);
  mGpuInitialized = false;
}
void PhysxSystemGpu::registerComponent(std::shared_ptr<PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.insert(component);
  mGpuInitialized = false;
}
void PhysxSystemGpu::registerComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.insert(component);
  mGpuInitialized = false;
}
void PhysxSystemGpu::unregisterComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.erase(component);
  mGpuInitialized = false;
}
void PhysxSystemGpu::unregisterComponent(std::shared_ptr<PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.erase(component);
  mGpuInitialized = false;
}
void PhysxSystemGpu::unregisterComponent(
    std::shared_ptr<PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.erase(component);
  mGpuInitialized = false;
}
std::vector<std::shared_ptr<PhysxRigidDynamicComponent>>
PhysxSystemGpu::getRigidDynamicComponents() const {
  return {mRigidDynamicComponents.begin(), mRigidDynamicComponents.end()};
}
std::vector<std::shared_ptr<PhysxRigidStaticComponent>>
PhysxSystemGpu::getRigidStaticComponents() const {
  return {mRigidStaticComponents.begin(), mRigidStaticComponents.end()};
}
std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
PhysxSystemGpu::getArticulationLinkComponents() const {
  return {mArticulationLinkComponents.begin(), mArticulationLinkComponents.end()};
}
#endif

std::unique_ptr<PhysxHitInfo> PhysxSystemCpu::raycast(Vec3 const &origin, Vec3 const &direction,
                                                      float distance) {
  PxRaycastBuffer hit;
  bool status = mPxScene->raycast(Vec3ToPxVec3(origin), Vec3ToPxVec3(direction), distance, hit);
  if (status) {
    return std::make_unique<PhysxHitInfo>(
        PxVec3ToVec3(hit.block.position), PxVec3ToVec3(hit.block.normal), hit.block.distance,
        static_cast<PhysxCollisionShape *>(hit.block.shape->userData),
        static_cast<PhysxRigidBaseComponent *>(hit.block.actor->userData));
  }
  return nullptr;
}

void PhysxSystemCpu::step() {
  mPxScene->simulate(mTimestep);
  mPxScene->fetchResults(true);
  for (auto c : mRigidStaticComponents) {
    c->syncPoseToEntity();
  }
  for (auto c : mRigidDynamicComponents) {
    c->syncPoseToEntity();
  }
  for (auto c : mArticulationLinkComponents) {
    c->syncPoseToEntity();
  }
}

#ifdef SAPIEN_CUDA
void PhysxSystemGpu::step() {
  stepStart();
  stepFinish();
}

void PhysxSystemGpu::stepStart() {
  checkGpuIdle();
  mContactUpToDate = false;
  ++mTotalSteps;
  mPxScene->simulate(mTimestep);
  mStepInProgress = true;
  mApplyPositionWithoutStep = false;
}

void PhysxSystemGpu::stepFinish() {
  if (!mStepInProgress) {
    throw std::runtime_error("step_finish requires a pending step_start");
  }
  PxU32 error = 0;
  bool ok = mPxScene->fetchResults(true, &error);
  mStepInProgress = false;
  if (!ok || error) {
    throw std::runtime_error("PhysX GPU step failed (error " + std::to_string(error) + ")");
  }
}
#endif

std::string PhysxSystemCpu::packState() const {
  std::ostringstream ss;
  for (auto &actor : mRigidDynamicComponents) {
    Pose pose = actor->getPose();
    Vec3 v = actor->getLinearVelocity();
    Vec3 w = actor->getAngularVelocity();
    ss.write(reinterpret_cast<const char *>(&pose), sizeof(Pose));
    ss.write(reinterpret_cast<const char *>(&v), sizeof(Vec3));
    ss.write(reinterpret_cast<const char *>(&w), sizeof(Vec3));
  }
  for (auto &link : mArticulationLinkComponents) {
    if (link->isRoot()) {
      auto art = link->getArticulation();

      Pose pose = art->getRootPose();
      Vec3 v = art->getRootLinearVelocity();
      Vec3 w = art->getRootAngularVelocity();
      ss.write(reinterpret_cast<const char *>(&pose), sizeof(Pose));
      ss.write(reinterpret_cast<const char *>(&v), sizeof(Vec3));
      ss.write(reinterpret_cast<const char *>(&w), sizeof(Vec3));

      auto qpos = art->getQpos();
      auto qvel = art->getQvel();

      ss.write(reinterpret_cast<const char *>(qpos.data()), qpos.size() * sizeof(float));
      ss.write(reinterpret_cast<const char *>(qvel.data()), qvel.size() * sizeof(float));

      for (auto j : art->getActiveJoints()) {
        auto pos = j->getDriveTargetPosition();
        auto vel = j->getDriveTargetVelocity();

        ss.write(reinterpret_cast<const char *>(pos.data()), pos.size() * sizeof(float));
        ss.write(reinterpret_cast<const char *>(vel.data()), vel.size() * sizeof(float));
      }
    }
  }
  return ss.str();
}

void PhysxSystemCpu::unpackState(std::string const &data) {
  std::istringstream ss(data);
  for (auto &actor : mRigidDynamicComponents) {
    Pose pose;
    Vec3 v, w;
    ss.read(reinterpret_cast<char *>(&pose), sizeof(Pose));
    ss.read(reinterpret_cast<char *>(&v), sizeof(Vec3));
    ss.read(reinterpret_cast<char *>(&w), sizeof(Vec3));
    actor->setPose(pose);
    if (!actor->isKinematic()) {
      actor->setLinearVelocity(v);
      actor->setAngularVelocity(w);
    }
  }
  for (auto &link : mArticulationLinkComponents) {
    if (link->isRoot()) {
      Pose pose;
      Vec3 v, w;
      ss.read(reinterpret_cast<char *>(&pose), sizeof(Pose));
      ss.read(reinterpret_cast<char *>(&v), sizeof(Vec3));
      ss.read(reinterpret_cast<char *>(&w), sizeof(Vec3));
      auto art = link->getArticulation();
      art->setRootPose(pose);
      art->setRootLinearVelocity(v);
      art->setRootAngularVelocity(w);

      Eigen::VectorXf qpos;
      Eigen::VectorXf qvel;
      qpos.resize(art->getDof());
      qvel.resize(art->getDof());
      ss.read(reinterpret_cast<char *>(qpos.data()), qpos.size() * sizeof(float));
      ss.read(reinterpret_cast<char *>(qvel.data()), qvel.size() * sizeof(float));
      art->setQpos(qpos);
      art->setQvel(qvel);

      for (auto j : art->getActiveJoints()) {
        Eigen::VectorXf pos, vel;
        pos.resize(j->getDof());
        vel.resize(j->getDof());
        ss.read(reinterpret_cast<char *>(pos.data()), pos.size() * sizeof(float));
        ss.read(reinterpret_cast<char *>(vel.data()), vel.size() * sizeof(float));
        j->setDriveTargetPosition(pos);
        j->setDriveTargetVelocity(vel);
      }
    }
  }
}

int PhysxSystem::getArticulationCount() const {
  // TODO: ensure this count matches registered articulations
  return getPxScene()->getNbArticulations();
}

int PhysxSystem::computeArticulationMaxDof() const {
  int result = 0;
  uint32_t count = getPxScene()->getNbArticulations();
  std::vector<PxArticulationReducedCoordinate *> articulations(count);
  getPxScene()->getArticulations(articulations.data(), count);
  for (auto a : articulations) {
    result = std::max(result, static_cast<int>(a->getDofs()));
  }
  return result;
}

int PhysxSystem::computeArticulationMaxLinkCount() const {
  int result = 0;
  uint32_t count = getPxScene()->getNbArticulations();
  std::vector<PxArticulationReducedCoordinate *> articulations(count);
  getPxScene()->getArticulations(articulations.data(), count);
  for (auto a : articulations) {
    result = std::max(result, static_cast<int>(a->getNbLinks()));
  }
  return result;
}

#ifdef SAPIEN_CUDA
void PhysxSystemGpu::gpuInit() {
  if (mStepInProgress) {
    throw std::runtime_error("Finish the pending physics step before gpu_init");
  }
  ++mTotalSteps;
  ensureCudaDevice();
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

  allocateCudaBuffers();

  ensureCudaDevice();
  mCudaEventRecord.init();
  mCudaEventWait.init();

  mGpuInitialized = true;
  mContactUpToDate = false;
}

void PhysxSystemGpu::checkGpuInitialized() const {
  if (!isInitialized()) {
    throw std::runtime_error("GPU PhysX is not initialized.");
  }
}

void PhysxSystemGpu::checkGpuIdle() const {
  checkGpuInitialized();
  if (mStepInProgress) {
    throw std::runtime_error("Finish the pending physics step before accessing GPU state");
  }
}

void PhysxSystemGpu::gpuSetCudaStream(uintptr_t stream) { mCudaStream = (cudaStream_t)stream; }

std::shared_ptr<PhysxGpuContactPairImpulseQuery> PhysxSystemGpu::gpuCreateContactPairImpulseQuery(
    std::vector<std::pair<std::shared_ptr<PhysxRigidBaseComponent>,
                          std::shared_ptr<PhysxRigidBaseComponent>>> const &bodyPairs) {
  if (bodyPairs.empty()) {
    throw std::runtime_error("failed to create contact query: empty body pairs");
  }
  std::vector<ActorPairQuery> pairs;
  for (uint32_t i = 0; i < bodyPairs.size(); ++i) {
    auto &[b0, b1] = bodyPairs[i];
    if (!b0 || !b1) {
      throw std::runtime_error("failed to create contact query: invalid body");
    }
    int order{0};
    ActorPair pair = makeActorPair(b0->getPxActor(), b1->getPxActor(), order);
    pairs.push_back({pair, i, order});
  }

  std::sort(pairs.begin(), pairs.end(),
            [](ActorPairQuery const &a, ActorPairQuery const &b) { return a.pair < b.pair; });

  static_assert(sizeof(ActorPairQuery) == 24);

  ensureCudaDevice();
  CudaArray query({static_cast<int>(pairs.size()), 6}, "i4");
  checkCudaErrors(cudaMemcpy(query.ptr, pairs.data(), pairs.size() * sizeof(ActorPairQuery),
                             cudaMemcpyHostToDevice));
  CudaArray buffer({static_cast<int>(pairs.size()), 3}, "f4");

  auto res = std::make_shared<PhysxGpuContactPairImpulseQuery>();
  res->query = std::move(query);
  res->buffer = std::move(buffer);
  return res;
}

std::shared_ptr<PhysxGpuContactBodyImpulseQuery> PhysxSystemGpu::gpuCreateContactBodyImpulseQuery(
    std::vector<std::shared_ptr<PhysxRigidBaseComponent>> const &bodies) {
  if (bodies.empty()) {
    throw std::runtime_error("failed to create contact query: empty body list");
  }
  std::vector<ActorQuery> actors;
  for (uint32_t i = 0; i < bodies.size(); ++i) {
    if (!bodies[i]) {
      throw std::runtime_error("failed to create contact actors: invalid body");
    }
    actors.push_back({bodies[i]->getPxActor(), i});
  }

  std::sort(actors.begin(), actors.end(),
            [](ActorQuery const &a, ActorQuery const &b) { return a.actor < b.actor; });
  static_assert(sizeof(ActorQuery) == 16);

  ensureCudaDevice();
  CudaArray query({static_cast<int>(actors.size()), 4}, "i4");
  checkCudaErrors(cudaMemcpy(query.ptr, actors.data(), actors.size() * sizeof(ActorQuery),
                             cudaMemcpyHostToDevice));
  CudaArray buffer({static_cast<int>(actors.size()), 3}, "f4");

  // TODO: use dedicated type, do not reuse contact query
  auto res = std::make_shared<PhysxGpuContactBodyImpulseQuery>();
  res->query = std::move(query);
  res->buffer = std::move(buffer);
  return res;
}
#endif

inline static int upperPowerOf2(int x) {
  x--;
  x |= x >> 1;
  x |= x >> 2;
  x |= x >> 4;
  x |= x >> 8;
  x |= x >> 16;
  x++;
  return x;
}

#ifdef SAPIEN_CUDA
void PhysxSystemGpu::copyContactData() {
  checkGpuIdle();
  if (mContactUpToDate) {
    return;
  }

  ensureCudaDevice();
  if (!mCudaContactCount.ptr) {
    mCudaContactCount = CudaArray({1}, "u4");
  }

  if (!mCudaContactBuffer.ptr) {
    mCudaContactBuffer = CudaArray({1024, sizeof(PxGpuContactPair)}, "u1");
  }

  SAPIEN_PROFILE_BLOCK_BEGIN("fetch contact count");
  mPxScene->getDirectGPUAPI().copyContactData(mCudaContactBuffer.ptr,
                                               static_cast<uint32_t *>(mCudaContactCount.ptr), 0);
  cudaMemcpy(&mContactCount, mCudaContactCount.ptr, sizeof(int), cudaMemcpyDeviceToHost);
  SAPIEN_PROFILE_BLOCK_END;

  int size = upperPowerOf2(mContactCount);
  if (mCudaContactBuffer.shape[0] < size) {
    SAPIEN_PROFILE_BLOCK("re-allocate contact buffer");
    mCudaContactBuffer = CudaArray({size, sizeof(PxGpuContactPair)}, "u1");
  }

  mPxScene->getDirectGPUAPI().copyContactData(mCudaContactBuffer.ptr,
                                              static_cast<uint32_t *>(mCudaContactCount.ptr),
                                              size);

  mContactUpToDate = true;
}

void PhysxSystemGpu::gpuQueryContactPairImpulses(PhysxGpuContactPairImpulseQuery const &query) {
  SAPIEN_PROFILE_FUNCTION;
  query.query.handle().checkShape({-1, 6});

  if (mApplyPositionWithoutStep) {
    logger::debug("Contact queried between apply and step. This may be unintended. Note that "
                  "contacts are only updated after step and should be queried immediately after "
                  "stepping. Setting position and then query contact will report contacts from "
                  "last simulation step.");
  }

  ensureCudaDevice();
  cudaMemsetAsync(query.buffer.ptr, 0, query.query.shape.at(0) * 3 * sizeof(float), mCudaStream);

  copyContactData();

  if (mContactCount) {
    handle_contacts((PxGpuContactPair *)mCudaContactBuffer.ptr, mContactCount,
                    (ActorPairQuery *)query.query.ptr, query.query.shape.at(0),
                    (Vec3 *)query.buffer.ptr, mCudaStream);
  }

  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaStreamSynchronize(mCudaStream));
}

void PhysxSystemGpu::gpuQueryContactBodyImpulses(PhysxGpuContactBodyImpulseQuery const &query) {
  SAPIEN_PROFILE_FUNCTION;
  query.query.handle().checkShape({-1, 4});

  if (mApplyPositionWithoutStep) {
    logger::debug("Contact queried between apply and step. This may be unintended. Note that "
                  "contacts are only updated after step and should be queried immediately after "
                  "stepping. Setting position and then query contact will report contacts from "
                  "last simulation step.");
  }

  ensureCudaDevice();
  cudaMemsetAsync(query.buffer.ptr, 0, query.query.shape.at(0) * 3 * sizeof(float), mCudaStream);

  copyContactData();

  if (mContactCount) {
    handle_net_contact_force((PxGpuContactPair *)mCudaContactBuffer.ptr, mContactCount,
                             (ActorQuery *)query.query.ptr, query.query.shape.at(0),
                             (Vec3 *)query.buffer.ptr, mCudaStream);
  }
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaStreamSynchronize(mCudaStream));
}

void PhysxSystemGpu::gpuFetchRigidDynamicData() {
  checkGpuInitialized();
  if (mRigidDynamicComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getRigidDynamicData(
      mCudaRigidDynamicPoseHandle.ptr, (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mCudaRigidDynamicIndexBuffer.shape.at(0), nullptr,
      mCudaEventWait.event);
  mPxScene->getDirectGPUAPI().getRigidDynamicData(
      mCudaRigidDynamicLinearVelocityHandle.ptr,
      (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY, mCudaRigidDynamicIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mPxScene->getDirectGPUAPI().getRigidDynamicData(
      mCudaRigidDynamicAngularVelocityHandle.ptr,
      (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY, mCudaRigidDynamicIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);

  body_data_physx_to_sapien(
      (SapienBodyData *)mCudaRigidDynamicDataHandle.ptr, (PhysxPose *)mCudaRigidDynamicPoseHandle.ptr,
      (Vec3 *)mCudaRigidDynamicLinearVelocityHandle.ptr,
      (Vec3 *)mCudaRigidDynamicAngularVelocityHandle.ptr,
      (Vec3 *)mCudaRigidDynamicOffsetBuffer.ptr, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationLinkData() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaLinkPoseHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaLinkLinearVelocityHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaLinkAngularVelocityHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);

  int maxLinks = mCudaLinkPoseHandle.shape.at(1);
  link_data_physx_to_sapien((SapienBodyData *)mCudaLinkDataHandle.ptr,
                           (PhysxPose *)mCudaLinkPoseHandle.ptr,
                           (Vec3 *)mCudaLinkLinearVelocityHandle.ptr,
                           (Vec3 *)mCudaLinkAngularVelocityHandle.ptr,
                           (Vec3 *)mCudaArticulationOffsetBuffer.ptr, maxLinks,
                           mCudaLinkPoseHandle.shape.at(0) * maxLinks, mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQpos() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaQposHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eJOINT_POSITION, mCudaArticulationIndexBuffer.shape.at(0), nullptr,
      mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQvel() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaQvelHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eJOINT_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0), nullptr,
      mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQTargetPos() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaQTargetPosHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQTargetVel() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaQTargetVelHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationLinkIncomingJointForce() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaArticulationLinkIncomingJointForceBuffer.ptr,
      (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQacc() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mPxScene->getDirectGPUAPI().getArticulationData(
      mCudaQaccHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIReadType::eJOINT_ACCELERATION, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuUpdateArticulationKinematics() {
  checkGpuInitialized();
  ensureCudaDevice();
  checkCudaErrors(cudaDeviceSynchronize()); // is this needed?
  mPxScene->getDirectGPUAPI().computeArticulationData(
      nullptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIComputeType::eUPDATE_KINEMATIC, mCudaArticulationIndexBuffer.shape.at(0),
      nullptr, nullptr);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyRigidDynamicData() {
  SAPIEN_PROFILE_FUNCTION;
  checkGpuInitialized();
  if (mRigidDynamicComponents.empty()) {
    return;
  }
  ensureCudaDevice();
  body_data_sapien_to_physx(
      (SapienBodyData *)mCudaRigidDynamicDataHandle.ptr, (PhysxPose *)mCudaRigidDynamicPoseHandle.ptr,
      (Vec3 *)mCudaRigidDynamicLinearVelocityHandle.ptr,
      (Vec3 *)mCudaRigidDynamicAngularVelocityHandle.ptr,
      (Vec3 *)mCudaRigidDynamicOffsetBuffer.ptr, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaStream);
  mCudaEventRecord.record(mCudaStream);

  mPxScene->getDirectGPUAPI().setRigidDynamicData(
      mCudaRigidDynamicPoseHandle.ptr, (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mPxScene->getDirectGPUAPI().setRigidDynamicData(
      mCudaRigidDynamicLinearVelocityHandle.ptr,
      (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mPxScene->getDirectGPUAPI().setRigidDynamicData(
      mCudaRigidDynamicAngularVelocityHandle.ptr,
      (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);

  mCudaEventWait.wait(mCudaStream);
  mApplyPositionWithoutStep = true;
}

void PhysxSystemGpu::gpuApplyArticulationRootData() {
  SAPIEN_PROFILE_FUNCTION;
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  root_data_sapien_to_physx((SapienBodyData *)mCudaLinkDataHandle.ptr,
                            (PhysxPose *)mCudaRootPoseBuffer.ptr,
                            (Vec3 *)mCudaRootLinearVelocityBuffer.ptr,
                            (Vec3 *)mCudaRootAngularVelocityBuffer.ptr,
                            (Vec3 *)mCudaArticulationOffsetBuffer.ptr, mCudaLinkDataHandle.shape.at(1),
                            mCudaLinkDataHandle.shape.at(0), mCudaStream);
  mCudaEventRecord.record(mCudaStream);

  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaRootPoseBuffer.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);

  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaRootLinearVelocityBuffer.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);

  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaRootAngularVelocityBuffer.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);

  mCudaEventWait.wait(mCudaStream);
  mApplyPositionWithoutStep = true;
}

void PhysxSystemGpu::gpuApplyRigidDynamicForce() {
  checkGpuInitialized();
  if (mRigidDynamicComponents.empty()) {
    return;
  }
  ensureCudaDevice();
  mCudaEventRecord.record(mCudaStream);

  mPxScene->getDirectGPUAPI().setRigidDynamicData(
      mCudaRigidDynamicForceHandle.ptr, (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIWriteType::eFORCE, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyRigidDynamicTorque() {
  checkGpuInitialized();
  if (mRigidDynamicComponents.empty()) {
    return;
  }
  ensureCudaDevice();
  mCudaEventRecord.record(mCudaStream);

  mPxScene->getDirectGPUAPI().setRigidDynamicData(
      mCudaRigidDynamicTorqueHandle.ptr, (const PxRigidDynamicGPUIndex *)mCudaRigidDynamicIndexBuffer.ptr,
      PxRigidDynamicGPUAPIWriteType::eTORQUE, mCudaRigidDynamicIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyLinkForce() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaLinkForceHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eLINK_FORCE, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);

  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyLinkTorque() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaLinkTorqueHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eLINK_TORQUE, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);

  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQpos() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaQposHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eJOINT_POSITION, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
  mApplyPositionWithoutStep = true;
}

void PhysxSystemGpu::gpuApplyArticulationQvel() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaQvelHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eJOINT_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQf() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaQfHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eJOINT_FORCE, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQTargetPos() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaQTargetPosHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQTargetVel() {
  checkGpuInitialized();
  if (mArticulationLinkComponents.empty()) {
    return;
  }
  ensureCudaDevice();

  mCudaEventRecord.record(mCudaStream);
  mPxScene->getDirectGPUAPI().setArticulationData(
      mCudaQTargetVelHandle.ptr, (const PxArticulationGPUIndex *)mCudaArticulationIndexBuffer.ptr,
      PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY, mCudaArticulationIndexBuffer.shape.at(0),
      mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::syncPosesGpuToCpu() {
  checkGpuInitialized();
  gpuFetchRigidDynamicData();
  gpuFetchArticulationLinkData();
  if (mCudaHostBodyBuffer.shape != mCudaBodyDataBuffer.shape) {
    mCudaHostBodyBuffer = CudaHostArray(mCudaBodyDataBuffer.shape, mCudaBodyDataBuffer.type);
  }
  mCudaHostBodyBuffer.copyFrom(mCudaBodyDataBuffer);
  auto data = (SapienBodyData *)mCudaHostBodyBuffer.ptr;

  for (auto &body : mRigidDynamicComponents) {
    assert(body->getGpuPoseIndex() >= 0);
    body->getEntity()->internalSyncPose(
        {data[body->getGpuPoseIndex()].p, data[body->getGpuPoseIndex()].q});
  }
  for (auto &body : mArticulationLinkComponents) {
    assert(body->getGpuPoseIndex() >= 0);
    body->getEntity()->internalSyncPose(
        {data[body->getGpuPoseIndex()].p, data[body->getGpuPoseIndex()].q});
  }
}

std::vector<float> PhysxSystemGpu::gpuDownloadArticulationQpos(int index) {
  ensureCudaDevice();
  gpuFetchArticulationQpos();
  cudaStreamSynchronize(mCudaStream);
  auto counts = mPxScene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts();

  if (index < 0 || index >= mCudaQposHandle.shape.at(0)) {
    throw std::runtime_error("failed to download articulation qpos: invalid index");
  }

  std::vector<float> buffer(counts.maxDofs);

  cudaMemcpy(buffer.data(), &((float *)mCudaQposHandle.ptr)[index * counts.maxDofs],
             counts.maxDofs * sizeof(float), cudaMemcpyDeviceToHost);
  return buffer;
}

void PhysxSystemGpu::gpuUploadArticulationQpos(int index, Eigen::VectorXf const &q) {
  ensureCudaDevice();
  cudaStreamSynchronize(mCudaStream);
  auto counts = mPxScene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts();

  if (index < 0 || index >= mCudaQposHandle.shape.at(0)) {
    throw std::runtime_error("failed to upload articulation qpos: invalid index");
  }

  cudaMemcpy(&((float *)mCudaQposHandle.ptr)[index * counts.maxDofs], q.data(),
             q.size() * sizeof(float), cudaMemcpyHostToDevice);
  gpuApplyArticulationQpos();
}

void PhysxSystemGpu::setSceneOffset(std::shared_ptr<Scene> scene, Vec3 offset) {
  // clean up occasionally
  if (mSceneOffset.size() % 1024 == 0) {
    std::erase_if(mSceneOffset, [](const auto &p) { return p.first.expired(); });
  }

  mSceneOffset[scene] = offset;
}

Vec3 PhysxSystemGpu::getSceneOffset(std::shared_ptr<Scene> scene) const {
  if (mSceneOffset.contains(scene)) {
    return mSceneOffset.at(scene);
  }
  return Vec3(0.0f);
}

void PhysxSystemGpu::allocateCudaBuffers() {
  SAPIEN_PROFILE_FUNCTION;
  auto counts = mPxScene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts();

  int rigidDynamicCount = mRigidDynamicComponents.size();
  int articulationCount = mPxScene->getNbArticulations();
  int maxLinks = counts.maxLinks;
  int linkCount = articulationCount * maxLinks;
  int maxDofs = counts.maxDofs;
  int bodyCount = rigidDynamicCount + linkCount;

  ensureCudaDevice();

  // pose velocity

  mCudaBodyPoseBuffer = CudaArray({bodyCount, 7}, "f4");
  mCudaRigidDynamicPoseHandle = mCudaBodyPoseBuffer.handle().slice(0, rigidDynamicCount);
  mCudaLinkPoseHandle = mCudaBodyPoseBuffer.handle()
                             .slice(rigidDynamicCount, rigidDynamicCount + linkCount)
                             .view({articulationCount, maxLinks, 7});

  mCudaBodyLinearVelocityBuffer = CudaArray({bodyCount, 3}, "f4");
  mCudaRigidDynamicLinearVelocityHandle =
      mCudaBodyLinearVelocityBuffer.handle().slice(0, rigidDynamicCount);
  mCudaLinkLinearVelocityHandle = mCudaBodyLinearVelocityBuffer.handle()
                                      .slice(rigidDynamicCount, rigidDynamicCount + linkCount)
                                      .view({articulationCount, maxLinks, 3});

  mCudaBodyAngularVelocityBuffer = CudaArray({bodyCount, 3}, "f4");
  mCudaRigidDynamicAngularVelocityHandle =
      mCudaBodyAngularVelocityBuffer.handle().slice(0, rigidDynamicCount);
  mCudaLinkAngularVelocityHandle = mCudaBodyAngularVelocityBuffer.handle()
                                       .slice(rigidDynamicCount, rigidDynamicCount + linkCount)
                                       .view({articulationCount, maxLinks, 3});

  mCudaRootPoseBuffer = CudaArray({articulationCount, 7}, "f4");
  mCudaRootLinearVelocityBuffer = CudaArray({articulationCount, 3}, "f4");
  mCudaRootAngularVelocityBuffer = CudaArray({articulationCount, 3}, "f4");

  mCudaBodyDataBuffer = CudaArray({bodyCount, 13}, "f4");
  mCudaRigidDynamicDataHandle = mCudaBodyDataBuffer.handle().slice(0, rigidDynamicCount);
  mCudaLinkDataHandle = mCudaBodyDataBuffer.handle()
                            .slice(rigidDynamicCount, rigidDynamicCount + linkCount)
                            .view({articulationCount, maxLinks, 13});

  // force torque

  mCudaBodyForceBuffer = CudaArray({bodyCount, 3}, "f4");
  mCudaRigidDynamicForceHandle = mCudaBodyForceBuffer.handle().slice(0, rigidDynamicCount);
  mCudaLinkForceHandle = mCudaBodyForceBuffer.handle()
                             .slice(rigidDynamicCount, rigidDynamicCount + linkCount)
                             .view({articulationCount, maxLinks, 3});

  mCudaBodyTorqueBuffer = CudaArray({bodyCount, 3}, "f4");
  mCudaRigidDynamicTorqueHandle = mCudaBodyTorqueBuffer.handle().slice(0, rigidDynamicCount);
  mCudaLinkTorqueHandle = mCudaBodyTorqueBuffer.handle()
                              .slice(rigidDynamicCount, rigidDynamicCount + linkCount)
                              .view({articulationCount, maxLinks, 3});

  // q

  mCudaArticulationBuffer = CudaArray({6, articulationCount, maxDofs}, "f4");
  mCudaQposHandle = mCudaArticulationBuffer.handle().slice(0, 1).view({articulationCount, maxDofs});
  mCudaQvelHandle = mCudaArticulationBuffer.handle().slice(1, 2).view({articulationCount, maxDofs});
  mCudaQfHandle = mCudaArticulationBuffer.handle().slice(2, 3).view({articulationCount, maxDofs});
  mCudaQaccHandle = mCudaArticulationBuffer.handle().slice(3, 4).view({articulationCount, maxDofs});
  mCudaQTargetPosHandle =
      mCudaArticulationBuffer.handle().slice(4, 5).view({articulationCount, maxDofs});
  mCudaQTargetVelHandle =
      mCudaArticulationBuffer.handle().slice(5, 6).view({articulationCount, maxDofs});

  mCudaArticulationLinkIncomingJointForceBuffer = CudaArray({articulationCount, maxLinks, 6}, "f4");

  {
    mCudaRigidDynamicIndexBuffer = CudaArray({rigidDynamicCount}, "i4");
    mCudaRigidDynamicOffsetBuffer = CudaArray({rigidDynamicCount, 3}, "f4");
    std::vector<std::array<float, 3>> host_offset;
    std::vector<PxRigidDynamicGPUIndex> host_index;
    auto bodies = getRigidDynamicComponents();
    for (uint32_t i = 0; i < bodies.size(); ++i) {
      Vec3 offset = getSceneOffset(bodies[i]->getScene());
      host_offset.push_back({offset.x, offset.y, offset.z});
      host_index.push_back(bodies[i]->getPxActor()->getGPUIndex());
      bodies[i]->internalSetGpuIndex(i);
    }
    checkCudaErrors(cudaMemcpy(mCudaRigidDynamicIndexBuffer.ptr, host_index.data(),
                               host_index.size() * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(mCudaRigidDynamicOffsetBuffer.ptr, host_offset.data(),
                               host_offset.size() * sizeof(float) * 3, cudaMemcpyHostToDevice));
  }

  {
    std::vector<std::shared_ptr<PhysxArticulation>> articulations;
    for (auto link : getArticulationLinkComponents()) {
      if (link->isRoot()) {
        articulations.push_back(link->getArticulation());
      }
    }
    mCudaArticulationIndexBuffer = CudaArray({articulationCount}, "i4");
    mCudaArticulationOffsetBuffer = CudaArray({articulationCount, 3}, "f4");
    std::vector<std::array<float, 3>> host_offset;
    std::vector<int> host_index;
    for (uint32_t i = 0; i < articulations.size(); ++i) {
      Vec3 offset = getSceneOffset(articulations.at(i)->getRoot()->getScene());
      host_offset.push_back({offset.x, offset.y, offset.z});
      host_index.push_back(articulations.at(i)->getPxArticulation()->getGPUIndex());
      for (auto link : articulations.at(i)->getLinks()) {
        link->internalSetGpuIndex(rigidDynamicCount + i * maxLinks + link->getIndex());
      }
    }

    checkCudaErrors(cudaMemcpy(mCudaArticulationOffsetBuffer.ptr, host_offset.data(),
                               host_offset.size() * sizeof(float) * 3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(mCudaArticulationIndexBuffer.ptr, host_index.data(),
                               host_index.size() * sizeof(int), cudaMemcpyHostToDevice));
  }
}

void PhysxSystemGpu::ensureCudaDevice() { checkCudaErrors(cudaSetDevice(mDevice->cudaId)); }
#endif

PhysxSystem::~PhysxSystem() { logger::info("Deleting PhysxSystem"); }

PhysxSystemCpu::~PhysxSystemCpu() {
  if (mPxScene) {
    mPxScene->release();
  }
  if (mPxCPUDispatcher) {
    mPxCPUDispatcher->release();
  }
}

#ifdef SAPIEN_CUDA
PhysxSystemGpu::~PhysxSystemGpu() {
  if (mPxScene) {
    mPxScene->release();
  }
  if (mPxCPUDispatcher) {
    mPxCPUDispatcher->release();
  }
}
#endif
} // namespace physx
} // namespace sapien
