#include "sapien/physx/physx_system.h"
#include "../logger.h"
#include "./filter_shader.hpp"
#include "sapien/math/conversion.h"
#include "sapien/physx/articulation.h"
#include "sapien/physx/articulation_link_component.h"
#include "sapien/physx/material.h"
#include "sapien/physx/rigid_component.h"
#include <extensions/PxExtensionsAPI.h>

#ifdef SAPIEN_CUDA
#include "./physx_system.cuh"
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

PhysxSystem::PhysxSystem(PhysxSceneConfig const &config)
    : mSceneConfig(config), mEngine(PhysxEngine::Get()) {}

PhysxSystemCpu::PhysxSystemCpu(PhysxSceneConfig const &config) : PhysxSystem(config) {
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

  mPxCPUDispatcher = PxDefaultCpuDispatcherCreate(0);
  if (!mPxCPUDispatcher) {
    throw std::runtime_error("PhysX system creation failed: failed to create CPU dispatcher");
  }
  sceneDesc.cpuDispatcher = mPxCPUDispatcher;
  mPxScene = mEngine->getPxPhysics()->createScene(sceneDesc);
  mPxScene->setSimulationEventCallback(&mSimulationCallback);
}

PhysxSystemGpu::PhysxSystemGpu(PhysxSceneConfig const &config) : PhysxSystem(config) {
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

  sceneFlags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
  sceneFlags |= PxSceneFlag::eENABLE_DIRECT_GPU_API;
  sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
  sceneDesc.cudaContextManager = mEngine->getCudaContextManager();
  if (!config.enablePCM) {
    logger::warn("PCM must be enabled when using GPU.");
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }

  sceneDesc.flags = sceneFlags;

  mPxCPUDispatcher = PxDefaultCpuDispatcherCreate(0);
  if (!mPxCPUDispatcher) {
    throw std::runtime_error("PhysX system creation failed: failed to create CPU dispatcher");
  }
  sceneDesc.cpuDispatcher = mPxCPUDispatcher;
  mPxScene = mEngine->getPxPhysics()->createScene(sceneDesc);
  // TODO: contact handling
  // mPxScene->setSimulationEventCallback(&mSimulationCallback);
}

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
  while (!mPxScene->fetchResults(true)) {
  }
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

void PhysxSystemGpu::step() {
  if (!mGpuInitialized) {
    throw std::runtime_error("failed to step: gpu simulation is not initialized.");
  }

  ++mTotalSteps;
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }
  // TODO: does the GPU API require fetch results?
}

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
    actor->setLinearVelocity(v);
    actor->setAngularVelocity(w);
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

void PhysxSystemGpu::gpuInit() {
  ++mTotalSteps;
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

  mGpuArticulationCount = getArticulationCount();
  mGpuArticulationMaxDof = computeArticulationMaxDof();
  mGpuArticulationMaxLinkCount = computeArticulationMaxLinkCount();

  allocateCudaBuffers();

  mCudaEventRecord.init();
  mCudaEventWait.init();

  mGpuInitialized = true;
}

void PhysxSystemGpu::checkGpuInitialized() const {
  if (!isInitialized()) {
    throw std::runtime_error("GPU PhysX is not initialized.");
  }
}

void PhysxSystemGpu::gpuSetCudaStream(uintptr_t stream) { mCudaStream = (cudaStream_t)stream; }

// int PhysxSystemGpu::gpuQueryContacts(CudaArrayHandle const &data) const {
//   data.checkCongiguous();
//   data.checkShape({-1, sizeof(PxGpuContactPair)});
//   data.checkStride({sizeof(PxGpuContactPair), 1});
//   int count;
//   mPxScene->copyContactData(data.ptr, data.shape.at(0), &count, mCudaEventWait.event);
//   mCudaEventWait.wait(mCudaStream);
//   return count;
// }

void PhysxSystemGpu::gpuFetchRigidDynamicData() {
  checkGpuInitialized();
  mPxScene->copyBodyData((PxGpuBodyData *)mCudaRigidDynamicScratch.ptr,
                         (PxGpuActorPair *)mCudaRigidDynamicIndexBuffer.ptr,
                         mCudaRigidDynamicIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
  body_data_physx_to_sapien(mCudaRigidDynamicHandle.ptr, mCudaRigidDynamicScratch.ptr,
                            mCudaRigidDynamicOffsetBuffer.ptr,
                            mCudaRigidDynamicIndexBuffer.shape.at(0), mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationLinkPose() {
  checkGpuInitialized();

  mPxScene->copyArticulationData(mCudaLinkPoseScratch.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eLINK_TRANSFORM,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
  link_pose_physx_to_sapien(
      mCudaLinkHandle.ptr, mCudaLinkPoseScratch.ptr, mCudaArticulationOffsetBuffer.ptr,
      mGpuArticulationMaxLinkCount,
      mCudaArticulationIndexBuffer.shape.at(0) * mGpuArticulationMaxLinkCount, mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationLinkVel() {
  checkGpuInitialized();

  mPxScene->copyArticulationData(mCudaLinkVelScratch.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eLINK_TRANSFORM,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
  link_vel_physx_to_sapien(mCudaLinkHandle.ptr, mCudaLinkVelScratch.ptr,
                           mCudaArticulationIndexBuffer.shape.at(0) * mGpuArticulationMaxLinkCount,
                           mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQpos() {
  checkGpuInitialized();
  mPxScene->copyArticulationData(mCudaQposHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eJOINT_POSITION,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQvel() {
  checkGpuInitialized();
  mPxScene->copyArticulationData(mCudaQvelHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eJOINT_VELOCITY,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQTargetPos() {
  checkGpuInitialized();
  mPxScene->copyArticulationData(mCudaQTargetPosHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eJOINT_TARGET_POSITION,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQTargetVel() {
  checkGpuInitialized();
  mPxScene->copyArticulationData(mCudaQTargetVelHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eJOINT_TARGET_VELOCITY,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuFetchArticulationQacc() {
  checkGpuInitialized();
  mPxScene->copyArticulationData(mCudaQaccHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                 PxArticulationGpuDataType::eJOINT_ACCELERATION,
                                 mCudaArticulationIndexBuffer.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuUpdateArticulationKinematics() {
  checkGpuInitialized();
  mPxScene->updateArticulationsKinematic(mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyRigidDynamicData() {
  checkGpuInitialized();

  body_data_sapien_to_physx(mCudaRigidDynamicScratch.ptr, mCudaRigidDynamicHandle.ptr,
                            mCudaRigidDynamicOffsetBuffer.ptr,
                            mCudaRigidDynamicIndexBuffer.shape.at(0), mCudaStream);

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyActorData(mCudaRigidDynamicScratch.ptr,
                           (PxGpuActorPair *)mCudaRigidDynamicIndexBuffer.ptr,
                           PxActorCacheFlag::eACTOR_DATA, mCudaRigidDynamicIndexBuffer.shape.at(0),
                           mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyRigidDynamicData(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  // gather indices to physx indices
  // gather sapien poses into physx poses
  body_data_sapien_to_physx(mCudaRigidDynamicScratch.ptr, mCudaRigidDynamicIndexScratch.ptr,
                            mCudaRigidDynamicHandle.ptr, mCudaRigidDynamicIndexBuffer.ptr,
                            indices.ptr, mCudaRigidDynamicOffsetBuffer.ptr, indices.shape.at(0),
                            mCudaStream);
  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyActorData(mCudaRigidDynamicScratch.ptr,
                           (PxGpuActorPair *)mCudaRigidDynamicIndexScratch.ptr,
                           PxActorCacheFlag::eACTOR_DATA, indices.shape.at(0),
                           mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationRootPose() {
  gpuApplyArticulationRootPose(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationRootPose(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  root_pose_sapien_to_physx(mCudaLinkPoseScratch.ptr, mCudaLinkHandle.ptr, indices.ptr,
                            mCudaArticulationOffsetBuffer.ptr, mGpuArticulationMaxLinkCount,
                            indices.shape.at(0), mCudaStream);
  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaLinkPoseScratch.ptr, indices.ptr,
                                  PxArticulationGpuDataType::eROOT_TRANSFORM, indices.shape.at(0),
                                  mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationRootVel() {
  gpuApplyArticulationRootVel(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationRootVel(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  root_vel_sapien_to_physx(mCudaLinkPoseScratch.ptr, mCudaLinkHandle.ptr, indices.ptr,
                           mGpuArticulationMaxLinkCount, indices.shape.at(0), mCudaStream);
  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaLinkPoseScratch.ptr, indices.ptr,
                                  PxArticulationGpuDataType::eROOT_VELOCITY, indices.shape.at(0),
                                  mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQpos() {
  gpuApplyArticulationQpos(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationQpos(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaQposHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                  PxArticulationGpuDataType::eJOINT_POSITION, indices.shape.at(0),
                                  mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQvel() {
  gpuApplyArticulationQvel(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationQvel(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaQvelHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                  PxArticulationGpuDataType::eJOINT_VELOCITY, indices.shape.at(0),
                                  mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQf() {
  gpuApplyArticulationQf(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationQf(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaQfHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                  PxArticulationGpuDataType::eJOINT_FORCE, indices.shape.at(0),
                                  mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQTargetPos() {
  gpuApplyArticulationQTargetPos(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationQTargetPos(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaQTargetPosHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                  PxArticulationGpuDataType::eJOINT_TARGET_POSITION,
                                  indices.shape.at(0), mCudaEventRecord.event,
                                  mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQTargetVel() {
  gpuApplyArticulationQTargetVel(mCudaArticulationIndexBuffer.handle());
}

void PhysxSystemGpu::gpuApplyArticulationQTargetVel(CudaArrayHandle const &indices) {
  checkGpuInitialized();
  indices.checkCongiguous();
  indices.checkShape({-1});
  indices.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(mCudaQTargetVelHandle.ptr, mCudaArticulationIndexBuffer.ptr,
                                  PxArticulationGpuDataType::eJOINT_TARGET_VELOCITY,
                                  indices.shape.at(0), mCudaEventRecord.event,
                                  mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::syncPosesGpuToCpu() {
  checkGpuInitialized();
  gpuFetchRigidDynamicData();
  gpuFetchArticulationLinkPose();
  if (mCudaHostRigidBodyBuffer.shape != mCudaRigidBodyBuffer.shape) {
    mCudaHostRigidBodyBuffer =
        CudaHostArray(mCudaRigidBodyBuffer.shape, mCudaRigidBodyBuffer.type);
  }
  mCudaHostRigidBodyBuffer.copyFrom(mCudaRigidBodyBuffer);
  auto data = (SapienBodyData *)mCudaHostRigidBodyBuffer.ptr;

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

void PhysxSystemGpu::setSceneOffset(std::shared_ptr<Scene> scene, Vec3 offset) {
  if (mRigidDynamicComponents.size() || mRigidStaticComponents.size() ||
      mArticulationLinkComponents.size()) {
    throw std::runtime_error(
        "Scene offset may only be changed when no bodies are added to the scene.");
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
  int rigidDynamicCount = mRigidDynamicComponents.size();
  int rigidBodyCount = rigidDynamicCount + mGpuArticulationCount * mGpuArticulationMaxLinkCount;

  mCudaRigidBodyBuffer = CudaArray({rigidBodyCount, 13}, "f4");

  mCudaRigidDynamicHandle = CudaArrayHandle{.shape = {rigidDynamicCount, 13},
                                            .strides = {52, 4},
                                            .type = "f4",
                                            .cudaId = mCudaRigidBodyBuffer.cudaId,
                                            .ptr = (float *)mCudaRigidBodyBuffer.ptr};

  mCudaLinkHandle =
      CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxLinkCount, 13},
                      .strides = {mGpuArticulationMaxLinkCount * 52, 52, 4},
                      .type = "f4",
                      .cudaId = mCudaRigidBodyBuffer.cudaId,
                      .ptr = (float *)mCudaRigidBodyBuffer.ptr + 13 * rigidDynamicCount};

  mCudaArticulationBuffer = CudaArray({mGpuArticulationCount * mGpuArticulationMaxDof * 6}, "f4");

  mCudaQposHandle = CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxDof},
                                    .strides = {mGpuArticulationMaxDof * 4, 4},
                                    .type = "f4",
                                    .cudaId = mCudaArticulationBuffer.cudaId,
                                    .ptr = mCudaArticulationBuffer.ptr};

  mCudaQvelHandle =
      CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxDof},
                      .strides = {mGpuArticulationMaxDof * 4, 4},
                      .type = "f4",
                      .cudaId = mCudaArticulationBuffer.cudaId,
                      .ptr = (float *)mCudaArticulationBuffer.ptr + mGpuArticulationMaxDof};

  mCudaQfHandle =
      CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxDof},
                      .strides = {mGpuArticulationMaxDof * 4, 4},
                      .type = "f4",
                      .cudaId = mCudaArticulationBuffer.cudaId,
                      .ptr = (float *)mCudaArticulationBuffer.ptr + mGpuArticulationMaxDof * 2};

  mCudaQaccHandle =
      CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxDof},
                      .strides = {mGpuArticulationMaxDof * 4, 4},
                      .type = "f4",
                      .cudaId = mCudaArticulationBuffer.cudaId,
                      .ptr = (float *)mCudaArticulationBuffer.ptr + mGpuArticulationMaxDof * 3};

  mCudaQTargetPosHandle =
      CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxDof},
                      .strides = {mGpuArticulationMaxDof * 4, 4},
                      .type = "f4",
                      .cudaId = mCudaArticulationBuffer.cudaId,
                      .ptr = (float *)mCudaArticulationBuffer.ptr + mGpuArticulationMaxDof * 4};

  mCudaQTargetVelHandle =
      CudaArrayHandle{.shape = {mGpuArticulationCount, mGpuArticulationMaxDof},
                      .strides = {mGpuArticulationMaxDof * 4, 4},
                      .type = "f4",
                      .cudaId = mCudaArticulationBuffer.cudaId,
                      .ptr = (float *)mCudaArticulationBuffer.ptr + mGpuArticulationMaxDof * 5};

  {
    mCudaRigidDynamicIndexBuffer = CudaArray({rigidDynamicCount, 4}, "i4");
    mCudaRigidDynamicIndexScratch = CudaArray({rigidDynamicCount, 4}, "i4");
    mCudaRigidDynamicOffsetBuffer = CudaArray({rigidDynamicCount, 3}, "f4");

    std::vector<std::array<float, 3>> host_offset;
    std::vector<PxGpuActorPair> host_index;
    auto bodies = getRigidDynamicComponents();
    for (uint32_t i = 0; i < bodies.size(); ++i) {
      // body set internal gpu id
      Vec3 offset = getSceneOffset(bodies[i]->getScene());
      host_offset.push_back({offset.x, offset.y, offset.z});

      PxGpuActorPair pair;
      memset(&pair, 0, sizeof(pair));
      pair.srcIndex = i;
      pair.nodeIndex = bodies[i]->getPxActor()->getInternalIslandNodeIndex();
      host_index.push_back(pair);

      bodies[i]->internalSetGpuIndex(i);
    }

    checkCudaErrors(cudaMemcpy(mCudaRigidDynamicIndexBuffer.ptr, host_index.data(),
                               host_offset.size() * sizeof(int) * 4, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(mCudaRigidDynamicOffsetBuffer.ptr, host_offset.data(),
                               host_offset.size() * sizeof(float) * 3, cudaMemcpyHostToDevice));
  }

  {
    mCudaArticulationOffsetBuffer = CudaArray({mGpuArticulationCount, 3}, "f4");
    std::vector<std::array<float, 3>> host_offset(mGpuArticulationCount, {0.f, 0.f, 0.f});
    for (auto a : getArticulationLinkComponents()) {
      uint32_t art_idx = a->getArticulation()->getPxArticulation()->getGpuArticulationIndex();
      if (a->isRoot()) {
        Vec3 offset = getSceneOffset(a->getScene());
        host_offset.at(art_idx) = {offset.x, offset.y, offset.z};
      }
      a->internalSetGpuPoseIndex(rigidDynamicCount + art_idx * mGpuArticulationMaxLinkCount +
                                 a->getIndex());
    }
    checkCudaErrors(cudaMemcpy(mCudaArticulationOffsetBuffer.ptr, host_offset.data(),
                               host_offset.size() * sizeof(float) * 3, cudaMemcpyHostToDevice));

    // index
    mCudaArticulationIndexBuffer = CudaArray({mGpuArticulationCount}, "i4");
    mCudaArticulationIndexScratch = CudaArray({mGpuArticulationCount}, "i4");
    std::vector<int> host_index;
    for (uint32_t i = 0; i < mGpuArticulationCount; ++i) {
      host_index.push_back(i);
    }
    checkCudaErrors(cudaMemcpy(mCudaArticulationIndexBuffer.ptr, host_index.data(),
                               host_index.size() * sizeof(int), cudaMemcpyHostToDevice));
  }

  int bodySize = rigidDynamicCount * 16 * sizeof(float);
  mCudaRigidDynamicScratch = CudaArray({bodySize}, "u1");

  int linkPoseSize = mGpuArticulationCount * mGpuArticulationMaxLinkCount * 7 * sizeof(float);
  mCudaLinkPoseScratch = CudaArray({linkPoseSize}, "u1");

  int linkVelSize = mGpuArticulationCount * mGpuArticulationMaxLinkCount * 6 * sizeof(float);
  mCudaLinkVelScratch = CudaArray({linkVelSize}, "u1");
}

} // namespace physx
} // namespace sapien
