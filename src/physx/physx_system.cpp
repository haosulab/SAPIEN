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
  mInitialized = false;
}
void PhysxSystemGpu::registerComponent(std::shared_ptr<PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.insert(component);
  mInitialized = false;
}
void PhysxSystemGpu::registerComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.insert(component);
  mInitialized = false;
}
void PhysxSystemGpu::unregisterComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.erase(component);
  mInitialized = false;
}
void PhysxSystemGpu::unregisterComponent(std::shared_ptr<PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.erase(component);
  mInitialized = false;
}
void PhysxSystemGpu::unregisterComponent(
    std::shared_ptr<PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.erase(component);
  mInitialized = false;
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
  if (!mInitialized) {
    throw std::runtime_error("failed to step: gpu simulation is not initialized.");
  }

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

int PhysxSystem::getArticulationMaxDof() const {
  int result = 0;
  uint32_t count = getPxScene()->getNbArticulations();
  std::vector<PxArticulationReducedCoordinate *> articulations(count);
  getPxScene()->getArticulations(articulations.data(), count);
  for (auto a : articulations) {
    result = std::max(result, static_cast<int>(a->getDofs()));
  }
  return result;
}

int PhysxSystem::getArticulationMaxLinkCount() const {
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
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

  mGpuArticulationCount = getArticulationCount();
  mGpuArticulationMaxDof = getArticulationMaxDof();
  mGpuArticulationMaxLinkCount = getArticulationMaxLinkCount();
  mInitialized = true;

  mCudaEventRecord.init();
  mCudaEventWait.init();
}

void PhysxSystemGpu::checkGpuInitialized() const {
  if (!mInitialized) {
    throw std::runtime_error("Gpu PhysX is not initialized.");
  }
}

void PhysxSystemGpu::gpuSetCudaStream(uintptr_t stream) {
  mCudaStream = reinterpret_cast<cudaStream_t>(stream);
}

void PhysxSystemGpu::gpuFillBodyIndices(
    CudaArrayHandle const &index,
    std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies) {
  index.checkCongiguous();
  index.checkShape({static_cast<int>(bodies.size()), sizeof(PxGpuActorPair) / sizeof(int)});
  index.checkStride({sizeof(PxGpuActorPair), sizeof(int)});

  std::vector<PxGpuActorPair> host_index;
  host_index.reserve(bodies.size());
  for (uint32_t i = 0; i < bodies.size(); ++i) {
    auto actor = bodies[i]->getPxActor();

    PxGpuActorPair pair;
    memset(&pair, 0, sizeof(pair));
    pair.srcIndex = i;
    pair.nodeIndex = actor->getInternalIslandNodeIndex();

    host_index.push_back(pair);
  }

  checkCudaErrors(cudaMemcpy(index.ptr, host_index.data(),
                             sizeof(PxGpuActorPair) * host_index.size(), cudaMemcpyHostToDevice));
}

CudaArray PhysxSystemGpu::gpuCreateBodyIndices(
    std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies) {
  CudaArray array({static_cast<int>(bodies.size()), 4}, "i4");
  gpuFillBodyIndices(array.handle(), bodies);
  return array;
}

void PhysxSystemGpu::gpuFillBodyOffsets(
    CudaArrayHandle const &offset,
    std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies) {
  offset.checkCongiguous();
  offset.checkShape({static_cast<int>(bodies.size()), 4});
  offset.checkStride({4 * sizeof(float), sizeof(float)});

  std::vector<std::array<float, 4>> host_offset;
  host_offset.reserve(bodies.size());
  for (uint32_t i = 0; i < bodies.size(); ++i) {
    Vec3 offset = getSceneOffset(bodies[i]->getScene());
    host_offset.push_back({offset.x, offset.y, offset.z, 0.f});
  }

  checkCudaErrors(cudaMemcpy(offset.ptr, host_offset.data(),
                             4 * sizeof(float) * host_offset.size(), cudaMemcpyHostToDevice));
}

CudaArray PhysxSystemGpu::gpuCreateBodyOffsets(
    std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies) {
  CudaArray offsets({static_cast<int>(bodies.size()), 4}, "f4");
  gpuFillBodyOffsets(offsets.handle(), bodies);
  return offsets;
}

void PhysxSystemGpu::gpuFillArticulationOffsets(CudaArrayHandle const &offset) {
  offset.checkCongiguous();
  offset.checkShape({mGpuArticulationCount, 4});
  offset.checkStride({4 * sizeof(float), sizeof(float)});

  std::vector<std::array<float, 4>> host_offset(mGpuArticulationCount, {0.f, 0.f, 0.f, 0.f});
  for (auto a : getArticulationLinkComponents()) {
    if (a->isRoot()) {
      Vec3 offset = getSceneOffset(a->getScene());
      uint32_t idx = a->getArticulation()->getPxArticulation()->getGpuArticulationIndex();
      host_offset.at(idx) = {offset.x, offset.y, offset.z, 0.f};
    }
  }

  checkCudaErrors(cudaMemcpy(offset.ptr, host_offset.data(),
                             4 * sizeof(float) * host_offset.size(), cudaMemcpyHostToDevice));
}

CudaArray PhysxSystemGpu::gpuCreateArticulationOffsets() {
  CudaArray offsets({static_cast<int>(mGpuArticulationCount), 4}, "f4");
  gpuFillArticulationOffsets(offsets.handle());
  return offsets;
}

CudaArray PhysxSystemGpu::gpuCreateBodyDataBuffer(uint32_t count) {
  return CudaArray({static_cast<int>(count), sizeof(PxGpuBodyData) / sizeof(float)}, "f4");
}

void PhysxSystemGpu::gpuFillArticulationIndices(
    CudaArrayHandle const &index,
    std::vector<std::shared_ptr<PhysxArticulation>> const &articulations) const {
  assert(index.type == "i4");
  assert(index.shape.size() == 1);
  assert(index.shape.at(0) == articulations.size());
  assert(index.isContiguous());

  std::vector<int> hostIndex;
  hostIndex.reserve(articulations.size());
  for (auto a : articulations) {
    hostIndex.push_back(a->getPxArticulation()->getGpuArticulationIndex());
  }
  checkCudaErrors(cudaMemcpy(index.ptr, hostIndex.data(), sizeof(float) * hostIndex.size(),
                             cudaMemcpyHostToDevice));
}

CudaArray PhysxSystemGpu::gpuCreateArticulationIndices(
    std::vector<std::shared_ptr<PhysxArticulation>> const &articulations) const {
  CudaArray index({static_cast<int>(articulations.size())}, "i4");
  gpuFillArticulationIndices(index.handle(), articulations);
  return index;
}

CudaArray PhysxSystemGpu::gpuCreateArticulationQBuffer() const {
  return CudaArray({static_cast<int>(mPxScene->getNbArticulations()), getArticulationMaxDof()},
                   "f4");
}

CudaArray PhysxSystemGpu::gpuCreateArticulationRootPoseBuffer() const {
  return CudaArray(
      {static_cast<int>(mPxScene->getNbArticulations()), sizeof(PxTransform) / sizeof(float)},
      "f4");
}

CudaArray PhysxSystemGpu::gpuCreateArticulationRootVelocityBuffer() const {
  return CudaArray({static_cast<int>(mPxScene->getNbArticulations()), 8}, "f4");
}

void PhysxSystemGpu::gpuQueryBodyDataRaw(CudaArrayHandle const &data,
                                         CudaArrayHandle const &index) const {
  checkGpuInitialized();

  data.checkCongiguous();
  data.checkShape({-1, sizeof(PxGpuBodyData) / sizeof(float)});
  data.checkStride({sizeof(PxGpuBodyData), sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1, sizeof(PxGpuActorPair) / sizeof(float)});
  index.checkStride({sizeof(PxGpuActorPair), 4});

  mPxScene->copyBodyData((PxGpuBodyData *)data.ptr, (PxGpuActorPair *)index.ptr, index.shape.at(0),
                         mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryBodyData(CudaArrayHandle const &data, CudaArrayHandle const &index,
                                      CudaArrayHandle const &offset) const {
  gpuQueryBodyDataRaw(data, index);
  body_data_physx_to_sapien_subtract_offset(data.ptr, index.ptr, offset.ptr, index.shape.at(0),
                                            mCudaStream);
}

void PhysxSystemGpu::gpuApplyBodyData(CudaArrayHandle const &data, CudaArrayHandle const &index,
                                      CudaArrayHandle const &offset) const {
  data.checkCongiguous();
  data.checkShape({-1, sizeof(PxGpuBodyData) / sizeof(float)});
  data.checkStride({sizeof(PxGpuBodyData), sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1, sizeof(PxGpuActorPair) / sizeof(float)});
  index.checkStride({sizeof(PxGpuActorPair), 4});

  body_data_sapien_to_physx_add_offset(data.ptr, index.ptr, offset.ptr, index.shape.at(0),
                                       mCudaStream);

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyActorData(data.ptr, (PxGpuActorPair *)index.ptr, PxActorCacheFlag::eACTOR_DATA,
                           index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyBodyForce(CudaArrayHandle const &data,
                                       CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({-1, 4});
  data.checkStride({4 * sizeof(float), sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1, sizeof(PxGpuActorPair) / sizeof(float)});
  index.checkStride({sizeof(PxGpuActorPair), 4});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyActorData(data.ptr, (PxGpuActorPair *)index.ptr, PxActorCacheFlag::eFORCE,
                           index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyBodyTorque(CudaArrayHandle const &data,
                                        CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({-1, 4});
  data.checkStride({4 * sizeof(float), sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1, sizeof(PxGpuActorPair) / sizeof(float)});
  index.checkStride({sizeof(PxGpuActorPair), 4});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyActorData(data.ptr, (PxGpuActorPair *)index.ptr, PxActorCacheFlag::eTORQUE,
                           index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationQpos(CudaArrayHandle const &data,
                                              CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mPxScene->copyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eJOINT_POSITION,
                                 index.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationQvel(CudaArrayHandle const &data,
                                              CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mPxScene->copyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eJOINT_VELOCITY,
                                 index.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationDrivePos(CudaArrayHandle const &data,
                                                  CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mPxScene->copyArticulationData(data.ptr, index.ptr,
                                 PxArticulationGpuDataType::eJOINT_TARGET_POSITION,
                                 index.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationDriveVel(CudaArrayHandle const &data,
                                                  CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mPxScene->copyArticulationData(data.ptr, index.ptr,
                                 PxArticulationGpuDataType::eJOINT_TARGET_VELOCITY,
                                 index.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQpos(CudaArrayHandle const &data,
                                              CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eJOINT_POSITION,
                                  index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationQvel(CudaArrayHandle const &data,
                                              CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eJOINT_VELOCITY,
                                  index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationDrivePos(CudaArrayHandle const &data,
                                                  CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(data.ptr, index.ptr,
                                  PxArticulationGpuDataType::eJOINT_TARGET_POSITION,
                                  index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationDriveVel(CudaArrayHandle const &data,
                                                  CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, mGpuArticulationMaxDof});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(data.ptr, index.ptr,
                                  PxArticulationGpuDataType::eJOINT_TARGET_VELOCITY,
                                  index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationRootPoseRaw(CudaArrayHandle const &data,
                                                     CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, sizeof(PxTransform) / sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mPxScene->copyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eROOT_TRANSFORM,
                                 index.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationRootPose(CudaArrayHandle const &data,
                                                  CudaArrayHandle const &index,
                                                  CudaArrayHandle const &offset) const {
  gpuQueryArticulationRootPoseRaw(data, index);
  transform_physx_to_sapien_subtract_offset(data.ptr, index.ptr, offset.ptr, index.shape.at(0),
                                            mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationRootPose(CudaArrayHandle const &data,
                                                  CudaArrayHandle const &index,
                                                  CudaArrayHandle const &offset) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, sizeof(PxTransform) / sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  transform_sapien_to_physx_add_offset(data.ptr, index.ptr, offset.ptr, index.shape.at(0),
                                       mCudaStream);
  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eROOT_TRANSFORM,
                                  index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuQueryArticulationRootVelocity(CudaArrayHandle const &data,
                                                      CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, 8});
  data.checkStride({8 * sizeof(float), sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mPxScene->copyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eROOT_VELOCITY,
                                 index.shape.at(0), mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuApplyArticulationRootVelocity(CudaArrayHandle const &data,
                                                      CudaArrayHandle const &index) const {
  data.checkCongiguous();
  data.checkShape({mGpuArticulationCount, 8});
  data.checkStride({8 * sizeof(float), sizeof(float)});

  index.checkCongiguous();
  index.checkShape({-1});
  index.checkStride({sizeof(int)});

  mCudaEventRecord.record(mCudaStream);
  mPxScene->applyArticulationData(data.ptr, index.ptr, PxArticulationGpuDataType::eROOT_VELOCITY,
                                  index.shape.at(0), mCudaEventRecord.event, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

void PhysxSystemGpu::gpuUpdateArticulationKinematics() const {
  mPxScene->updateArticulationsKinematic(mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
}

int PhysxSystemGpu::gpuQueryContacts(CudaArrayHandle const &data) const {
  data.checkCongiguous();
  data.checkShape({-1, sizeof(PxGpuContactPair)});
  data.checkStride({sizeof(PxGpuContactPair), 1});
  int count;
  mPxScene->copyContactData(data.ptr, data.shape.at(0), &count, mCudaEventWait.event);
  mCudaEventWait.wait(mCudaStream);
  return count;
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

} // namespace physx
} // namespace sapien
