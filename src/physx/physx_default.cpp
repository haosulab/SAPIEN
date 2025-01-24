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
#include "sapien/physx/physx_default.h"
#include "sapien/physx/material.h"
#include "sapien/physx/physx_system.h"

namespace sapien {
namespace physx {

static float gStaticFriction{0.3};
static float gDynamicFriction{0.3};
static float gRestitution{0.1};
static std::weak_ptr<PhysxMaterial> gDefaultMaterial;
static bool gGPUEnabled{false};
static PhysxSceneConfig gSceneConfig{};
static PhysxBodyConfig gBodyConfig{};
static PhysxShapeConfig gShapeConfig{};
static PhysxSDFShapeConfig gSDFConfig{};

static ::physx::PxgDynamicsMemoryConfig gGpuMemoryConfig{};

void PhysxDefault::SetDefaultMaterial(float staticFriction, float dynamicFriction,
                                      float restitution) {
  gDefaultMaterial.reset();
  gStaticFriction = staticFriction;
  gDynamicFriction = dynamicFriction;
  gRestitution = restitution;
}

std::shared_ptr<PhysxMaterial> PhysxDefault::GetDefaultMaterial() {
  auto m = gDefaultMaterial.lock();
  if (m) {
    return m;
  }
  gDefaultMaterial = m =
      std::make_shared<PhysxMaterial>(gStaticFriction, gDynamicFriction, gRestitution);
  return m;
}

void PhysxDefault::setGpuMemoryConfig(uint32_t tempBufferCapacity, uint32_t maxRigidContactCount,
                                      uint32_t maxRigidPatchCount, uint32_t heapCapacity,
                                      uint32_t foundLostPairsCapacity,
                                      uint32_t foundLostAggregatePairsCapacity,
                                      uint32_t totalAggregatePairsCapacity,
                                      uint32_t collisionStackSize) {
  gGpuMemoryConfig.tempBufferCapacity = tempBufferCapacity;
  gGpuMemoryConfig.maxRigidContactCount = maxRigidContactCount;
  gGpuMemoryConfig.maxRigidPatchCount = maxRigidPatchCount;
  gGpuMemoryConfig.heapCapacity = heapCapacity;
  gGpuMemoryConfig.foundLostPairsCapacity = foundLostPairsCapacity;
  gGpuMemoryConfig.foundLostAggregatePairsCapacity = foundLostAggregatePairsCapacity;
  gGpuMemoryConfig.totalAggregatePairsCapacity = totalAggregatePairsCapacity;
  gGpuMemoryConfig.collisionStackSize = collisionStackSize;
}

::physx::PxgDynamicsMemoryConfig const &PhysxDefault::getGpuMemoryConfig() {
  return gGpuMemoryConfig;
}

void PhysxDefault::setSceneConfig(Vec3 gravity, float bounceThreshold, bool enablePCM,
                                  bool enableTGS, bool enableCCD, bool enableEnhancedDeterminism,
                                  bool enableFrictionEveryIteration, uint32_t cpuWorkers) {
  gSceneConfig.gravity = gravity;
  gSceneConfig.bounceThreshold = bounceThreshold;
  gSceneConfig.enablePCM = enablePCM;
  gSceneConfig.enableTGS = enableTGS;
  gSceneConfig.enableCCD = enableCCD;
  gSceneConfig.enableEnhancedDeterminism = enableEnhancedDeterminism;
  gSceneConfig.enableFrictionEveryIteration = enableFrictionEveryIteration;
  gSceneConfig.cpuWorkers = cpuWorkers;
}
void PhysxDefault::setSceneConfig(PhysxSceneConfig const &config) { gSceneConfig = config; }
PhysxSceneConfig const &PhysxDefault::getSceneConfig() { return gSceneConfig; }

void PhysxDefault::setBodyConfig(uint32_t solverIterations, uint32_t solverVelocityIterations,
                                 float sleepThreshold) {
  gBodyConfig.solverPositionIterations = solverIterations;
  gBodyConfig.solverVelocityIterations = solverVelocityIterations;
  gBodyConfig.sleepThreshold = sleepThreshold;
}
void PhysxDefault::setBodyConfig(PhysxBodyConfig const &config) { gBodyConfig = config; }
PhysxBodyConfig const &PhysxDefault::getBodyConfig() { return gBodyConfig; }

void PhysxDefault::setShapeConfig(float contactOffset, float restOffset) {
  gShapeConfig.contactOffset = contactOffset;
  gShapeConfig.restOffset = restOffset;
}
void PhysxDefault::setShapeConfig(PhysxShapeConfig const &config) { gShapeConfig = config; }
PhysxShapeConfig const &PhysxDefault::getShapeConfig() { return gShapeConfig; }

void PhysxDefault::EnableGPU() {
  if (PhysxEngine::GetIfExists()) {
    throw std::runtime_error(
        "GPU PhysX can only be enabled once before any other code involving PhysX");
  }
  gGPUEnabled = true;
}

void PhysxDefault::setSDFShapeConfig(float spacing, uint32_t subgridSize,
                                     uint32_t numThreadsForConstruction) {
  gSDFConfig.spacing = spacing;
  gSDFConfig.subgridSize = subgridSize;
  gSDFConfig.numThreadsForConstruction = numThreadsForConstruction;
}
void PhysxDefault::setSDFShapeConfig(PhysxSDFShapeConfig const &c) { gSDFConfig = c; }
PhysxSDFShapeConfig PhysxDefault::getSDFShapeConfig() { return gSDFConfig; }

bool PhysxDefault::GetGPUEnabled() { return gGPUEnabled; }
std::string PhysxDefault::getPhysxVersion() { return PHYSX_VERSION; }

} // namespace physx
} // namespace sapien
