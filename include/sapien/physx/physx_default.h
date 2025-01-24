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
#pragma once
#include "sapien/math/vec3.h"
#include <cstdint>
#include <memory>
#include <string>

namespace physx {
struct PxgDynamicsMemoryConfig;
};

namespace sapien {
namespace physx {
class PhysxMaterial;

struct PhysxSceneConfig {
  Vec3 gravity = {0, 0, -9.81};           // default gravity
  float bounceThreshold = 2.f;            // relative velocity below this will not bounce
  bool enablePCM = true;                  // Use persistent contact manifold solver for contact
  bool enableTGS = true;                  // use TGS solver
  bool enableCCD = false;                 // use continuous collision detection
  bool enableEnhancedDeterminism = false; // improve determinism
  bool enableFrictionEveryIteration =
      true;                // better friction calculation, recommended for robotics
  uint32_t cpuWorkers = 0; // CPU workers, 0 for using main thread
};

struct PhysxBodyConfig {
  uint32_t solverPositionIterations = 10; // solver position iterations, helps reduce jittering
  uint32_t solverVelocityIterations = 1;  // solver velocity iterations
  float sleepThreshold = 0.005f;          // put to sleep if (kinetic energy/(mass) falls below
};

struct PhysxShapeConfig {
  float contactOffset = 0.01f; // how close should contacts be generated
  float restOffset = 0.f;
};

struct PhysxSDFShapeConfig {
  float spacing = 0.01f;
  uint32_t subgridSize = 6;
  uint32_t numThreadsForConstruction = 4;
};

class PhysxDefault {
public:
  static std::shared_ptr<PhysxMaterial> GetDefaultMaterial();
  static void SetDefaultMaterial(float staticFriction, float dynamicFriction, float restitution);
  static void setGpuMemoryConfig(uint32_t tempBufferCapacity, uint32_t maxRigidContactCount,
                                 uint32_t maxRigidPatchCount, uint32_t heapCapacity,
                                 uint32_t foundLostPairsCapacity,
                                 uint32_t foundLostAggregatePairsCapacity,
                                 uint32_t totalAggregatePairsCapacity,
                                 uint32_t collisionStackSize);
  static ::physx::PxgDynamicsMemoryConfig const &getGpuMemoryConfig();

  static void setSceneConfig(Vec3 gravity, float bounceThreshold, bool enablePCM, bool enableTGS,
                             bool enableCCD, bool enableEnhancedDeterminism,
                             bool enableFrictionEveryIteration, uint32_t cpuWorkers);
  static void setSceneConfig(PhysxSceneConfig const &);
  static PhysxSceneConfig const &getSceneConfig();

  static void setBodyConfig(uint32_t solverIterations, uint32_t solverVelocityIterations,
                            float sleepThreshold);
  static void setBodyConfig(PhysxBodyConfig const &);
  static PhysxBodyConfig const &getBodyConfig();

  static void setShapeConfig(float contactOffset, float restOffset);
  static void setShapeConfig(PhysxShapeConfig const &);
  static PhysxShapeConfig const &getShapeConfig();

  static void setSDFShapeConfig(float spacing, uint32_t subgridSize,
                                uint32_t numThreadsForConstruction);
  static void setSDFShapeConfig(PhysxSDFShapeConfig const &);
  static PhysxSDFShapeConfig getSDFShapeConfig();

  // enable GPU simulation, may not be disabled
  static void EnableGPU();
  static bool GetGPUEnabled();

  static std::string getPhysxVersion();
};

} // namespace physx
} // namespace sapien
