#pragma once
#include "../array.h"
#include "../component.h"
#include "../system.h"
#include "./physx_engine.h"
#include "mesh_manager.h"
#include "sapien/scene.h"
#include "scene_query.h"
#include "simulation_callback.hpp"
#include <PxPhysicsAPI.h>
#include <memory>
#include <set>

#ifdef SAPIEN_CUDA
#include "sapien/utils/cuda.h"
#endif

namespace sapien {
namespace physx {
class PhysxArticulation;
class PhysxMaterial;
class PhysxRigidBodyComponent;
class PhysxRigidDynamicComponent;
class PhysxRigidStaticComponent;
class PhysxArticulationLinkComponent;

struct PhysxSceneConfig {
  Vec3 gravity = {0, 0, -9.81};           // default gravity
  float bounceThreshold = 2.f;            // relative velocity below this will not bounce
  float sleepThreshold = 0.005f;          // put to sleep if (kinetic energy/(mass) falls below
  float contactOffset = 0.01f;            // how close should contacts be generated
  uint32_t solverIterations = 10;         // solver position iterations, helps reduce jittering
  uint32_t solverVelocityIterations = 1;  // solver velocity iterations
  bool enablePCM = true;                  // Use persistent contact manifold solver for contact
  bool enableTGS = true;                  // use TGS solver
  bool enableCCD = false;                 // use continuous collision detection
  bool enableEnhancedDeterminism = false; // improve determinism
  bool enableFrictionEveryIteration =
      true; // better friction calculation, recommended for robotics

  template <class Archive> void serialize(Archive &ar) {
    ar(gravity, bounceThreshold, sleepThreshold, contactOffset, solverIterations,
       solverVelocityIterations, enablePCM, enableTGS, enableCCD, enableEnhancedDeterminism,
       enableFrictionEveryIteration);
  }
};

class PhysxSystem : public System {

public:
  std::shared_ptr<PhysxEngine> getEngine() const { return mEngine; }

  PhysxSceneConfig const &getSceneConfig() const { return mSceneConfig; };

  virtual ::physx::PxScene *getPxScene() const { return mPxScene; }
  virtual void registerComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) = 0;
  virtual void registerComponent(std::shared_ptr<PhysxRigidStaticComponent> component) = 0;
  virtual void registerComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) = 0;
  virtual void unregisterComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) = 0;
  virtual void unregisterComponent(std::shared_ptr<PhysxRigidStaticComponent> component) = 0;
  virtual void unregisterComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) = 0;
  virtual std::vector<std::shared_ptr<PhysxRigidDynamicComponent>>
  getRigidDynamicComponents() const = 0;
  virtual std::vector<std::shared_ptr<PhysxRigidStaticComponent>>
  getRigidStaticComponents() const = 0;
  virtual std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
  getArticulationLinkComponents() const = 0;

  void setTimestep(float step) { mTimestep = step; };
  float getTimestep() const { return mTimestep; }

  std::string getName() const override { return "physx"; }
  virtual bool isGpu() const = 0;

  /** get articulation count directly from PhysX */
  int getArticulationCount() const;

  /** get articulation max dof directly from PhysX */
  int getArticulationMaxDof() const;

  /** get articulation max link count directly from PhysX */
  int getArticulationMaxLinkCount() const;

protected:
  PhysxSystem(PhysxSceneConfig const &config);

  PhysxSceneConfig mSceneConfig;
  std::shared_ptr<PhysxEngine> mEngine;

  ::physx::PxScene *mPxScene;
  float mTimestep{0.01f};

  ::physx::PxDefaultCpuDispatcher *mPxCPUDispatcher;
};

class PhysxSystemCpu : public PhysxSystem {
public:
  PhysxSystemCpu(PhysxSceneConfig const &config);

  void registerComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) override;
  void registerComponent(std::shared_ptr<PhysxRigidStaticComponent> component) override;
  void registerComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) override;
  void unregisterComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) override;
  void unregisterComponent(std::shared_ptr<PhysxRigidStaticComponent> component) override;
  void unregisterComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) override;
  std::vector<std::shared_ptr<PhysxRigidDynamicComponent>>
  getRigidDynamicComponents() const override;
  std::vector<std::shared_ptr<PhysxRigidStaticComponent>>
  getRigidStaticComponents() const override;
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
  getArticulationLinkComponents() const override;

  std::unique_ptr<PhysxHitInfo> raycast(Vec3 const &origin, Vec3 const &direction, float distance);

  void step() override;
  bool isGpu() const override { return false; }

  std::string packState() const;
  void unpackState(std::string const &data);

  std::vector<Contact *> getContacts() const { return mSimulationCallback.getContacts(); }

private:
  DefaultEventCallback mSimulationCallback;

  std::set<std::shared_ptr<PhysxRigidDynamicComponent>, comp_cmp> mRigidDynamicComponents;
  std::set<std::shared_ptr<PhysxRigidStaticComponent>, comp_cmp> mRigidStaticComponents;
  std::set<std::shared_ptr<PhysxArticulationLinkComponent>, comp_cmp> mArticulationLinkComponents;
};

#ifdef SAPIEN_CUDA

class PhysxSystemGpu : public PhysxSystem {
public:
  PhysxSystemGpu(PhysxSceneConfig const &config);

  void registerComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) override;
  void registerComponent(std::shared_ptr<PhysxRigidStaticComponent> component) override;
  void registerComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) override;
  void unregisterComponent(std::shared_ptr<PhysxRigidDynamicComponent> component) override;
  void unregisterComponent(std::shared_ptr<PhysxRigidStaticComponent> component) override;
  void unregisterComponent(std::shared_ptr<PhysxArticulationLinkComponent> component) override;
  std::vector<std::shared_ptr<PhysxRigidDynamicComponent>>
  getRigidDynamicComponents() const override;
  std::vector<std::shared_ptr<PhysxRigidStaticComponent>>
  getRigidStaticComponents() const override;
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
  getArticulationLinkComponents() const override;

  void step() override;
  bool isGpu() const override { return true; }

  void gpuInit();
  bool isInitialized() const { return mInitialized; }

  void checkGpuInitialized() const;

  /** Set the CUDA stream for all GPU operations.
   *  gpuQuery* and gpuApply* will be synchronized with the stream
   *  If not set, gpuQuery* and gpuApply* synchronizes with the default stream */
  void gpuSetCudaStream(uintptr_t stream);

  void gpuFillBodyIndices(CudaArrayHandle const &index,
                          std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies);

  /** Create a index buffer for rigid bodies given a list rigid dynamic objects
   *  Each entry of the index buffer is 128 bits. First 32 bits represent the
   *  location in a target data buffer, it is filled with consecutive integers
   *  starting from 0. Next 32 bits are paddings. The final 64 bits are PhysX
   *  internal id used to identify a GPU rigid body. */
  CudaArray
  gpuCreateBodyIndices(std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies);

  void gpuFillArticulationIndices(
      CudaArrayHandle const &index,
      std::vector<std::shared_ptr<PhysxArticulation>> const &articulations) const;

  /** Create a index buffer for articulations given a list of articulations.
   *  The content of the buffer are a list of GPU indices of the provided articulations
   *  Each entry is a 32-bit integer articulation id */
  CudaArray gpuCreateArticulationIndices(
      std::vector<std::shared_ptr<PhysxArticulation>> const &articulations) const;

  void gpuFillBodyOffsets(CudaArrayHandle const &offset,
                          std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies);
  CudaArray
  gpuCreateBodyOffsets(std::vector<std::shared_ptr<PhysxRigidBodyComponent>> const &bodies);

  void gpuFillArticulationOffsets(CudaArrayHandle const &offset);
  CudaArray gpuCreateArticulationOffsets();

  /** Allocate a CUDA buffer capable of storing data of "count" number of rigid bodies */
  CudaArray gpuCreateBodyDataBuffer(uint32_t count);

  /** Allocate a CUDA buffer capable of storing data of ALL articulation joint data */
  CudaArray gpuCreateArticulationQBuffer() const;

  /** Allocate a CUDA buffer capable of storing data of ALL articulation root poses */
  CudaArray gpuCreateArticulationRootPoseBuffer() const;

  /** Allocate a CUDA buffer capable of storing data of ALL articulation root velocity */
  CudaArray gpuCreateArticulationRootVelocityBuffer() const;

  /** Query the pose and velocity of rigid bodies represented by the index buffer,
   *  The result is stored to the data buffer */

  void gpuQueryBodyDataRaw(CudaArrayHandle const &data, CudaArrayHandle const &index) const;
  void gpuQueryBodyData(CudaArrayHandle const &data, CudaArrayHandle const &index,
                        CudaArrayHandle const &offset) const;
  void gpuApplyBodyData(CudaArrayHandle const &data, CudaArrayHandle const &index,
                        CudaArrayHandle const &offset) const;

  void gpuApplyBodyForce(CudaArrayHandle const &data, CudaArrayHandle const &index) const;
  void gpuApplyBodyTorque(CudaArrayHandle const &data, CudaArrayHandle const &index) const;

  void gpuQueryArticulationQpos(CudaArrayHandle const &data, CudaArrayHandle const &index) const;
  void gpuQueryArticulationQvel(CudaArrayHandle const &data, CudaArrayHandle const &index) const;
  void gpuQueryArticulationDrivePos(CudaArrayHandle const &data,
                                    CudaArrayHandle const &index) const;
  void gpuQueryArticulationDriveVel(CudaArrayHandle const &data,
                                    CudaArrayHandle const &index) const;

  void gpuQueryArticulationRootPoseRaw(CudaArrayHandle const &data,
                                       CudaArrayHandle const &index) const;
  void gpuQueryArticulationRootPose(CudaArrayHandle const &data, CudaArrayHandle const &index,
                                    CudaArrayHandle const &offset) const;

  void gpuQueryArticulationRootVelocity(CudaArrayHandle const &data,
                                        CudaArrayHandle const &index) const;

  void gpuApplyArticulationQpos(CudaArrayHandle const &data, CudaArrayHandle const &index) const;
  void gpuApplyArticulationQvel(CudaArrayHandle const &data, CudaArrayHandle const &index) const;
  void gpuApplyArticulationDrivePos(CudaArrayHandle const &data,
                                    CudaArrayHandle const &index) const;
  void gpuApplyArticulationDriveVel(CudaArrayHandle const &data,
                                    CudaArrayHandle const &index) const;
  void gpuApplyArticulationRootPose(CudaArrayHandle const &data, CudaArrayHandle const &index,
                                    CudaArrayHandle const &offset) const;
  void gpuApplyArticulationRootVelocity(CudaArrayHandle const &data,
                                        CudaArrayHandle const &index) const;

  void gpuUpdateArticulationKinematics() const;

  int gpuQueryContacts(CudaArrayHandle const &data) const;

  void setSceneOffset(std::shared_ptr<Scene> scene, Vec3 offset);
  Vec3 getSceneOffset(std::shared_ptr<Scene> scene) const;

private:
  // TODO do weak pointer clean up
  std::map<std::weak_ptr<Scene>, Vec3, std::owner_less<>> mSceneOffset;

  std::set<std::shared_ptr<PhysxRigidDynamicComponent>, comp_cmp> mRigidDynamicComponents;
  std::set<std::shared_ptr<PhysxRigidStaticComponent>, comp_cmp> mRigidStaticComponents;
  std::set<std::shared_ptr<PhysxArticulationLinkComponent>, comp_cmp> mArticulationLinkComponents;

  bool mInitialized{false};

  // cache values updated in gpuInit
  int mGpuArticulationCount{-1};
  int mGpuArticulationMaxDof{-1};
  int mGpuArticulationMaxLinkCount{-1};

  CudaEvent mCudaEventRecord;
  CudaEvent mCudaEventWait;
  cudaStream_t mCudaStream{0};
};

#endif

} // namespace physx
} // namespace sapien

// CEREAL_REGISTER_TYPE(sapien::physx::PhysxSystem);
// CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::System, sapien::physx::PhysxSystem);
