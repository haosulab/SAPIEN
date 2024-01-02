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
  int computeArticulationMaxDof() const;

  /** get articulation max link count directly from PhysX */
  int computeArticulationMaxLinkCount() const;

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
  bool isInitialized() const { return mGpuInitialized; }
  void checkGpuInitialized() const;

  /** Set the CUDA stream for all GPU operations.
   *  gpuQuery* and gpuApply* will be synchronized with the stream
   *  If not set, gpuQuery* and gpuApply* synchronizes with the default stream */
  void gpuSetCudaStream(uintptr_t stream);

  // int gpuQueryContacts(CudaArrayHandle const &data) const;

  /** handle to the pose-vel buffer for rigid dynamic bodies and links */
  CudaArrayHandle gpuGetRigidBodyCudaHandle() const { return mCudaRigidBodyBuffer.handle(); }
  CudaArrayHandle gpuGetRigidDynamicCudaHandle() const { return mCudaRigidDynamicHandle; }
  CudaArrayHandle gpuGetArticulationLinkCudaHandle() const { return mCudaLinkHandle; }

  CudaArrayHandle gpuGetArticulationQposCudaHandle() const { return mCudaQposHandle; }
  CudaArrayHandle gpuGetArticulationQvelCudaHandle() const { return mCudaQTargetVelHandle; }
  CudaArrayHandle gpuGetArticulationQaccCudaHandle() const { return mCudaQaccHandle; }
  CudaArrayHandle gpuGetArticulationQfCudaHandle() const { return mCudaQfHandle; }

  CudaArrayHandle gpuGetArticulationQTargetPosCudaHandle() const { return mCudaQTargetPosHandle; }
  CudaArrayHandle gpuGetArticulationQTargetVelCudaHandle() const { return mCudaQTargetVelHandle; }

  void gpuFetchRigidDynamicData();
  void gpuFetchArticulationLinkPose();
  void gpuFetchArticulationLinkVel();
  void gpuFetchArticulationQpos();
  void gpuFetchArticulationQvel();
  void gpuFetchArticulationQacc();
  void gpuFetchArticulationQTargetPos();
  void gpuFetchArticulationQTargetVel();

  void gpuApplyRigidDynamicData(CudaArrayHandle const &indices);
  void gpuApplyArticulationRootPose(CudaArrayHandle const &indices);
  void gpuApplyArticulationRootVel(CudaArrayHandle const &indices);
  void gpuApplyArticulationQpos(CudaArrayHandle const &indices);
  void gpuApplyArticulationQvel(CudaArrayHandle const &indices);
  void gpuApplyArticulationQf(CudaArrayHandle const &indices);
  void gpuApplyArticulationQTargetPos(CudaArrayHandle const &indices);
  void gpuApplyArticulationQTargetVel(CudaArrayHandle const &indices);

  void gpuApplyRigidDynamicData();
  void gpuApplyArticulationRootPose();
  void gpuApplyArticulationRootVel();
  void gpuApplyArticulationQpos();
  void gpuApplyArticulationQvel();
  void gpuApplyArticulationQf();
  void gpuApplyArticulationQTargetPos();
  void gpuApplyArticulationQTargetVel();

  void gpuUpdateArticulationKinematics();

  void syncPosesGpuToCpu();

  void setSceneOffset(std::shared_ptr<Scene> scene, Vec3 offset);
  Vec3 getSceneOffset(std::shared_ptr<Scene> scene) const;

private:
  // TODO do weak pointer clean up
  std::map<std::weak_ptr<Scene>, Vec3, std::owner_less<>> mSceneOffset;

  std::set<std::shared_ptr<PhysxRigidDynamicComponent>, comp_cmp> mRigidDynamicComponents;
  std::set<std::shared_ptr<PhysxRigidStaticComponent>, comp_cmp> mRigidStaticComponents;
  std::set<std::shared_ptr<PhysxArticulationLinkComponent>, comp_cmp> mArticulationLinkComponents;

  uint64_t mTotalSteps;

  bool mGpuInitialized{false};

  // cache values updated in gpuInit
  int mGpuArticulationCount{-1};
  int mGpuArticulationMaxDof{-1};
  int mGpuArticulationMaxLinkCount{-1};

  CudaEvent mCudaEventRecord;
  CudaEvent mCudaEventWait;
  cudaStream_t mCudaStream{0};

  CudaArray mCudaRigidDynamicScratch;
  CudaArray mCudaLinkPoseScratch;
  CudaArray mCudaLinkVelScratch;
  CudaArray mCudaRigidDynamicIndexScratch;
  CudaArray mCudaArticulationIndexScratch;

  void allocateCudaBuffers();

  // indx buffer for all rigid dynamic bodies
  CudaArray mCudaRigidDynamicIndexBuffer;
  CudaArray mCudaRigidDynamicOffsetBuffer;

  // index buffer for all articulations
  CudaArray mCudaArticulationIndexBuffer;
  CudaArray mCudaArticulationOffsetBuffer;

  CudaArray mCudaRigidBodyBuffer;
  CudaArrayHandle mCudaRigidDynamicHandle;
  CudaArrayHandle mCudaLinkHandle;

  CudaHostArray mCudaHostRigidBodyBuffer;

  CudaArray mCudaArticulationBuffer;
  CudaArrayHandle mCudaQposHandle;
  CudaArrayHandle mCudaQvelHandle;
  CudaArrayHandle mCudaQfHandle;
  CudaArrayHandle mCudaQaccHandle;
  CudaArrayHandle mCudaQTargetPosHandle;
  CudaArrayHandle mCudaQTargetVelHandle;
};

#endif

} // namespace physx
} // namespace sapien
