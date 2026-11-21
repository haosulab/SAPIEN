#include "sapien/physx/physx_system.h"

#ifdef SAPIEN_CUDA
#include "physx_system.cuh"
#include "physx_system_indexed.cuh"
#include <cuda_runtime.h>
#include <unordered_set>

using namespace physx;
namespace sapien::physx {
namespace {
// Own staging until the Direct GPU API has finished. Indexed operations are
// synchronous reset operations; the existing no-argument paths stay unchanged.
struct Selection {
  CudaArray rows;
  CudaArray ids;
  int count;
  cudaStream_t stream;

  CudaArray gather(CudaArrayHandle source) const {
    auto shape = source.shape;
    shape[0] = count;
    CudaArray result(shape, source.type);
    int rowBytes = result.handle().strides[0];
    gather_rows(result.ptr, source.ptr, static_cast<int *>(rows.ptr), count,
                rowBytes, source.strides[0], stream);
    checkCudaErrors(cudaGetLastError());
    return result;
  }

  Selection(std::vector<int> const &indices, CudaArray const &allIds, cudaStream_t s)
      : count(indices.size()), stream(s) {
    if (indices.size() > size_t(allIds.shape[0])) {
      throw std::invalid_argument("GPU apply selection has more rows than the source buffer");
    }
    std::unordered_set<int> seen;
    for (int row : indices) {
      if (row < 0 || row >= allIds.shape[0]) {
        throw std::out_of_range("GPU apply index is outside the SAPIEN buffer rows");
      }
      if (!seen.insert(row).second) {
        throw std::invalid_argument("GPU apply indices must be unique");
      }
    }
    rows = CudaArray({count}, "i4");
    checkCudaErrors(cudaMemcpyAsync(rows.ptr, indices.data(), count * sizeof(int),
                                    cudaMemcpyHostToDevice, stream));
    // The caller's CPU list need not outlive this method.
    checkCudaErrors(cudaStreamSynchronize(stream));
    ids = gather(allIds.handle());
  }
};

void checkApply(bool ok) {
  if (!ok) {
    throw std::runtime_error("PhysX Direct GPU API rejected indexed apply");
  }
}
} // namespace

void PhysxSystemGpu::checkGpuIdle() const {
  checkGpuInitialized();
  if (mStepInProgress) {
    throw std::runtime_error("Finish the pending physics step before accessing GPU state");
  }
}

void PhysxSystemGpu::gpuApplyBodyDataIndexed(std::vector<int> const &indices, bool articulation) {
  checkGpuIdle();
  if (indices.empty()) {
    return;
  }
  ensureCudaDevice();
  Selection selected(indices, articulation ? mCudaArticulationIndexBuffer : mCudaRigidDynamicIndexBuffer,
                     mCudaStream);
  auto source = articulation ? mCudaLinkDataHandle : mCudaRigidDynamicDataHandle;
  if (articulation) {
    // Only the root (link 0), retaining the full articulation's row stride.
    source.shape = {source.shape[0], 13};
    source.strides = {source.strides[0], int(sizeof(float))};
  }
  auto state = selected.gather(source);
  auto offset = selected.gather((articulation ? mCudaArticulationOffsetBuffer :
                                              mCudaRigidDynamicOffsetBuffer).handle());
  CudaArray pose({selected.count, 7}, "f4"), linear({selected.count, 3}, "f4"),
      angular({selected.count, 3}, "f4");
  body_data_sapien_to_physx(static_cast<SapienBodyData *>(state.ptr),
                           static_cast<PhysxPose *>(pose.ptr), static_cast<Vec3 *>(linear.ptr),
                           static_cast<Vec3 *>(angular.ptr), static_cast<Vec3 *>(offset.ptr),
                           selected.count, mCudaStream);
  checkCudaErrors(cudaGetLastError());
  mCudaEventRecord.record(mCudaStream);
  auto &api = mPxScene->getDirectGPUAPI();
  auto ids = static_cast<PxU32 const *>(selected.ids.ptr);
  // A null finish event makes each call wait for completion, including staging reads.
  if (articulation) {
    checkApply(api.setArticulationData(pose.ptr, ids, PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE,
                                       selected.count, mCudaEventRecord.event));
    checkApply(api.setArticulationData(linear.ptr, ids, PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY,
                                       selected.count, mCudaEventRecord.event));
    checkApply(api.setArticulationData(angular.ptr, ids, PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY,
                                       selected.count, mCudaEventRecord.event));
  } else {
    checkApply(api.setRigidDynamicData(pose.ptr, ids, PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE,
                                       selected.count, mCudaEventRecord.event));
    checkApply(api.setRigidDynamicData(linear.ptr, ids, PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY,
                                       selected.count, mCudaEventRecord.event));
    checkApply(api.setRigidDynamicData(angular.ptr, ids, PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY,
                                       selected.count, mCudaEventRecord.event));
  }
  mApplyPositionWithoutStep = true;
}

void PhysxSystemGpu::gpuApplyRowsIndexed(std::vector<int> const &indices, CudaArrayHandle source,
                                        bool articulation, int type) {
  checkGpuIdle();
  if (indices.empty()) {
    return;
  }
  ensureCudaDevice();
  Selection selected(indices, articulation ? mCudaArticulationIndexBuffer : mCudaRigidDynamicIndexBuffer,
                     mCudaStream);
  auto data = selected.gather(source);
  if (!data.ptr) { // A scene of zero-DOF articulations has empty joint rows.
    checkCudaErrors(cudaStreamSynchronize(mCudaStream));
    return;
  }
  mCudaEventRecord.record(mCudaStream);
  auto ids = static_cast<PxU32 const *>(selected.ids.ptr);
  auto &api = mPxScene->getDirectGPUAPI();
  if (articulation) {
    checkApply(api.setArticulationData(data.ptr, ids, static_cast<PxArticulationGPUAPIWriteType::Enum>(type),
                                       selected.count, mCudaEventRecord.event));
  } else {
    checkApply(api.setRigidDynamicData(data.ptr, ids, static_cast<PxRigidDynamicGPUAPIWriteType::Enum>(type),
                                       selected.count, mCudaEventRecord.event));
  }
}

void PhysxSystemGpu::gpuApplyRigidDynamicDataIndexed(std::vector<int> const &indices) {
  gpuApplyBodyDataIndexed(indices, false);
}
void PhysxSystemGpu::gpuApplyArticulationRootDataIndexed(std::vector<int> const &indices) {
  gpuApplyBodyDataIndexed(indices, true);
}
void PhysxSystemGpu::gpuApplyRigidDynamicForceIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaRigidDynamicForceHandle, false, PxRigidDynamicGPUAPIWriteType::eFORCE);
}
void PhysxSystemGpu::gpuApplyRigidDynamicTorqueIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaRigidDynamicTorqueHandle, false, PxRigidDynamicGPUAPIWriteType::eTORQUE);
}
void PhysxSystemGpu::gpuApplyLinkForceIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaLinkForceHandle, true, PxArticulationGPUAPIWriteType::eLINK_FORCE);
}
void PhysxSystemGpu::gpuApplyLinkTorqueIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaLinkTorqueHandle, true, PxArticulationGPUAPIWriteType::eLINK_TORQUE);
}
void PhysxSystemGpu::gpuApplyArticulationQposIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaQposHandle, true, PxArticulationGPUAPIWriteType::eJOINT_POSITION);
  if (!indices.empty()) {
    mApplyPositionWithoutStep = true;
  }
}
void PhysxSystemGpu::gpuApplyArticulationQvelIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaQvelHandle, true, PxArticulationGPUAPIWriteType::eJOINT_VELOCITY);
}
void PhysxSystemGpu::gpuApplyArticulationQfIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaQfHandle, true, PxArticulationGPUAPIWriteType::eJOINT_FORCE);
}
void PhysxSystemGpu::gpuApplyArticulationQTargetPosIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaQTargetPosHandle, true, PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION);
}
void PhysxSystemGpu::gpuApplyArticulationQTargetVelIndexed(std::vector<int> const &indices) {
  gpuApplyRowsIndexed(indices, mCudaQTargetVelHandle, true, PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY);
}

void PhysxSystemGpu::gpuUpdateArticulationKinematicsIndexed(std::vector<int> const &indices) {
  checkGpuIdle();
  if (indices.empty()) {
    return;
  }
  ensureCudaDevice();
  Selection selected(indices, mCudaArticulationIndexBuffer, mCudaStream);
  mCudaEventRecord.record(mCudaStream);
  checkApply(mPxScene->getDirectGPUAPI().computeArticulationData(
      nullptr, static_cast<PxU32 const *>(selected.ids.ptr),
      PxArticulationGPUAPIComputeType::eUPDATE_KINEMATIC, selected.count, mCudaEventRecord.event));
}
} // namespace sapien::physx
#endif
