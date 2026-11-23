#include "sapien/physx/physx_system.h"
#include "sapien/physx/rigid_component.h"
#include "sapien/physx/articulation.h"
#include "sapien/physx/articulation_link_component.h"
#include <unordered_set>
#ifdef SAPIEN_CUDA
#include <cuda_runtime.h>
#endif

namespace sapien::physx {
void PhysxSystem::resetFilteringScene(std::shared_ptr<Scene> scene) {
  if (!scene || scene->getPhysxSystem().get() != this) {
    throw std::invalid_argument("reset_filtering_scene requires a scene using this PhysX system");
  }
  std::vector<std::shared_ptr<PhysxRigidBaseComponent>> bodies;
  for (auto const &entity : scene->getEntities()) {
    for (auto const &component : entity->getComponents()) {
      if (auto body = std::dynamic_pointer_cast<PhysxRigidBaseComponent>(component)) {
        auto actor = body->getPxActor();
        if (!actor || !(actor->getActorFlags() & ::physx::PxActorFlag::eDISABLE_SIMULATION)) {
          bodies.push_back(body);
        }
      }
    }
  }
  resetFiltering(bodies);
}

void PhysxSystem::resetFiltering(
    std::vector<std::shared_ptr<PhysxRigidBaseComponent>> const &bodies) {
  // Validate the entire request before destroying any interaction.
  std::unordered_set<::physx::PxRigidActor *> seen;
  for (auto const &body : bodies) {
    auto actor = body ? body->getPxActor() : nullptr;
    if (!actor || actor->getScene() != mPxScene) {
      throw std::invalid_argument("reset_filtering requires actors belonging to this PhysX system");
    }
    if (actor->getActorFlags() & ::physx::PxActorFlag::eDISABLE_SIMULATION) {
      throw std::invalid_argument("reset_filtering requires simulation-enabled actors");
    }
    if (!seen.insert(actor).second) {
      throw std::invalid_argument("reset_filtering requires unique actors");
    }
  }
  if (bodies.empty()) {
    return;
  }
  beforeResetFiltering();
  for (auto const &body : bodies) {
    if (!mPxScene->resetFiltering(*body->getPxActor())) {
      throw std::runtime_error("PhysX rejected reset_filtering");
    }
  }
}

#ifdef SAPIEN_CUDA
std::vector<int> PhysxSystemGpu::readResetIndices(GpuIndexSelection const &selection, size_t rowCount) {
  std::vector<int> indices;
  if (auto host = std::get_if<std::vector<int>>(&selection)) {
    indices = *host;
  } else {
    auto const &array = std::get<CudaArrayHandle>(selection);
    array.checkShape({-1});
    array.checkCongiguous();
    if (array.cudaId != mDevice->cudaId || array.shape[0] < 0 || size_t(array.shape[0]) > rowCount) {
      throw std::invalid_argument("Reset index buffer must be on the system CUDA device and fit its rows");
    }
    bool int32 = array.type == "i4" || array.type == "<i4";
    bool int64 = array.type == "i8" || array.type == "<i8";
    if (!int32 && !int64) {
      throw std::invalid_argument("Reset index buffer must contain int32 or int64 values");
    }
    indices.resize(array.shape[0]);
    if (!indices.empty()) {
      std::vector<int64_t> wide(int64 ? indices.size() : 0);
      void *dst = int64 ? static_cast<void *>(wide.data()) : static_cast<void *>(indices.data());
      checkCudaErrors(cudaMemcpyAsync(dst, array.ptr, indices.size() * (int64 ? 8 : 4),
                                      cudaMemcpyDeviceToHost, mCudaStream));
      checkCudaErrors(cudaStreamSynchronize(mCudaStream));
      if (int64) {
        for (size_t i = 0; i < wide.size(); ++i) {
          if (wide[i] < 0 || uint64_t(wide[i]) >= rowCount) {
            throw std::out_of_range("Reset index is outside the SAPIEN buffer rows");
          }
          indices[i] = static_cast<int>(wide[i]);
        }
      }
    }
  }
  std::unordered_set<int> seen;
  for (int row : indices) {
    if (row < 0 || size_t(row) >= rowCount) {
      throw std::out_of_range("Reset index is outside the SAPIEN buffer rows");
    }
    if (!seen.insert(row).second) {
      throw std::invalid_argument("Reset indices must be unique");
    }
  }
  return indices;
}

void PhysxSystemGpu::gpuResetFiltering(GpuIndexSelection const &rigidBodyIndices,
                                      GpuIndexSelection const &articulationIndices) {
  checkGpuIdle();
  ensureCudaDevice();
  auto rigidRows = readResetIndices(rigidBodyIndices, mGpuRigidDynamicRows.size());
  auto artRows = readResetIndices(articulationIndices, mGpuArticulationRows.size());
  std::vector<std::shared_ptr<PhysxRigidBaseComponent>> bodies;
  for (int row : rigidRows) {
    auto body = mGpuRigidDynamicRows[row].lock();
    if (!body) {
      throw std::runtime_error("GPU body rows are stale; call gpu_init");
    }
    bodies.push_back(body);
  }
  for (int row : artRows) {
    auto art = mGpuArticulationRows[row].lock();
    if (!art) {
      throw std::runtime_error("GPU articulation rows are stale; call gpu_init");
    }
    auto links = art->getLinks();
    bodies.insert(bodies.end(), links.begin(), links.end());
  }
  resetFiltering(bodies);
}

void PhysxSystemGpu::beforeResetFiltering() {
  checkGpuIdle();
  ensureCudaDevice();
  checkCudaErrors(cudaStreamSynchronize(mCudaStream));
  // Exported contacts contain pointers into the now-destroyed interactions.
  // Do not permit queries until a completed simulation step replaces them.
  mContactInvalidatedByReset = true;
  mContactUpToDate = false;
}
#endif
} // namespace sapien::physx
