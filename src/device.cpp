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
#include "sapien/device.h"
#include <svulkan2/core/context.h>
#include <svulkan2/core/instance.h>
#include <svulkan2/core/physical_device.h>

#ifdef SAPIEN_CUDA
#include <cuda_runtime.h>
#endif

namespace sapien {

static std::vector<std::shared_ptr<Device>> vulkanFindDevices() {
  auto level = svulkan2::logger::getLogLevel();
  svulkan2::logger::setLogLevel("off");

  std::vector<std::shared_ptr<Device>> res;
  try {
    std::shared_ptr<svulkan2::core::Instance> instance;
    try {
      auto context = svulkan2::core::Context::Get();
      instance = context->getInstance2();
    } catch (std::runtime_error &) {
      instance = std::make_shared<svulkan2::core::Instance>(
          VK_MAKE_VERSION(0, 0, 1), VK_MAKE_VERSION(0, 0, 1), VK_API_VERSION_1_2, false);
    }

    if (instance) {
      auto devices = instance->summarizePhysicalDevices();
      for (auto &d : devices) {
        int priority = 0;

        if (d.supported) {
          priority = 1;
          if (d.cudaId >= 0) {
            priority += 1000;
          }
          if (d.present) {
            priority += 100;
          }
          if (d.deviceType == vk::PhysicalDeviceType::eDiscreteGpu) {
            priority += 10;
          }
          if (d.rayTracing) {
            priority += 1;
          }
        }

        Device::Type type = Device::Type::GPU;
        res.push_back(std::make_shared<Device>(Device{.type = type,
                                                      .name = d.name,
                                                      .render = d.supported,
                                                      .present = d.present,
                                                      .cudaId = d.cudaId,
                                                      .pci = d.pci,
                                                      .renderPriority = priority}));
      }
    }
  } catch (std::runtime_error &e) {
  }
  svulkan2::logger::setLogLevel(level);
  return res;
}

#ifdef SAPIEN_CUDA

static std::array<uint32_t, 4> parsePCIString(std::string s) {
  if (s.length() == 12) {
    return {static_cast<uint32_t>(std::stoi(s.substr(0, 4), 0, 16)),
            static_cast<uint32_t>(std::stoi(s.substr(5, 2), 0, 16)),
            static_cast<uint32_t>(std::stoi(s.substr(8, 2), 0, 16)),
            static_cast<uint32_t>(std::stoi(s.substr(11, 1), 0, 16))};
  }
  if (s.length() == 7) {
    return {0u, static_cast<uint32_t>(std::stoi(s.substr(0, 2), 0, 16)),
            static_cast<uint32_t>(std::stoi(s.substr(3, 2), 0, 16)),
            static_cast<uint32_t>(std::stoi(s.substr(6, 1), 0, 16))};
  }
  throw std::runtime_error("invalid PCI string");
}

static std::vector<std::shared_ptr<Device>> cudaFindDevices() {
  std::vector<std::shared_ptr<Device>> res;

  int nDevices;
  if (cudaGetDeviceCount(&nDevices) != cudaSuccess) {
    return res;
  }
  for (int i = 0; i < nDevices; i++) {
    cudaDeviceProp prop;
    if (cudaGetDeviceProperties(&prop, i) != cudaSuccess) {
      continue;
    }
    char pci[20] = {0};
    if (cudaDeviceGetPCIBusId(pci, 20, i) != cudaSuccess) {
      continue;
    }

    res.push_back(std::make_shared<Device>(Device{.type = Device::Type::GPU,
                                                  .name = prop.name,
                                                  .render = false,
                                                  .present = false,
                                                  .cudaId = i,
                                                  .pci = parsePCIString(std::string(pci))}));
  }
  return res;
}
#endif

static std::vector<std::shared_ptr<Device>> findDevices() {
  std::vector<std::shared_ptr<Device>> devices;
  devices.push_back(std::make_shared<Device>(Device{
      .type = Device::Type::CPU, .name = "cpu", .render = false, .present = false, .cudaId = -1}));

  auto gpuDevices = vulkanFindDevices();
#ifdef SAPIEN_CUDA
  auto cudaDevices = cudaFindDevices();
  // merge devices
  for (auto cd : cudaDevices) {
    for (auto vd : gpuDevices) {
      if (cd->pci == vd->pci) {
        continue;
      }
    }
    gpuDevices.push_back(cd);
  }
#endif

  devices.insert(devices.end(), gpuDevices.begin(), gpuDevices.end());
  return devices;
}

std::string Device::getAlias() const {
  if (isCpu()) {
    return "cpu";
  }
  if (cudaId >= 0) {
    return "cuda:" + std::to_string(cudaId);
  }
  if (isGpu()) {
    char pciString[20] = {0};
    sprintf(pciString, "%04x:%02x:%02x.%1x", pci[0], pci[1], pci[2], pci[3]);
    return "pci:" + std::string(pciString);
  }
  return "unknown";
}

static std::vector<std::shared_ptr<Device>> gDevices;

std::shared_ptr<Device> findDevice(std::string alias) {
  if (gDevices.empty()) {
    gDevices = findDevices();
  }

  // CPU
  if (alias == "cpu") {
    for (auto d : gDevices) {
      if (d->isCpu()) {
        return d;
      }
    }
    throw std::runtime_error("failed to find device \"" + alias + "\"");
  }

  // CUDA
  if (alias.starts_with("cuda")) {
    // any cuda
    if (alias == "cuda") {
      for (auto d : gDevices) {
        if (d->isCuda()) {
          return d;
        }
      }
    }

    // cuda with id
    if (alias.starts_with("cuda:")) {
      int id = std::stoi(alias.substr(5));
      for (auto d : gDevices) {
        if (d->cudaId == id) {
          return d;
        }
      }
    }

    throw std::runtime_error("failed to find device \"" + alias + "\"");
  }

  // PCI
  if (alias.starts_with("pci:")) {
    std::array<uint32_t, 4> pci = {};
#ifdef SAPIEN_CUDA
    try {
      pci = parsePCIString(alias.substr(4));
      for (auto d : gDevices) {
        if (d->isGpu() && d->pci == pci) {
          return d;
        }
      }
    } catch (std::runtime_error const &) {
    }
#endif
    try {
      uint32_t busId = std::stoi(alias.substr(4), 0, 16);
      for (auto d : gDevices) {
        if (d->isGpu() && d->pci[1] == busId) {
          return d;
        }
      }
    } catch (std::runtime_error const &) {
    }

    throw std::runtime_error("failed to find device \"" + alias + "\"");
  }

  throw std::runtime_error("failed to find device \"" + alias + "\"");
}

std::shared_ptr<Device> findBestRenderDevice() {
  if (gDevices.empty()) {
    gDevices = findDevices();
  }
  std::shared_ptr<Device> bestDevice;
  int priority = 0;
  for (auto d : gDevices) {
    if (d->renderPriority > priority) {
      priority = d->renderPriority;
      bestDevice = d;
    }
  }
  return bestDevice;
}

} // namespace sapien
