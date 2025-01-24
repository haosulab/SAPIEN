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
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace sapien {
struct Device {
  enum class Type { CPU, GPU, OTHER } type;
  std::string name{};
  bool render{};
  bool present{};
  int cudaId{-1};
  std::array<uint32_t, 4> pci;
  int renderPriority{}; // how good is this device for rendering

  inline bool isCpu() const { return type == Type::CPU; }
  inline bool isGpu() const { return type == Type::GPU; }
  inline bool isCuda() const { return cudaId >= 0; }

  inline bool canRender() const { return render; }
  inline bool canPresent() const { return present; }
  inline std::optional<std::string> getPciString() const {
    if (isCpu()) {
      return {};
    }

    std::string pciStr(12, '\0');
    sprintf(pciStr.data(), "%04x:%02x:%02x.%1x", pci[0], pci[1], pci[2], pci[3]);
    return pciStr;
  }

  std::string getAlias() const;
};

std::shared_ptr<Device> findDevice(std::string alias);
std::shared_ptr<Device> findBestRenderDevice();

} // namespace sapien
