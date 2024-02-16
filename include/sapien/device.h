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
