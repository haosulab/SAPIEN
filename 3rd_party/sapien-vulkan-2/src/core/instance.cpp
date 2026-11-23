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
#include "svulkan2/core/instance.h"
#include "../common/cuda_helper.h"
#include "../common/logger.h"
#include "svulkan2/core/physical_device.h"
#include <GLFW/glfw3.h>
#include <filesystem>
#include <openvr.h>

namespace svulkan2 {
namespace core {

VkBool32 myDebugCallback(VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType,
                         uint64_t object, size_t location, int32_t messageCode,
                         const char *pLayerPrefix, const char *pMessage, void *pUserData) {
  printf("%s\n", pMessage);
  return false;
}

// static std::weak_ptr<Instance> gInstance;

static void glfwErrorCallback(int error_code, const char *description) {
  logger::error("GLFW error: {}. You may suppress this message by unsetting environment variable "
                "DISPLAY so SAPIEN will not attempt to test on-screen rendering",
                description);
}

// https://github.com/ValveSoftware/openvr/blob/f51d87ecf8f7903e859b0aa4d617ff1e5f33db5a/samples/hellovr_vulkan/hellovr_vulkan_main.cpp#L673
static bool GetVRInstanceExtensionsRequired(std::vector<std::string> &outInstanceExtensionList) {
  if (!vr::VRCompositor()) {
    return false;
  }

  outInstanceExtensionList.clear();
  uint32_t nBufferSize = vr::VRCompositor()->GetVulkanInstanceExtensionsRequired(nullptr, 0);
  if (nBufferSize > 0) {
    // Allocate memory for the space separated list and query for it
    char *pExtensionStr = new char[nBufferSize];
    pExtensionStr[0] = 0;
    vr::VRCompositor()->GetVulkanInstanceExtensionsRequired(pExtensionStr, nBufferSize);

    // Break up the space separated list into entries on the CUtlStringList
    std::string curExtStr;
    uint32_t nIndex = 0;
    while (pExtensionStr[nIndex] != 0 && (nIndex < nBufferSize)) {
      if (pExtensionStr[nIndex] == ' ') {
        outInstanceExtensionList.push_back(curExtStr);
        curExtStr.clear();
      } else {
        curExtStr += pExtensionStr[nIndex];
      }
      nIndex++;
    }
    if (curExtStr.size() > 0) {
      outInstanceExtensionList.push_back(curExtStr);
    }

    delete[] pExtensionStr;
  }

  return true;
}

Instance::Instance(uint32_t appVersion, uint32_t engineVersion, uint32_t apiVersion, bool enableVR)
    : mApiVersion(apiVersion) {

  bool glfw = false;
  if (glfwInit() && glfwVulkanSupported()) {
    logger::info("GLFW initialized.");
    glfw = true;
    glfwSetErrorCallback(glfwErrorCallback);
  } else {
    logger::info("Failed to initialize GLFW. Display is disabled.");
  }

  // try to load system vulkan library
  try {
    mDynamicLoader = std::make_unique<vk::DynamicLoader>();
  } catch (std::runtime_error &) {
    mDynamicLoader.reset();
  }

  // try to load SAPIEN-provided Vulkan
  if (!mDynamicLoader) {
    auto path = std::getenv("SAPIEN_VULKAN_LIBRARY_PATH");
    if (path && std::filesystem::is_regular_file(std::filesystem::path(path))) {
      try {
        mDynamicLoader = std::make_unique<vk::DynamicLoader>(path);
      } catch (std::runtime_error &) {
        mDynamicLoader.reset();
      }
    }
  }

  // give up
  if (!mDynamicLoader) {
    throw std::runtime_error("Failed to load vulkan library!");
  }

  vk::ApplicationInfo appInfo("SAPIEN", appVersion, "SAPIEN", engineVersion, apiVersion);

  auto vkGetInstanceProcAddr =
      mDynamicLoader->getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
  VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);

  // ========== set up layers ========== //
  std::vector<const char *> layers;
  auto vkEnumerateInstanceLayerProperties =
      mDynamicLoader->getProcAddress<PFN_vkEnumerateInstanceLayerProperties>(
          "vkEnumerateInstanceLayerProperties");

  // query all layers
  uint32_t availableLayerCount{};
  vkEnumerateInstanceLayerProperties(&availableLayerCount, nullptr);
  std::vector<VkLayerProperties> availableLayers(availableLayerCount);
  vkEnumerateInstanceLayerProperties(&availableLayerCount, availableLayers.data());

#ifdef VK_VALIDATION
  static const std::vector<const char *> validationLayers = {"VK_LAYER_KHRONOS_validation"};
  for (const char *layerName : validationLayers) {
    bool layerFound = false;
    for (const auto &layerProperties : availableLayers) {
      if (strcmp(layerName, layerProperties.layerName) == 0) {
        layers.push_back(layerName);
        layerFound = true;
        break;
      }
    }
    if (!layerFound) {
      logger::error("Validation layer " + std::string(layerName) +
                    " is not available and skipped.");
    }
  }

  vk::ValidationFeatureEnableEXT enable = vk::ValidationFeatureEnableEXT::eDebugPrintf;
  vk::ValidationFeaturesEXT validationFeatures(enable);

#endif

  // ========== set up extensions ========== //
  std::vector<const char *> extensions;
#ifdef VK_USE_PLATFORM_MACOS_MVK
  extensions.push_back(VK_MVK_MACOS_SURFACE_EXTENSION_NAME);
#endif
  auto vkEnumerateInstanceExtensionProperties =
      mDynamicLoader->getProcAddress<PFN_vkEnumerateInstanceExtensionProperties>(
          "vkEnumerateInstanceExtensionProperties");

  uint32_t availableExtensionCount{};
  vkEnumerateInstanceExtensionProperties(nullptr, &availableExtensionCount, nullptr);
  std::vector<VkExtensionProperties> availableExtensions(availableExtensionCount);
  vkEnumerateInstanceExtensionProperties(nullptr, &availableExtensionCount,
                                         availableExtensions.data());

  // external memory
  static const std::vector<char const *> externalMemoryExtensions = {
      VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME,
      VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES_EXTENSION_NAME};
  for (const char *extName : externalMemoryExtensions) {
    if (std::any_of(availableExtensions.begin(), availableExtensions.end(),
                    [=](auto const &prop) { return strcmp(prop.extensionName, extName) == 0; })) {
      extensions.push_back(extName);
    } else {
      logger::error("Extension " + std::string(extName) +
                    " is not available. Interop with external library is disabled.");
    }
  }

  if (glfw) {
    uint32_t glfwExtensionCount = 0;
    const char **glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    if (glfwExtensions) {
      mGLFWSupported = true;
      for (uint32_t i = 0; i < glfwExtensionCount; ++i) {
        extensions.push_back(glfwExtensions[i]);
      }
    }
  }
  // Get extensions supported by the instance and store for later use
  std::vector<std::string> supportedInstanceExtensions;
	uint32_t extCount = 0;
      VULKAN_HPP_DEFAULT_DISPATCHER.vkEnumerateInstanceExtensionProperties(nullptr, &extCount, nullptr);
	if (extCount > 0) {
		std::vector<VkExtensionProperties> extensions(extCount);
		if (VULKAN_HPP_DEFAULT_DISPATCHER.vkEnumerateInstanceExtensionProperties(nullptr, &extCount, &extensions.front()) == VK_SUCCESS) {
			for (VkExtensionProperties& extension : extensions) {
				supportedInstanceExtensions.push_back(extension.extensionName);
			}
		}
	}
  std::vector<const char*> enabledInstanceExtensions;
#ifdef VK_USE_PLATFORM_MACOS_MVK
	// SRS - When running on iOS/macOS with MoltenVK, enable VK_KHR_get_physical_device_properties2 if not already enabled by the example (required by VK_KHR_portability_subset)
	if (std::find(enabledInstanceExtensions.begin(), enabledInstanceExtensions.end(), VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME) == enabledInstanceExtensions.end()) {
		enabledInstanceExtensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
	}
#endif
  // Enabled requested instance extensions
	if (enabledInstanceExtensions.size() > 0) {
		for (const char * enabledExtension : enabledInstanceExtensions) {
			// Output message if requested extension is not available
			if (std::find(supportedInstanceExtensions.begin(), supportedInstanceExtensions.end(), enabledExtension) == supportedInstanceExtensions.end()) {
        printf("Enabled instance extension %s is not present at instance level.\n", enabledExtension);
			}
			extensions.push_back(enabledExtension);
		}
	}
#if defined(VK_USE_PLATFORM_MACOS_MVK) && defined(VK_KHR_portability_enumeration)
	// SRS - When running on iOS/macOS with MoltenVK and VK_KHR_portability_enumeration is defined and supported by the instance, enable the extension and the flag
	if (std::find(supportedInstanceExtensions.begin(), supportedInstanceExtensions.end(), VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME) != supportedInstanceExtensions.end()) {
		extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
	}
#endif

  // vr
  if (enableVR && vr::VR_IsHmdPresent()) {
    vr::EVRInitError err = vr::VRInitError_None;
    vr::VR_Init(&err, vr::VRApplication_Scene);
    if (err == vr::VRInitError_None) {
      std::vector<char const *> vrExtensions;
      std::vector<std::string> names;
      if (GetVRInstanceExtensionsRequired(names)) {
        for (std::string const &name : names) {
          for (auto const &prop : availableExtensions) {
            if (strcmp(prop.extensionName, name.c_str()) == 0) {
              vrExtensions.push_back(prop.extensionName);
              continue;
            }
          }
        }
      }
      if (vrExtensions.size() == names.size()) {
        mVRSupported = true;
        extensions.insert(extensions.end(), vrExtensions.begin(), vrExtensions.end());
      }
    }
  }

#ifdef VK_VALIDATION
  extensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
#endif

  try {
    auto info = vk::InstanceCreateInfo({}, &appInfo, layers, extensions);
#ifdef VK_USE_PLATFORM_MACOS_MVK
    info.flags |= vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR;
#endif
#ifdef VK_VALIDATION
    info.setPNext(&validationFeatures);
#endif

    mInstance = vk::createInstanceUnique(info);
    VULKAN_HPP_DEFAULT_DISPATCHER.init(mInstance.get());

#ifdef VK_VALIDATION
    vk::DebugReportCallbackCreateInfoEXT ci(vk::DebugReportFlagBitsEXT::eInformation,
                                            myDebugCallback);
    mDebugCallbackHandle = mInstance->createDebugReportCallbackEXT(ci, nullptr);
#endif

  } catch (vk::OutOfHostMemoryError const &err) {
    logger::error("Failed to initialize renderer: out of host memory.");
    throw err;
  } catch (vk::OutOfDeviceMemoryError const &err) {
    logger::error("Failed to initialize renderer: out of device memory.");
    throw err;
  } catch (vk::InitializationFailedError const &err) {
    logger::error("Failed to initialize Vulkan. You may not use the renderer for rendering.");
    throw err;
  } catch (vk::LayerNotPresentError const &err) {
    logger::error(
        "Failed to load required Vulkan layer. You may not use the renderer for rendering.");
    throw err;
  } catch (vk::ExtensionNotPresentError const &err) {
    logger::error(
        "Failed to load required Vulkan extension. You may not use the renderer for rendering.");
    throw err;
  } catch (vk::IncompatibleDriverError const &err) {
    logger::error(
        "Your GPU driver does not support Vulkan. You may not use the renderer for rendering.");
    throw err;
  }

  mPhysicalDeviceInfo = summarizePhysicalDevices();
}

std::vector<PhysicalDeviceInfo> Instance::summarizePhysicalDevices() const {
  std::vector<PhysicalDeviceInfo> devices;

  // prepare a surface for window testing
  bool presentEnabled = isGLFWEnabled();
  GLFWwindow *window{};
  VkSurfaceKHR tmpSurface = nullptr;
  if (presentEnabled) {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window = glfwCreateWindow(1, 1, "SAPIEN", nullptr, nullptr);
    auto result = glfwCreateWindowSurface(mInstance.get(), window, nullptr, &tmpSurface);
    if (result != VK_SUCCESS) {
      throw std::runtime_error(
          "Window creation test failed, you may not create GLFW window for display.");
      presentEnabled = false;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  }

  std::stringstream ss;
  ss << "Devices visible to Vulkan" << std::endl;
  ss << std::setw(3) << "Id" << std::setw(40) << "name" << std::setw(10) << "Present"
     << std::setw(10) << "Supported" << std::setw(25) << "Pci" << std::setw(10) << "CudaId"
     << std::setw(15) << "RayTracing" << std::setw(10) << "Discrete" << std::setw(15)
     << "SubgroupSize" << std::endl;

  vk::PhysicalDeviceFeatures2 features{};
  vk::PhysicalDeviceDescriptorIndexingFeatures descriptorFeatures{};
  vk::PhysicalDeviceTimelineSemaphoreFeatures timelineSemaphoreFeatures{};
  features.setPNext(&descriptorFeatures);
  descriptorFeatures.setPNext(&timelineSemaphoreFeatures);

  int ord = 0;
  for (auto device : mInstance->enumeratePhysicalDevices()) {
    std::string name;
    bool present = false;
    bool required_features = false;
    int cudaId = -1;
    int queueIdxNoPresent = -1;
    int queueIdxPresent = -1;
    int queueIdx = -1;
    bool rayTracing = false;

    // check graphics & compute queues
    std::vector<vk::QueueFamilyProperties> queueFamilyProperties =
        device.getQueueFamilyProperties();
    for (uint32_t i = 0; i < queueFamilyProperties.size(); ++i) {
      if (queueFamilyProperties[i].queueFlags & vk::QueueFlagBits::eGraphics &&
          queueFamilyProperties[i].queueFlags & vk::QueueFlagBits::eCompute) {
        if (queueIdxNoPresent == -1) {
          queueIdxNoPresent = i;
        }
        if (tmpSurface && device.getSurfaceSupportKHR(i, tmpSurface)) {
          queueIdxPresent = i;
          present = true;
          break;
        }
      }
    }

    if (queueIdxPresent != -1) {
      queueIdx = queueIdxPresent;
    } else {
      queueIdx = queueIdxNoPresent;
    }

    // check features
    device.getFeatures2(&features);
    if (features.features.independentBlend && descriptorFeatures.descriptorBindingPartiallyBound &&
        timelineSemaphoreFeatures.timelineSemaphore) {
      required_features = true;
    }

    std::vector<const char *> all_required_extensions = {
        VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME};
    size_t required_extension_count = 0;

    // check extensions
    auto extensions = device.enumerateDeviceExtensionProperties();
    for (auto &ext : extensions) {
      if (std::strcmp(ext.extensionName, VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME) == 0) {
        rayTracing = 1;
      }
      for (auto name : all_required_extensions)
        if (std::strcmp(ext.extensionName, name) == 0) {
          required_extension_count += 1;
        }
    }

    bool required_extensions = required_extension_count == all_required_extensions.size();

    auto properties2 =
        device.getProperties2<vk::PhysicalDeviceProperties2, vk::PhysicalDeviceSubgroupProperties,
                              vk::PhysicalDevicePCIBusInfoPropertiesEXT>();
    auto properties = properties2.get<vk::PhysicalDeviceProperties2>().properties;

    name = std::string(properties.deviceName.begin(), properties.deviceName.end());
    auto deviceType = properties.deviceType;

    std::array<uint32_t, 4> pci = {
        properties2.get<vk::PhysicalDevicePCIBusInfoPropertiesEXT>().pciDomain,
        properties2.get<vk::PhysicalDevicePCIBusInfoPropertiesEXT>().pciBus,
        properties2.get<vk::PhysicalDevicePCIBusInfoPropertiesEXT>().pciDevice,
        properties2.get<vk::PhysicalDevicePCIBusInfoPropertiesEXT>().pciFunction};

    uint32_t subgroupSize = properties2.get<vk::PhysicalDeviceSubgroupProperties>().subgroupSize;

    int computeMode{-1};
    auto SAPIEN_DISABLE_RAY_TRACING = std::getenv("SAPIEN_DISABLE_RAY_TRACING");
    if (SAPIEN_DISABLE_RAY_TRACING && std::strcmp(SAPIEN_DISABLE_RAY_TRACING, "1") == 0) {
      rayTracing = 0;
    } else {
#ifdef SVULKAN2_CUDA_INTEROP
      cudaId = getCudaDeviceIdFromPhysicalDevice(device);
      cudaDeviceGetAttribute(&computeMode, cudaDevAttrComputeMode, cudaId);
      if ((computeMode == cudaComputeModeExclusiveProcess ||
           computeMode == cudaComputeModeExclusive) &&
          rayTracing) {
        logger::warn("CUDA device {} is in EXCLUSIVE or EXCLUSIVE_PROCESS mode. You "
                     "many not use this renderer with external CUDA programs unless "
                     "you switch off ray tracing by environment variable "
                     "SAPIEN_DISABLE_RAY_TRACING=1.",
                     cudaId);
      }
#endif
    }
    bool supported = required_features && required_extensions && queueIdx != -1;
    bool discrete = deviceType == vk::PhysicalDeviceType::eDiscreteGpu;

    ss << std::setw(3) << ord++ << std::setw(40) << name.substr(0, 39).c_str() << std::setw(10)
       << present << std::setw(10) << supported << std::setw(25)
#ifdef SVULKAN2_CUDA_INTEROP
        << std::hex << PCIToString(pci)
#endif
       << std::dec << std::setw(10) << (cudaId < 0 ? "No Device" : std::to_string(cudaId))
       << std::setw(15) << rayTracing << std::setw(10) << discrete << std::setw(15) << subgroupSize
       << std::endl;

    devices.push_back(
        PhysicalDeviceInfo{.name = std::string(name.c_str()),
                           .device = device,
                           .present = present,
                           .supported = supported,
                           .cudaId = cudaId,
                           .pci = pci,
                           .queueIndex = queueIdx,
                           .rayTracing = rayTracing,
                           .cudaComputeMode = computeMode,
                           .deviceType = deviceType,
                           .subgroupSize = subgroupSize,
                           .features{.wideLines = (bool)features.features.wideLines,
                                     .geometryShader = (bool)features.features.geometryShader}});
  }
  logger::info(ss.str());

#ifdef SVULKAN2_CUDA_INTEROP
  ss = {};
  ss << "Devices visible to Cuda" << std::endl;
  ss << std::setw(10) << "CudaId" << std::setw(25) << "Pci" << std::endl;

  int count{0};
  cudaGetDeviceCount(&count);
  for (int i = 0; i < count; ++i) {
    std::string pciBus(20, '\0');
    auto result = cudaDeviceGetPCIBusId(pciBus.data(), 20, i);

    switch (result) {
    case cudaErrorInvalidValue:
      pciBus = "Invalid Value";
      break;
    case cudaErrorInvalidDevice:
      pciBus = "Invalid Device";
      break;
    case cudaErrorInitializationError:
      pciBus = "Not Initialized";
      break;
    case cudaErrorInsufficientDriver:
      pciBus = "Insufficient Driver";
      break;
    case cudaErrorNoDevice:
      pciBus = "No Device";
      break;
    default:
      ss << std::setw(10) << i << std::hex << std::dec << std::setw(25) << pciBus.c_str()
         << std::endl;
      break;
    }
  }
  logger::info(ss.str());
#endif

  // clean up
  if (tmpSurface) {
    mInstance->destroySurfaceKHR(tmpSurface);
  }
  if (window) {
    glfwDestroyWindow(window);
  }

  return devices;
}

static inline uint32_t computeDevicePriority(PhysicalDeviceInfo const &info,
                                             std::string const &hint) {

  // specific cuda device
  if (hint.starts_with("cuda:")) {
    if (info.cudaId == std::stoi(hint.substr(5))) {
      return 1000;
    }
    return 0;
  }

  // any cuda device
  if (hint == std::string("cuda")) {
    uint32_t score = 0;
    if (info.cudaId >= 0) {
      score += 1000;
    }
    if (info.present) {
      score += 100;
    }
    if (info.rayTracing) {
      score += 1;
    }
    return score;
  }

  // specific device
  if (hint.starts_with("pci:")) {
    std::string pciString = hint.substr(4);

    // pci can be parsed
#ifdef SVULKAN2_CUDA_INTEROP
    try {
      auto pci = parsePCIString(pciString);
      if (info.pci == pci) {
        return 1000;
      }
    } catch (std::runtime_error const &) {
    }
  #endif

    // only bus is provided
    try {
      if (info.pci[1] == std::stoi(pciString, 0, 16)) {
        return 1000;
      }
    } catch (std::runtime_error const &) {
    }

    return 0;
  }

  // no device hint
  // still prefer cuda device
  if (!info.supported) {
    return 0;
  }
  uint32_t score = 0;
  if (info.cudaId >= 0) {
    score += 1000;
  }
  if (info.present) {
    score += 100;
  }
  if (info.deviceType == vk::PhysicalDeviceType::eDiscreteGpu) {
    score += 10;
  }
  if (info.rayTracing) {
    score += 1;
  }
  return score;
}

std::shared_ptr<PhysicalDevice> Instance::createPhysicalDevice(std::string const &hint) {

  int pickedDeviceIdx = -1;
  {
    uint32_t maxPriority = 0;
    uint32_t idx = 0;
    for (auto &device : mPhysicalDeviceInfo) {
      uint32_t priority = computeDevicePriority(device, hint);
      if (priority > maxPriority) {
        maxPriority = priority;
        pickedDeviceIdx = idx;
      }
      idx++;
    }
  }
  if (pickedDeviceIdx < 0) {
    throw std::runtime_error("Failed to find a supported physical device \"" + hint + "\"");
  }

  return std::make_shared<PhysicalDevice>(shared_from_this(),
                                          mPhysicalDeviceInfo.at(pickedDeviceIdx));
}

void Instance::shutdownVR() const {
  if (mVRSupported) {
    logger::info("VR shutting down");
    vr::VR_Shutdown();
  }
}

Instance::~Instance() { glfwTerminate(); }

} // namespace core
} // namespace svulkan2