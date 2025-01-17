if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SAPIEN_CUDA OFF CACHE INTERNAL "Enable SAPIEN CUDA functionalities, including dlpack, CUDA buffer, denoiser, and simsense")
set(SAPIEN_MACOS ON CACHE INTERNAL "Use platform macOS")
set(SVULKAN2_CUDA_INTEROP OFF CACHE INTERNAL "Allow CUDA to use Vulkan buffer")
set(VK_VALIDATION ON CACHE INTERNAL "Enable vk validation")