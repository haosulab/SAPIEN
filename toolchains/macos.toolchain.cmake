if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Set the default build type" FORCE)
endif()

if(NOT CMAKE_OSX_ARCHITECTURES)
    set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "Build architectures for Mac" FORCE)
endif()

if(NOT CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "12.0" CACHE STRING "Set the default os version" FORCE)
endif()

if(NOT CMAKE_Fortran_COMPILER)
    set(CMAKE_Fortran_COMPILER OFF CACHE STRING "Close compiler fortran" FORCE)
endif()

set(SAPIEN_CUDA OFF CACHE INTERNAL "Enable SAPIEN CUDA functionalities, including dlpack, CUDA buffer, denoiser, and simsense")
set(SAPIEN_MACOS ON CACHE INTERNAL "Use platform macOS")
set(SVULKAN2_CUDA_INTEROP OFF CACHE INTERNAL "Allow CUDA to use Vulkan buffer")
set(VK_VALIDATION ON CACHE INTERNAL "Enable vk validation")