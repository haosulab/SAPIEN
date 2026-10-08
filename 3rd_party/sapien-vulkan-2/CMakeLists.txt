cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
project(sapienvulkan2 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)

option(SVULKAN2_SHARED "Force svulkan2 to be shared" ON)
option(SVULKAN2_PROFILE "Build with profiler" OFF)
option(SVULKAN2_BUILD_TEST "Build with unit test" OFF)
option(SVULKAN2_CUDA_INTEROP "Allow CUDA to use Vulkan buffer" ON)

if(MSVC)
    add_compile_options("/W4" "$<$<CONFIG:Release>:/O2>")
else()
    add_compile_options("-Wall"
        "$<$<CONFIG:Debug>:-g3>" "$<$<CONFIG:Debug>:-O0>"
        "$<$<CONFIG:Release>:-O3>")
endif()

add_compile_definitions("$<$<CONFIG:DEBUG>:_DEBUG>")
add_compile_definitions("$<$<CONFIG:DEBUG>:VK_VALIDATION>")
add_compile_definitions("$<$<NOT:$<CONFIG:Debug>>:NDEBUG>")

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(glm)
include(assimp)
include(spdlog)
include(glfw)
include(ktx)
include(vulkan)
include(glslang)
include(spirv-cross)
include(imgui)
include(openexr)
include(openvr)

file(GLOB_RECURSE SVULKAN2_SRC "src/*.cpp")

if(SVULKAN2_SHARED)
    add_library(svulkan2 SHARED ${SVULKAN2_SRC})
else()
    add_library(svulkan2 ${SVULKAN2_SRC})
endif()

set_target_properties(svulkan2 PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_compile_definitions(imgui PRIVATE VK_NO_PROTOTYPES)
target_compile_definitions(svulkan2 PUBLIC VK_NO_PROTOTYPES)

if (${SAPIEN_MACOS})
    target_compile_definitions(svulkan2 PUBLIC SAPIEN_MACOS)
    target_compile_definitions(svulkan2 PUBLIC VK_USE_PLATFORM_MACOS_MVK)
    target_compile_definitions(svulkan2 PUBLIC VK_ENABLE_BETA_EXTENSIONS)
endif()
if (${VK_VALIDATION})
    target_compile_definitions(svulkan2 PUBLIC VK_VALIDATION)
endif()

target_include_directories(svulkan2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(svulkan2 PRIVATE
    "3rd_party/stb" "3rd_party/OptiX7SDK/include"
    "3rd_party/tinygltf"
)
target_link_libraries(svulkan2 PUBLIC Vulkan::Headers glm)

find_package(Threads REQUIRED)
target_link_libraries(svulkan2 PRIVATE assimp::assimp glslang SPIRV spirv-cross-cpp glfw spdlog::spdlog ktx imgui ${CMAKE_DL_LIBS} Threads::Threads OpenEXR)
if (MSVC)
    target_link_libraries(svulkan2 PRIVATE openvr_api64)
else()
    target_link_libraries(svulkan2 PRIVATE openvr_api)
endif()

if (${SVULKAN2_PROFILE})
    target_compile_definitions(svulkan2 PUBLIC SVULKAN2_PROFILE)
else()
    target_compile_definitions(svulkan2 PRIVATE NVTX_DISABLE=1)
endif()


if (${SVULKAN2_CUDA_INTEROP})
    find_package(CUDAToolkit REQUIRED)
    target_compile_definitions(svulkan2 PUBLIC SVULKAN2_CUDA_INTEROP)
    target_include_directories(svulkan2 PRIVATE "$ENV{CUDA_PATH}/include")
    if (MSVC)
        target_link_libraries(svulkan2 PRIVATE "$ENV{CUDA_PATH}/lib/x64/cudart_static.lib")
    else()
        target_link_libraries(svulkan2 PRIVATE "$ENV{CUDA_PATH}/lib64/libcudart_static.a" rt)
    endif()

    include(oidn)
    target_link_libraries(svulkan2 PRIVATE OpenImageDenoise)
endif()

# TODO: add option to control the following variables
# this library exports the storage api
target_compile_definitions(svulkan2 PRIVATE VULKAN_HPP_STORAGE_SHARED_EXPORT)
# other libraries use the exported storage api
target_compile_definitions(svulkan2 PUBLIC VULKAN_HPP_STORAGE_SHARED)

if (${SVULKAN2_CUDA_INTEROP})
    install(TARGETS svulkan2 Vulkan-Headers glm OpenImageDenoise OpenImageDenoise_core OpenImageDenoise_common
        EXPORT svulkan2Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
else ()
    install(TARGETS svulkan2 Vulkan-Headers glm
        EXPORT svulkan2Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif ()

install(EXPORT svulkan2Targets
    FILE svulkan2.cmake
    NAMESPACE svulkan2::
    DESTINATION lib/cmake/svulkan2
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/svulkan2 DESTINATION "include")
install(DIRECTORY "${vulkan_SOURCE_DIR}/include/vulkan" DESTINATION "include")
install(DIRECTORY "${vulkan_SOURCE_DIR}/include/vk_video" DESTINATION "include")
install(DIRECTORY "${glm_SOURCE_DIR}/glm" DESTINATION "include")

add_executable(svulkan2_main app/main.cpp)
target_link_libraries(svulkan2_main svulkan2 glfw imgui)
if (UNIX)
    target_link_libraries(svulkan2_main stdc++fs)
endif()

add_executable(svulkan2_vr app/vr.cpp)
target_link_libraries(svulkan2_vr svulkan2 glfw imgui)
if (UNIX)
    target_link_libraries(svulkan2_vr stdc++fs)
endif()

add_executable(gencube app/gencube.cpp)
target_link_libraries(gencube svulkan2)

add_executable(glslc_embed app/glslc_embed.cpp)
target_link_libraries(glslc_embed svulkan2)
if (UNIX)
    target_link_libraries(glslc_embed stdc++fs)
endif()
