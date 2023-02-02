cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(sapien LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wnon-virtual-dtor -fsized-deallocation -Wno-deprecated-declarations -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0 -Wall -Wnon-virtual-dtor  -fsized-deallocation -Wno-deprecated-declarations -fopenmp")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_DEBUG_POSTFIX "")

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
option(SAPIEN_PROFILE "Build SAPIEN with easy_profiler" OFF)
option(SAPIEN_DEBUG_VIEWER "Build debug viewer for debugging renderer in C++" OFF)
option(SAPIEN_CUDA "Enable SAPIEN CUDA functionalities, including dlpack, CUDA buffer, denoiser, and simsense" ON)
option(SAPIEN_KUAFU "Build Kuafu ray tracer" ON)

if (${SAPIEN_PROFILE})
    message("-- Profiler On")
    option(SVULKAN2_PROFILE "" ON)
    add_definitions(-DBUILD_WITH_EASY_PROFILER)
    find_package(easy_profiler REQUIRED)
else ()
    message("-- Profiler Off")
    option(SVULKAN2_PROFILE "" OFF)
endif ()


if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif ()

if (${SAPIEN_CUDA})
    include_directories("3rd_party/dlpack/include")
    include_directories("$ENV{CUDA_PATH}/include")
    option(SVULKAN2_CUDA_INTEROP "" ON)
    option(KUAFU_OPTIX_DENOISER "" ON)
    add_definitions(-DKUAFU_OPTIX_DENOISER)
    add_definitions(-DSVULKAN2_CUDA_INTEROP)
    add_definitions(-DSAPIEN_DLPACK)
    add_definitions(-DSAPIEN_CUDA)
else()
    option(SVULKAN2_CUDA_INTEROP "" OFF)
    option(KUAFU_OPTIX_DENOISER "" OFF)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("-- DEBUG Build")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})
    add_definitions(-DVK_VALIDATION)
    add_definitions(-DVK_CORE_LOGGING)
    add_definitions(-D_DEBUG)
else ()
    message("-- RELEASE Build")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})
    add_definitions(-DNDEBUG)
endif ()

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    message("Complie source code on Mac OSX")
    set(MACOSX TRUE)
    set(CMAKE_MACOSX_RPATH OFF CACHE BOOL "Do not use rpath for MacOS")
endif ()

add_definitions(-DVULKAN_HPP_DISPATCH_LOADER_DYNAMIC)

find_package(spdlog REQUIRED)
find_package(assimp REQUIRED)
# Pinocchio uses its own FindCppAD but does not provide it
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CppAD")
find_package(Boost COMPONENTS system REQUIRED)
find_package(pinocchio REQUIRED)

set(PINOCCHIO_LIBRARY ${pinocchio_LIBRARIES} ${Boost_LIBRARIES} urdfdom_model)
add_definitions(-DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
add_definitions(-DPINOCCHIO_WITH_URDFDOM)

set(GRPC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../grpc/install")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(GRPC REQUIRED)
include_directories(${GRPC_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR})

if (${BUILD_DEBUG_VIEWER})
    add_definitions(-D_DEBUG_VIEWER)
endif ()

## include headers inside the project
include_directories("src" "src/renderer" "/usr/include/eigen3" "/usr/local/include/eigen3")

include_directories(
    "include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/pxshared/include"
    "src/renderer"
    )

# Link directory outside the project
if (NOT MACOSX)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/checked")
    else ()
        link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/release")
    endif ()
else ()
    include_directories("/usr/local/include" "/usr/local/include/eigen3")
    link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/mac.x86_64/release")
    link_directories("/usr/local/opt/glm/lib" "/usr/local/opt/glew/lib" "/usr/local/lib")
endif ()

# PVD currently not used add_definitions(-DPVD_HOST="192.168.0.110")

add_subdirectory("3rd_party/pybind11")
add_subdirectory("3rd_party/tinyxml2")
add_subdirectory("3rd_party/sapien-vulkan-2" EXCLUDE_FROM_ALL)
option(OPENVDB_STATIC "Use Static OPENVDB" ON)
add_subdirectory("3rd_party/CoACD" EXCLUDE_FROM_ALL)
set_target_properties(coacd PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

if (${SAPIEN_KUAFU})
    add_subdirectory("3rd_party/kuafu" EXCLUDE_FROM_ALL)
    add_definitions(-DSAPIEN_KUAFU)
endif()

include_directories("3rd_party/pybind11/include")
include_directories("3rd_party/sapien-vulkan-2/include")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/VulkanMemoryAllocator")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/imgui")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/imgui/backends")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/ImGuizmo")
include_directories("$ENV{VULKAN_SDK}/include")

if (${SAPIEN_KUAFU})
    include_directories("3rd_party/kuafu/include")
    include_directories("3rd_party/kuafu/include/external")
    include_directories("3rd_party/kuafu/3rd_party/OptiX7SDK/include")
    find_package(SDL2 REQUIRED)
    include_directories(${SDL2_INCLUDE_DIRS})
    set(KUAFU_LIBRARY "kuafu")
endif()


if (${SAPIEN_CUDA})
    include_directories("3rd_party/simsense/include")
    add_subdirectory("3rd_party/simsense" EXCLUDE_FROM_ALL)
    set_target_properties(simsense PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    set(SIMSENSE_LIBRARY simsense)
    add_definitions(-DSAPIEN_SIMSENSE)
endif ()

link_directories("$ENV{VULKAN_SDK}/lib")

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp" "src/*.cc")
add_library(sapien SHARED ${PROJECT_SRC})


if (MACOSX)
    target_link_libraries(sapien
            libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
            libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
            libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
            libPhysX_static_64.a libPhysXVehicle_static_64.a
            libSnippetRender_static_64.a libSnippetUtils_static_64.a
            pthread ${OPENGL_LIBRARY} glfw GLEW ${ASSIMP_LIBRARIES} dl
            tinyxml2 ${OPTIX_LIBRARY} ${SPDLOG_LIBRARIES} ${EASY_PROFILER_LIBRARY}
            ${PINOCCHIO_LIBRARY} svulkan2 ${KUAFU_LIBRARY} rt gRPC::grpc++ "-framework Cocoa -framework IOKit")
else ()
    target_link_libraries(sapien -Wl,--start-group
            libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
            libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
            libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
            libPhysX_static_64.a libPhysXVehicle_static_64.a
            libSnippetRender_static_64.a libSnippetUtils_static_64.a -Wl,--end-group
            pthread ${OPENGL_LIBRARY} glfw GLEW ${ASSIMP_LIBRARIES} dl
            stdc++fs tinyxml2 ${OPTIX_LIBRARY} ${SPDLOG_LIBRARIES} ${EASY_PROFILER_LIBRARY}
            ${PINOCCHIO_LIBRARY} svulkan2 ${KUAFU_LIBRARY} rt ${GRPC_CLIENT_LIBS} coacd)
endif ()

set_target_properties(sapien PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(sapien PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

pybind11_add_module(pysapien "python/pysapien.cpp" NO_EXTRAS)
target_link_libraries(pysapien PRIVATE sapien ${SIMSENSE_LIBRARY})

add_custom_target(python_test COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/test/*.py ${CMAKE_CURRENT_SOURCE_DIR}/test/*.json ${CMAKE_CURRENT_BINARY_DIR})
add_custom_target(manual_python COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/manualtest/*.py ${CMAKE_CURRENT_BINARY_DIR})
