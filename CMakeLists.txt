cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(sapien LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall -Wnon-virtual-dtor -fsized-deallocation -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall -Wnon-virtual-dtor  -fsized-deallocation -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Do not try to build shared libraries")

set(SAPIEN_PROFILE FALSE CACHE BOOL "Whether to use Profiler")
set(SAPIEN_DLPACK_INTEROP TRUE CACHE BOOL "Whether to use render to tensor")
set(BUILD_WITH_PINOCCHIO_SUPPORT TRUE CACHE BOOL "Use Pinocchio kinematics dynamics library")
set(BUILD_DEBUG_VIEWER FALSE CACHE BOOL "For C++ debug only")

if (${SAPIEN_DLPACK_INTEROP})
    set(SVULKAN2_CUDA_INTEROP TRUE)
endif()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_DEBUG_POSTFIX "")
set(OpenGL_GL_PREFERENCE "LEGACY")

include_directories("3rd_party/sapien-vulkan-2/3rd_party/easy_profiler/easy_profiler_core/include")
if (${SAPIEN_PROFILE})
    add_definitions(-DBUILD_WITH_EASY_PROFILER)
    set(EASY_PROFILER_LIBRARY easy_profiler)
    message("-- Profiler On")
else ()
    message("-- Profiler Off")
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("-- DEBUG Build")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})
else ()
    message("-- RELEASE Build")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    message("Complie source code on Mac OSX")
    add_definitions(-D_USE_MACOSX)
    set(MACOSX TRUE)
    set(CMAKE_MACOSX_RPATH OFF CACHE BOOL "Do not use rpath for MacOS")
endif ()

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(assimp REQUIRED)
find_package(spdlog REQUIRED)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/pxshared/include" "3rd_party/optifuser/include"
        "src/renderer"
        )

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/checked"
            "3rd_party/optifuser/build/lib")
else ()
    link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/release"
            "3rd_party/optifuser/build/lib")
endif ()

# local changes by jet for clion
set(pinocchio_DIR "/zdata/anaconda3/envs/sapien/lib/cmake/pinocchio/")
set(eigenpy_DIR "/zdata/anaconda3/envs/sapien/lib/cmake/eigenpy/")
set(hpp-fcl_DIR "/zdata/anaconda3/envs/sapien/lib/cmake/hpp-fcl/")
set(octomap_DIR "/zdata/anaconda3/envs/sapien/share/octomap/")
include_directories("/zdata/anaconda3/envs/sapien/include/")

if (${BUILD_WITH_PINOCCHIO_SUPPORT})
    # Pinocchio uses its own FindCppAD but does not provide it
    set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CppAD")

    find_package(Boost COMPONENTS system REQUIRED)
    find_package(pinocchio REQUIRED)
    set(PINOCCHIO_LIBRARY ${pinocchio_LIBRARIES} ${Boost_LIBRARIES} urdfdom_model)
    add_definitions(-DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
    add_definitions(-DPINOCCHIO_WITH_URDFDOM)
    add_definitions(-D_USE_PINOCCHIO)
endif ()

if (${BUILD_DEBUG_VIEWER})
    add_definitions(-D_DEBUG_VIEWER)
endif ()

## include headers inside the project
include_directories("3rd_party/optifuser/include" "src" "src/renderer" "/usr/include/eigen3")

if (MACOSX)
    include_directories("/usr/local/include" "/usr/local/opt/glew/include"
            "/usr/local/include/eigen3")
    link_directories(/usr/local/lib/)
else ()
    include_directories("3rd_party/dlpack/include")
    if (${SAPIEN_DLPACK_INTEROP})
        add_definitions(-DSAPIEN_DLPACK_INTEROP)
        add_definitions(-DCUDA_INTEROP)
    endif ()
endif ()

## Link directory inside the project
link_directories("${CMAKE_CURRENT_BINARY_DIR}/3rd_party/optifuser/lib")

## Link directory outside the project
if (NOT MACOSX)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/checked")
    else ()
        link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/release")
    endif ()
else ()
    link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../PhysX/physx/bin/mac.x86_64/release")
    link_directories("/usr/local/opt/glm/lib" "/usr/local/opt/glew/lib"
            "/usr/local/lib")
endif ()

# set(ON_SCREEN TRUE CACHE BOOL "Vulkan renderer with on screen rendering")
# if (${ON_SCREEN})
add_definitions(-DON_SCREEN)
# endif()

set(OPTIX_HOME "NOT FOUND" CACHE FILEPATH "Home to OptiX")
if (EXISTS ${OPTIX_HOME})
    include_directories("${OPTIX_HOME}/include")
    link_directories("${OPTIX_HOME}/lib64")
    set(OPTIX_LIBRARY optix)
    add_definitions(-D_USE_OPTIX)
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-D_DEBUG)
    # add_definitions(-D_PVD)
    add_definitions(-DPVD_HOST="192.168.0.110")
else ()
    add_definitions(-DNDEBUG)
endif ()

add_subdirectory("3rd_party/optifuser" EXCLUDE_FROM_ALL)
add_subdirectory("3rd_party/pybind11")
add_subdirectory("3rd_party/tinyxml2")
include_directories("3rd_party/pybind11/include")

add_subdirectory("3rd_party/sapien-vulkan-2" EXCLUDE_FROM_ALL)
include_directories("3rd_party/sapien-vulkan-2/include")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/VulkanMemoryAllocator")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/cuda_include")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/imgui")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/imgui/backends")
include_directories("3rd_party/sapien-vulkan-2/3rd_party/ImGuizmo")
include_directories("$ENV{VULKAN_SDK}/include")
link_directories("$ENV{VULKAN_SDK}/lib")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/sapien-vulkan-2/3rd_party/cuda_lib")

add_subdirectory("3rd_party/kuafu" EXCLUDE_FROM_ALL)
include_directories("3rd_party/kuafu/include")
include_directories("3rd_party/kuafu/include/external")
include_directories("3rd_party/kuafu/3rd_party/SDL/include")
include_directories("3rd_party/kuafu/3rd_party/KTX-Software/include")

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")
add_custom_target(shader COMMAND cp -r ${CMAKE_CURRENT_SOURCE_DIR}/glsl_shader ${CMAKE_CURRENT_BINARY_DIR})
add_library(sapien STATIC ${PROJECT_SRC})
add_dependencies(sapien shader)

if (EXISTS ${OPTIX_HOME})
    add_dependencies(sapien ptx_dir)
endif ()

if (MACOSX)
    target_link_libraries(sapien
            libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
            libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
            libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
            libPhysX_static_64.a libPhysXVehicle_static_64.a
            libSnippetRender_static_64.a libSnippetUtils_static_64.a
            pthread ${OPENGL_LIBRARY} glfw GLEW ${ASSIMP_LIBRARIES} dl optifuser
            tinyxml2 ${OPTIX_LIBRARY} ${SPDLOG_LIBRARIES} ${EASY_PROFILER_LIBRARY}
			${PINOCCHIO_LIBRARY} svulkan2 kuafu "-framework Cocoa -framework IOKit")
else ()
    target_link_libraries(sapien -Wl,--start-group
            libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
            libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
            libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
            libPhysX_static_64.a libPhysXVehicle_static_64.a
            libSnippetRender_static_64.a libSnippetUtils_static_64.a -Wl,--end-group
            pthread ${OPENGL_LIBRARY} glfw GLEW ${ASSIMP_LIBRARIES} dl optifuser
            stdc++fs tinyxml2 ${OPTIX_LIBRARY} ${SPDLOG_LIBRARIES} ${EASY_PROFILER_LIBRARY}
            ${PINOCCHIO_LIBRARY} svulkan2 kuafu)
endif ()

set_target_properties(optifuser PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(sapien PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

pybind11_add_module(pysapien "python/pysapien.cpp" NO_EXTRAS)
target_link_libraries(pysapien PRIVATE sapien)

file(GLOB_RECURSE UNITTEST_SRC unittest/*.cpp)

add_executable(unittest ${UNITTEST_SRC})
target_link_libraries(unittest sapien)

add_executable(manual_actor manualtest/actor.cpp python/pysapien_content.hpp)
target_link_libraries(manual_actor sapien)
add_executable(manual_articulation manualtest/articulation.cpp)
target_link_libraries(manual_articulation sapien)
add_executable(manual_urdf manualtest/urdf.cpp)
target_link_libraries(manual_urdf sapien ${PINOCCHIO_LIBRARY})
add_executable(manual_kinematics manualtest/kinematics.cpp)
target_link_libraries(manual_kinematics sapien)
add_executable(manual_kuafu_minimal manualtest/kuafu_minimal.cpp)
target_link_libraries(manual_kuafu_minimal sapien)

add_custom_target(python_test COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/test/*.py ${CMAKE_CURRENT_SOURCE_DIR}/test/*.json ${CMAKE_CURRENT_BINARY_DIR})
add_custom_target(manual_python COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/manualtest/*.py ${CMAKE_CURRENT_BINARY_DIR})

