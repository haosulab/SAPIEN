cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(sapien LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0 -Wall")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_DEBUG_POSTFIX "")

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
option(SAPIEN_PROFILE "Build SAPIEN with easy_profiler" OFF)
option(SAPIEN_DEBUG_VIEWER "Build debug viewer for debugging renderer in C++" OFF)
option(SAPIEN_CUDA "Enable SAPIEN CUDA functionalities, including dlpack, CUDA buffer, denoiser, and simsense" ON)
set(SAPIEN_PHYSX5_DIR "" CACHE STRING "Directory for precompield PhysX5")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif ()

if (UNIX)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        message("-- DEBUG Build")
        add_definitions(-DVK_VALIDATION)
        add_definitions(-DVK_CORE_LOGGING)
        add_definitions(-D_DEBUG)
    else()
        add_definitions(-DNDEBUG)
    endif()
endif()

if (WIN32)
    # We will only build release version for Windows
    add_definitions(-DNDEBUG)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

macro(FetchContent_MakeAvailableExclude)
    FetchContent_GetProperties(${ARGV})
    if(NOT ${ARGV}_POPULATED)
        FetchContent_Populate(${ARGV})
        add_subdirectory(${${ARGV}_SOURCE_DIR} ${${ARGV}_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
endmacro()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(physx5)
include(zlib)
include(grpc)

# TODO: on Windows, manually link physx5 and grpc

include(tinyxml2)
include(eigen)
include(vulkan)

if (${SAPIEN_DEBUG_VIEWER})
    add_definitions(-D_DEBUG_VIEWER)
endif ()

if (${SAPIEN_PROFILE})
    set(SVULKAN2_PROFILE ON CACHE BOOL "" FORCE)
else ()
    set(SVULKAN2_PROFILE OFF CACHE BOOL "" FORCE)
endif ()

if (${SAPIEN_CUDA})
    find_package(CUDA REQUIRED)
    set(SVULKAN2_CUDA_INTEROP ON CACHE BOOL "" FORCE)
else()
    set(SVULKAN2_CUDA_INTEROP OFF CACHE BOOL "" FORCE)
endif()

add_subdirectory("3rd_party/sapien-vulkan-2" EXCLUDE_FROM_ALL)

if (${SAPIEN_CUDA})
    add_subdirectory("3rd_party/simsense")
    set_target_properties(simsense PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    target_compile_definitions(simsense PUBLIC SAPIEN_SIMSENSE)
    target_include_directories(simsense PRIVATE "3rd_party/dlpack/include" "$ENV{CUDA_PATH}/include")
endif ()

file(GLOB_RECURSE SAPIEN_SRC "src/*.h" "src/*.cpp" "src/*.cc")
add_library(sapien SHARED ${SAPIEN_SRC})

target_link_libraries(sapien PUBLIC eigen)
target_link_libraries(sapien PUBLIC svulkan2)
target_link_libraries(sapien PRIVATE physx5)
target_link_libraries(sapien PRIVATE tinyxml2 spdlog::spdlog easy_profiler
    grpc++ ${CMAKE_DL_LIBS})

if (UNIX)
    target_link_libraries(sapien PRIVATE stdc++fs)
endif ()


target_link_libraries(sapien PRIVATE Vulkan::Headers glm assimp::assimp)

if (${SAPIEN_CUDA})
    target_compile_definitions(sapien PUBLIC SAPIEN_DLPACK SAPIEN_CUDA)
    target_include_directories(sapien PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/dlpack/include> "$ENV{CUDA_PATH}/include")
endif()
target_include_directories(sapien PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(sapien PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

set_target_properties(sapien PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(sapien PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

if (${SAPIEN_PROFILE})
    target_compile_definitions(sapien SAPIEN_PROFILE)
endif()

target_include_directories(sapien INTERFACE
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/physx/include>
)

if (${SAPIEN_CUDA})
    install(
        TARGETS sapien svulkan2 simsense
        EXPORT sapienTargets
    )
else()
    install(
        TARGETS sapien svulkan2
        EXPORT sapienTargets
    )
endif()

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/sapien" DESTINATION "include")
install(DIRECTORY "${physx5_SOURCE_DIR}/physx/include" DESTINATION "include/physx")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/dlpack/include/dlpack" DESTINATION "include")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/sapien-vulkan-2/include/svulkan2" DESTINATION "include")
install(DIRECTORY "${glm_SOURCE_DIR}/glm" DESTINATION "include")
install(DIRECTORY "${vulkan_SOURCE_DIR}/include/vulkan" DESTINATION "include")
install(DIRECTORY "${vulkan_SOURCE_DIR}/include/vk_video" DESTINATION "include")

if (${SAPIEN_CUDA})
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/simsense/include/simsense" DESTINATION "include")
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/sapienConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/sapienConfig.cmake"
    INSTALL_DESTINATION "lib/cmake/sapien"
)
install(
    EXPORT sapienTargets
    FILE sapienTargets.cmake
    DESTINATION "lib/cmake/sapien"
    NAMESPACE sapien::
)
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/sapienConfig.cmake"
    DESTINATION "lib/cmake/sapien"
)
