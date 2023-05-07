if(TARGET physx5)
  return()
endif()

# TODO: add windows version

if (IS_DIRECTORY ${SAPIEN_PHYSX5_DIR})
  # Use provided PhysX5
  set(physx5_SOURCE_DIR ${SAPIEN_PHYSX5_DIR})
else()
  # We provide a precompiled physx5 here
  include(FetchContent)
  FetchContent_Declare(
    physx5
    URL https://storage1.ucsd.edu/datasets/PhysX5_compiled.zip
    URL_HASH MD5=1cd1243a81d7793cc7f66ec24a1f0641
  )
  FetchContent_MakeAvailable(physx5)
endif()

add_library(physx5 INTERFACE)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_link_directories(physx5 INTERFACE "${physx5_SOURCE_DIR}/physx/bin/linux.clang/checked")
else()
  target_link_directories(physx5 INTERFACE "${physx5_SOURCE_DIR}/physx/bin/linux.clang/checked")
endif()

target_link_libraries(physx5 INTERFACE
        -Wl,--start-group
        libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
        libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
        libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
        libPhysX_static_64.a libPhysXVehicle_static_64.a
        libSnippetRender_static_64.a libSnippetUtils_static_64.a
        -Wl,--end-group)
target_include_directories(physx5 INTERFACE "${physx5_SOURCE_DIR}/physx/include")
