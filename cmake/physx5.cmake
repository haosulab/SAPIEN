if(TARGET physx5)
  return()
endif()

set(PHYSX_VERSION 105.1-physx-5.3.1.patch0)

if (IS_DIRECTORY ${SAPIEN_PHYSX5_DIR})
  # Use provided PhysX5
  set(physx5_SOURCE_DIR ${SAPIEN_PHYSX5_DIR})
else()
  # We provide a precompiled physx5 here
  include(FetchContent)
  if (APPLE)
    FetchContent_Declare(
      physx5
      URL https://github.com/sapien-sim/physx-precompiled/releases/download/${PHYSX_VERSION}/macOS-release.zip
      URL_HASH MD5=ea9409baaa4dcd8bdcfd9511889a5737
    )
  elseif (UNIX)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
      FetchContent_Declare(
        physx5
        URL https://github.com/sapien-sim/physx-precompiled/releases/download/${PHYSX_VERSION}/linux-checked.zip
        URL_HASH MD5=8379bf7ba4d6a0866404fd8a11cc10c2
      )
    else ()
      FetchContent_Declare(
        physx5
        URL https://github.com/sapien-sim/physx-precompiled/releases/download/${PHYSX_VERSION}/linux-release.zip
        URL_HASH MD5=020222e5441b9ae2779dc05b1f04539c
      )
    endif ()
  elseif (WIN32)
    FetchContent_Declare(
      physx5
      URL https://github.com/sapien-sim/physx-precompiled/releases/download/${PHYSX_VERSION}/windows-release.zip
      URL_HASH MD5=77299ac291e17df438c090d565167d93
    )
  endif()
  FetchContent_MakeAvailable(physx5)
endif()

add_library(physx5 INTERFACE)

if (APPLE)
  if(CMAKE_SYSTEM_NAME MATCHES ".*Darwin.*" OR CMAKE_SYSTEM_NAME MATCHES ".*MacOS.*")
    target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/arm64/release>)
  endif()
  
  target_link_libraries(physx5 INTERFACE
    libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
    libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
    libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
    libPhysX_static_64.a libPhysXVehicle_static_64.a
    )
  target_include_directories(physx5 SYSTEM INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/include>)
elseif(UNIX)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/linux.clang/checked>)
  else()
    target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/linux.clang/release>)
  endif()
  target_link_libraries(physx5 INTERFACE
    -Wl,--start-group
    libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
    libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
    libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
    libPhysX_static_64.a libPhysXVehicle_static_64.a
    -Wl,--end-group)
  target_include_directories(physx5 SYSTEM INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/include>)
endif()

if (WIN32)
  target_include_directories(physx5 SYSTEM INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/include>)
  target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/win.x86_64.vc143.mt/release>)
  target_link_libraries(physx5 INTERFACE
    PhysXVehicle2_static_64.lib PhysXExtensions_static_64.lib
    PhysXVehicle_static_64.lib PhysX_static_64.lib PhysXPvdSDK_static_64.lib
    PhysXCooking_static_64.lib PhysXCommon_static_64.lib
    PhysXCharacterKinematic_static_64.lib PhysXFoundation_static_64.lib)
endif()

target_compile_definitions(physx5 INTERFACE PX_PHYSX_STATIC_LIB)
target_compile_definitions(physx5 INTERFACE PHYSX_VERSION="${PHYSX_VERSION}")
