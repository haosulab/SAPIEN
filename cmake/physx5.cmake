if(TARGET physx5)
  return()
endif()

set(PHYSX_VERSION 112.0-physx-5.11.0.patch0)

if (IS_DIRECTORY ${SAPIEN_PHYSX5_DIR})
  # Use provided PhysX5
  set(physx5_SOURCE_DIR ${SAPIEN_PHYSX5_DIR})
else()
  # We provide a precompiled physx5 here
  include(FetchContent)
  if (APPLE)
    FetchContent_Declare(
      physx5
      URL https://github.com/sapien-sim/PhysX/releases/download/${PHYSX_VERSION}/macOS-universal-release.zip
      URL_HASH SHA256=08f7a98081d513379a4247e7bb72c7862254c871f9cca9ebdf2459a0229139a6
    )
  elseif (UNIX)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")

      FetchContent_Declare(
        physx5
        URL https://github.com/sapien-sim/PhysX/releases/download/${PHYSX_VERSION}/linux-aarch64-release.zip
        URL_HASH SHA256=d2eb7f24ca437a0809f7b7a05039581663e7545e7718f16f6f53c27b26a36ab5
      )

    else ()
      if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        FetchContent_Declare(
          physx5
          URL https://github.com/sapien-sim/PhysX/releases/download/${PHYSX_VERSION}/linux-checked.zip
          URL_HASH SHA256=04fc64bf70087783d6692de7f356d28a55a0f3f3c10a2d318362fbd5759bac46
        )
      else ()
        FetchContent_Declare(
          physx5
          URL https://github.com/sapien-sim/PhysX/releases/download/${PHYSX_VERSION}/linux-release.zip
          URL_HASH SHA256=7033ea98fa4bc48b90839352cbd0a1f529c36131fd49873d446d1630bf63c572
        )
      endif ()
    endif ()

  elseif (WIN32)
    FetchContent_Declare(
      physx5
      URL https://github.com/sapien-sim/PhysX/releases/download/${PHYSX_VERSION}/windows-release.zip
      URL_HASH SHA256=b4c7a97e6f09d75b4109fbf87f79fa756c004648da682b7ac1a5377430a8eddd
    )
  endif()
  FetchContent_MakeAvailable(physx5)
endif()

add_library(physx5 INTERFACE)

if (APPLE)
  if(CMAKE_SYSTEM_NAME MATCHES ".*Darwin.*" OR CMAKE_SYSTEM_NAME MATCHES ".*MacOS.*")
    target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/universal/release>)
  endif()
  
  target_link_libraries(physx5 INTERFACE
    libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
    libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
    libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
    libPhysX_static_64.a libPhysXVehicle_static_64.a
    )
  target_include_directories(physx5 SYSTEM INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/include>)
elseif(UNIX)

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/linux.aarch64/release>)
  else()
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
      target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/linux.clang/checked>)
    else()
      target_link_directories(physx5 INTERFACE $<BUILD_INTERFACE:${physx5_SOURCE_DIR}/bin/linux.clang/release>)
    endif()
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
    PhysXExtensions_static_64.lib
    PhysXVehicle_static_64.lib PhysX_static_64.lib PhysXPvdSDK_static_64.lib
    PhysXCooking_static_64.lib PhysXCommon_static_64.lib
    PhysXCharacterKinematic_static_64.lib PhysXFoundation_static_64.lib)
endif()

target_compile_definitions(physx5 INTERFACE PX_PHYSX_STATIC_LIB)
target_compile_definitions(physx5 INTERFACE PHYSX_VERSION="${PHYSX_VERSION}")
