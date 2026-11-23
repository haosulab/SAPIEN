include(FetchContent)

FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG        v1.2.11
    OVERRIDE_FIND_PACKAGE
)
FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/fbxiang/assimp.git
    GIT_TAG        0ea31aa6734336dc1e62c6d9bde3e49b6d71b811
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    # PATCH_COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_LIST_DIR}/assimp.patch.CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt  # patch for MSVC
)

FetchContent_GetProperties(zlib)
if(NOT zlib_POPULATED)
  FetchContent_Populate(zlib)
  add_subdirectory(${zlib_SOURCE_DIR} ${zlib_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
set_target_properties(zlibstatic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set(ZLIB_FOUND TRUE)
set(ZLIB_INCLUDE_DIR "${zlib_SOURCE_DIR} ${zlib_BINARY_DIR}")

add_library(minizip STATIC
  "${zlib_SOURCE_DIR}/contrib/minizip/minizip.c"
  "${zlib_SOURCE_DIR}/contrib/minizip/unzip.c"
  "${zlib_SOURCE_DIR}/contrib/minizip/ioapi.c"
)
target_link_libraries(minizip PRIVATE zlibstatic)
target_include_directories(minizip PUBLIC "$<BUILD_INTERFACE:${zlib_SOURCE_DIR}/contrib/minizip>")
install(TARGETS minizip EXPORT assimpTargets)

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_DRACO ON CACHE BOOL "" FORCE)


FetchContent_GetProperties(assimp)
if(NOT assimp_POPULATED)
  FetchContent_Populate(assimp)
  add_subdirectory(${assimp_SOURCE_DIR} ${assimp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

set_target_properties(assimp PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# the following seems needed by MSVC
target_include_directories(assimp PUBLIC $<BUILD_INTERFACE:${zlib_SOURCE_DIR}> $<BUILD_INTERFACE:${zlib_BINARY_DIR}>)
target_link_libraries(assimp zlibstatic)
