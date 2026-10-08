include(FetchContent)
FetchContent_Declare(
    spirv-cross
    GIT_REPOSITORY https://github.com/KhronosGroup/SPIRV-Cross.git
    GIT_TAG sdk-1.3.243.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

set(SPIRV_CROSS_CLI OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_SKIP_INSTALL ON CACHE BOOL "" FORCE)
set(SPIRV_CROSS_FORCE_PIC ON CACHE BOOL "" FORCE)
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME spirv-cross)
FetchContent_GetProperties(spirv-cross)
if(NOT spirv-cross_POPULATED)
  FetchContent_Populate(spirv-cross)
  add_subdirectory(${spirv-cross_SOURCE_DIR} ${spirv-cross_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
