include(FetchContent)

FetchContent_Declare(
    glslang
    GIT_REPOSITORY https://github.com/KhronosGroup/glslang.git
    GIT_TAG 11.13.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

set(ENABLE_CTEST_INSTALL OFF CACHE BOOL "" FORCE)
set(SKIP_GLSLANG_INSTALL ON CACHE BOOL "" FORCE)
FetchContent_GetProperties(glslang)
if(NOT glslang_POPULATED)
  FetchContent_Populate(glslang)
  add_subdirectory(${glslang_SOURCE_DIR} ${glslang_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
set_target_properties(glslang PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(SPIRV PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
