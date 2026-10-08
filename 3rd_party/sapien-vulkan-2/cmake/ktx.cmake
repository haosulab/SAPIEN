include(FetchContent)

FetchContent_Declare(
    ktx
    GIT_REPOSITORY https://github.com/KhronosGroup/KTX-Software.git
    GIT_TAG        v4.3.2
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

set(KTX_FEATURE_TESTS OFF CACHE INTERNAL "")
set(KTX_FEATURE_STATIC_LIBRARY ON CACHE INTERNAL "build static")
FetchContent_GetProperties(ktx)
if(NOT ktx_POPULATED)
  FetchContent_Populate(ktx)
  add_subdirectory(${ktx_SOURCE_DIR} ${ktx_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
set_target_properties(ktx PROPERTIES POSITION_INDEPENDENT_CODE ON)
