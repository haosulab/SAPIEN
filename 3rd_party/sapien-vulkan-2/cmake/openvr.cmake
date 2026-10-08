include(FetchContent)
FetchContent_Declare(
    openvr
    GIT_REPOSITORY https://github.com/comoco-xiao/openvr.git
    GIT_TAG        v0.0.1
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

# FetchContent_MakeAvailable(openvr)
if(NOT openvr_POPULATED)
  FetchContent_Populate(openvr)
  add_subdirectory(${openvr_SOURCE_DIR} ${openvr_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

if (MSVC)
  target_compile_definitions(openvr_api64 INTERFACE OPENVR_BUILD_STATIC)
endif()

# HACK: disable openvr log
if (NOT MSVC)
  target_compile_options(openvr_api PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvr_definitions.h)
endif()