include(FetchContent)
FetchContent_Declare(
    vulkan
    GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
    GIT_TAG        v1.3.250
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

FetchContent_GetProperties(vulkan)
if(NOT vulkan_POPULATED)
  FetchContent_Populate(vulkan)
  add_subdirectory(${vulkan_SOURCE_DIR} ${vulkan_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
