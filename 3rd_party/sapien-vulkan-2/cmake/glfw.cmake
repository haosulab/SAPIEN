include(FetchContent)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.3
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)


set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_GetProperties(glfw)
if(NOT glfw_POPULATED)
  FetchContent_Populate(glfw)
  add_subdirectory(${glfw_SOURCE_DIR} ${glfw_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

set_target_properties(glfw PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(glfw INTERFACE GLFW_INCLUDE_VULKAN)

# apply patch
if(APPLE)
  set(PATCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/patches/glfw-3.3.3-for-apple.patch")
  execute_process(
      COMMAND git apply --check ${PATCH_FILE}
      WORKING_DIRECTORY ${glfw_SOURCE_DIR}
      RESULT_VARIABLE PATCH_RESULT
      OUTPUT_QUIET
      ERROR_QUIET
  )
  if(PATCH_RESULT EQUAL 0)
      execute_process(
          COMMAND git apply ${PATCH_FILE}
          WORKING_DIRECTORY ${glfw_SOURCE_DIR}
          RESULT_VARIABLE APPLY_RESULT
          OUTPUT_VARIABLE APPLY_OUTPUT
          ERROR_VARIABLE APPLY_ERROR
      )
      if(NOT APPLY_RESULT EQUAL 0)
          message(FATAL_ERROR "Failed to apply glfw patch:\n${APPLY_OUTPUT}\n${APPLY_ERROR}")
      else()
          message(STATUS "Glfw patch applied successfully.")
      endif()
  else()
      message(STATUS "Glfw patch already applied.")
  endif()
endif()