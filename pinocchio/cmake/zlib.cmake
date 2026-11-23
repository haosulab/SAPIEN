if(TARGET zlibstatic)
    return()
endif()

include(FetchContent)
find_package(Git REQUIRED)

FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG        v1.2.11
    PATCH_COMMAND  ${CMAKE_COMMAND} "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}" "-DSOURCE_DIR=<SOURCE_DIR>" -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/zlib-apple-fdopen.cmake"
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(zlib)
set_target_properties(zlibstatic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
set(ZLIB_LIBRARY zlibstatic)
add_library(ZLIB::ZLIB ALIAS zlibstatic)
