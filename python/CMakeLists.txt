cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(sapien LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /bigobj")
else()
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -std=c++20")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0 -Wall -std=c++20")
endif()

if (APPLE)
    set(CMAKE_INSTALL_RPATH "@loader_path/libs")
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-D_DEBUG)
else ()
    add_definitions(-DNDEBUG)
endif ()

macro(FetchContent_MakeAvailableExclude)
    FetchContent_GetProperties(${ARGV})
    if(NOT ${ARGV}_POPULATED)
        FetchContent_Populate(${ARGV})
        add_subdirectory(${${ARGV}_SOURCE_DIR} ${${ARGV}_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
endmacro()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(eigen)
include(pybind11)

find_package(sapien REQUIRED)

file(GLOB_RECURSE PYBIND_SRC "pybind/*.cpp")
pybind11_add_module(pysapien ${PYBIND_SRC})
target_link_libraries(pysapien PRIVATE sapien::sapien eigen)
target_compile_definitions(pysapien PRIVATE PYBIND11_USE_SMART_HOLDER_AS_DEFAULT)

if (MSVC)
    set_target_properties(pysapien PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set_target_properties(pysapien PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set_target_properties(pysapien PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endif ()

if (TARGET sapien::simsense)
    target_link_libraries(pysapien PRIVATE sapien::simsense)
endif ()
