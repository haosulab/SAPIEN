cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(sapien_pinocchio LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0 -Wall")

set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
set(BUILD_PYTHON_INTERFACE  OFF CACHE BOOL "")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-D_DEBUG)
else ()
    add_definitions(-DNDEBUG)
endif ()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(pybind11)
include(zlib)
include(eigen)
include(pinocchio)

find_package(sapien REQUIRED)
pybind11_add_module(pysapien_pinocchio "pinocchio_model.cpp" NO_EXTRAS)
target_link_libraries(pysapien_pinocchio PRIVATE sapien::sapien eigen pinocchio)
