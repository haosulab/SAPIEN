cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(simsense LANGUAGES CUDA)

set(CMAKE_CUDA_STANDARD 20)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3 -lineinfo -ftemplate-depth=1024")

file(GLOB_RECURSE SIMSENSE_SRC "src/*")
add_library(simsense SHARED ${SIMSENSE_SRC})
set_target_properties(simsense PROPERTIES CUDA_ARCHITECTURES "60;61;70;75;80;86") # Supporting GPU compute capabilities

target_include_directories(simsense PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
