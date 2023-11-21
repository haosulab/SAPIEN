cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(sapien-render-server LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0 -Wall")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "")

add_compile_definitions("$<$<CONFIG:DEBUG>:_DEBUG>")
add_compile_definitions("$<$<NOT:$<CONFIG:Debug>>:NDEBUG>")

set(CMAKE_DEBUG_POSTFIX "")

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(eigen)
include(grpc)
include(pybind11)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/proto proto/render_server.proto)
grpc_generate_cpp(GRPC_SRCS GRPC_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/proto proto/render_server.proto)

find_package(sapien REQUIRED)

file(GLOB_RECURSE RENDER_SERVER_SRC "src/*.cpp")

pybind11_add_module(pysapien_render_server ${RENDER_SERVER_SRC} ${PROTO_SRCS} ${GRPC_SRCS} NO_EXTRAS)
target_link_libraries(pysapien_render_server PRIVATE grpc++ sapien::sapien eigen)
target_include_directories(pysapien_render_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
