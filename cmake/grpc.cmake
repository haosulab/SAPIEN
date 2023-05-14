if(TARGET grpc++)
    return()
endif()

set(gRPC_ZLIB_PROVIDER "package")
set(gRPC_BUILD_TESTS OFF CACHE BOOL "" FORCE)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  grpc
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.51.1
  PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cares.txt <SOURCE_DIR>/third_party/cares/cares/CMakeLists.txt
)

set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailableExclude(grpc)
set(FETCHCONTENT_QUIET ON)

# if (zlib_SOURCE_DIR)
#   target_include_directories(grpc PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
#   target_include_directories(grpc++ PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
# endif()

set(CMAKE_CXX_STANDARD 20)

