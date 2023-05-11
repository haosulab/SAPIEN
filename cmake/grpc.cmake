if(TARGET grpc++)
    return()
endif()

set(gRPC_ZLIB_PROVIDER "package")

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  grpc
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.51.1
)

set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailableExclude(grpc)
set(FETCHCONTENT_QUIET ON)
set_target_properties(grpc++ PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

set(CMAKE_CXX_STANDARD 20)
