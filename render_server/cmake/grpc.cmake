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
  GIT_SHALLOW TRUE
)

set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(grpc)
set(FETCHCONTENT_QUIET ON)

set(CMAKE_CXX_STANDARD 20)

if (zlib_SOURCE_DIR)
  target_include_directories(grpc PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
  target_include_directories(grpc++ PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
endif()


# MIT License

# Copyright (c) 2018 Ivan Safonov

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

function(PROTOBUF_GENERATE_CPP SRCS HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
    return()
  endif()

  set(${SRCS})
  set(${HDRS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(_protobuf_include_path ${ABS_FIL} DIRECTORY)

    list(APPEND ${SRCS} "${DEST}/${FIL_WE}.pb.cc")
    list(APPEND ${HDRS} "${DEST}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${DEST}/${FIL_WE}.pb.cc"
             "${DEST}/${FIL_WE}.pb.h"
      COMMAND protobuf::protoc
      ARGS --cpp_out ${DEST} -I${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL} protobuf::protoc
      COMMENT "Running C++ protocol buffer compiler on ${FIL}"
      VERBATIM )
  endforeach()
  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()


function(GRPC_GENERATE_CPP SRCS HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: GRPC_GENERATE_CPP() called without any proto files")
    return()
  endif()

  set(${SRCS})
  set(${HDRS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(_protobuf_include_path ${ABS_FIL} DIRECTORY)

    list(APPEND ${SRCS} "${DEST}/${FIL_WE}.grpc.pb.cc")
    list(APPEND ${HDRS} "${DEST}/${FIL_WE}.grpc.pb.h")

    add_custom_command(
      OUTPUT "${DEST}/${FIL_WE}.grpc.pb.cc"
             "${DEST}/${FIL_WE}.grpc.pb.h"
      COMMAND protobuf::protoc
      ARGS --grpc_out ${DEST} -I${_protobuf_include_path} --plugin=protoc-gen-grpc=$<TARGET_FILE:grpc_cpp_plugin> ${ABS_FIL}
      DEPENDS ${ABS_FIL} protobuf::protoc grpc_cpp_plugin
      COMMENT "Running C++ gRPC compiler on ${FIL}"
      VERBATIM )
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()
