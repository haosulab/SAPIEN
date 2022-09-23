#!/bin/bash

DIR=$(dirname "$(readlink -f "${BASH_SOURCE:-$_}" )")

protoc -I ${DIR} --cpp_out=${DIR} render_server.proto
protoc -I ${DIR} --grpc_out=${DIR} --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` render_server.proto
