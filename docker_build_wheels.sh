#!/bin/bash

docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       -u $(id -u ${USER}):$(id -g ${USER}) \
       fxiangucsd/sapien-build-env:3.0 bash -c \
       "export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL} && cd /workspace/SAPIEN && ./build.sh $1"
