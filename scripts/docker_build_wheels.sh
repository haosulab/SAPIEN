#!/bin/bash

docker run -v "$(pwd)":/workspace/SAPIEN -it --rm \
       -u "$(id -u "${USER}")":"$(id -g "${USER}")" \
       fxiangucsd/sapien-build-env:3.8 bash -c \
       "export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL} && cd /workspace/SAPIEN && ./scripts/build.sh $1 --profile"
