#!/bin/bash

docker run -v "$(pwd)":/workspace/SAPIEN -it --rm \
       -u "$(id -u "${USER}")":"$(id -g "${USER}")" \
       -e CUDA_PATH=/usr/local/cuda-12.8 \
       ghcr.io/haosulab/sapien-build-env:1.0 bash -c \
       "export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL} && cd /workspace/SAPIEN && ./scripts/build.sh $1 --profile"
