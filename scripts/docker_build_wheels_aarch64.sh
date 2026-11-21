#!/bin/bash

docker run --platform linux/arm64 -v `pwd`:/workspace/SAPIEN -it --rm \
       -u "$(id -u "${USER}")":"$(id -g "${USER}")" \
       ghcr.io/haosulab/sapien-build-env:1.0-aarch64 bash -c \
       "export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL} && cd /workspace/SAPIEN && ./scripts/build_aarch64.sh $1 --profile"
