#!/bin/bash
docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       fxiangucsd/sapien-build-env bash -c "cd /workspace/SAPIEN && ./build_all_optix.sh"
