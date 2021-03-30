#!/bin/bash
docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       -u $(id -u ${USER}):$(id -g ${USER}) \
       fxiangucsd/sapien-build-env:latest bash -c "cd /workspace/SAPIEN && ./build_all.sh"
