#!/bin/bash
docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       -u $(id -u ${USER}):$(id -g ${USER}) \
       fxiangucsd/sapien-build-env:1.5 bash -c "cd /workspace/SAPIEN && ./build_all.sh --profile"
