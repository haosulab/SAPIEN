#!/bin/bash
docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       fxiangucsd/sapien-build-env bash -c "cd /workspace/SAPIEN && rm -rf build wheelhouse dist sapien.egg-info"
