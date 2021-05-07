#!/bin/bash
echo 1.0.dev_$(date +"%m_%d_%y") > python/VERSION

docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       -u $(id -u ${USER}):$(id -g ${USER}) \
       fxiangucsd/sapien-build-env:1.6 bash -c "cd /workspace/SAPIEN && ./build.sh 38 --debug"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

pip3 uninstall sapien -y && cd wheelhouse && pip3 install *
cd /tmp && rm stubs -rf && pybind11-stubgen sapien.core --ignore-invalid all
cp /tmp/stubs/sapien/core-stubs/__init__.pyi $DIR/python/py_package/core
cp -r /tmp/stubs/sapien/core-stubs/pysapien $DIR/python/py_package/core
