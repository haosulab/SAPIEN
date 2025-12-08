#!/bin/bash

[[ $(python -V) =~ ^Python\ 3\.([0-9]+)\..*$ ]] || echo failed to detect Python version

PYTHON_VERSION=3${BASH_REMATCH[1]}

echo Python "${PYTHON_VERSION}" detected

docker run -v "$(pwd)":/workspace/SAPIEN -it --rm \
       -u "$(id -u "${USER}")":"$(id -g "${USER}")" \
       fxiangucsd/sapien-build-env:3.8 bash -c "export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL} && cd /workspace/SAPIEN && ./scripts/build.sh ${PYTHON_VERSION} --debug"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd wheelhouse || exit 1
pip3 uninstall -y sapien
pip3 install ./*

cd /tmp && rm stubs -rf && python3 "${DIR}/python/stubgen.py" sapien.core --ignore-invalid all
cp /tmp/stubs/sapien/core-stubs/__init__.pyi "${DIR}/python/py_package/core"
cp -r /tmp/stubs/sapien/core-stubs/pysapien "${DIR}/python/py_package/core"
