#!/bin/bash

[[ `python -V` =~ ^Python\ 3\.([0-9]+)\..*$ ]] || echo failed to detect Python version

PYTHON_VERSION=3${BASH_REMATCH[1]}

echo Python ${PYTHON_VERSION} detected

rm -rf sapien.egg-info
rm -f wheelhouse/*.whl

docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       -u $(id -u ${USER}):$(id -g ${USER}) \
       fxiangucsd/sapien-build-env:3.2 bash -c "cd /workspace/SAPIEN && ./build.sh ${PYTHON_VERSION}"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd wheelhouse
pip3 uninstall -y sapien
pip3 install *

cd /tmp && rm stubs -rf && python3 ${DIR}/python/stubgen.py sapien --ignore-invalid all
cp -r /tmp/stubs/sapien-stubs/__init__.pyi ${DIR}/python/py_package
cp -r /tmp/stubs/sapien-stubs/pysapien ${DIR}/python/py_package
