#!/bin/bash

rm -rf sapien.egg-info
rm -f dist/*.whl

python3 setup.py bdist_wheel --debug --profile

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd dist || exit 1
pip3 uninstall -y sapien
pip3 install ./*

cd /tmp && rm stubs -rf && python3 "${DIR}/python/stubgen.py" sapien
cp -r /tmp/stubs/sapien/__init__.pyi "${DIR}/python/py_package"
cp -r /tmp/stubs/sapien/physx.pyi "${DIR}/python/py_package/physx/__init__.pyi"
cp -r /tmp/stubs/sapien/pysapien_pinocchio.pyi "${DIR}/python/py_package"
cp -r /tmp/stubs/sapien/pysapien "${DIR}/python/py_package"
