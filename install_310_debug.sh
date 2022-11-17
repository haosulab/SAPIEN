#!/bin/bash

rm -rf sapien.egg-info
rm -f dist/*.whl
if [ -z ${VERSION} ]
then
    echo VERSION variable is not specified
    VERSION=2.0.0.dev$(date +"%Y%m%d")
    echo VERSION defatuls to ${VERSION}
fi

echo ${VERSION} > python/VERSION
echo __version__=\"${VERSION}\" > python/py_package/version.py

python3 setup.py bdist_wheel --debug

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd dist
pip3 uninstall -y sapien
pip3 install *

cd /tmp && rm stubs -rf && pybind11-stubgen sapien.core --ignore-invalid all
cp /tmp/stubs/sapien/core-stubs/__init__.pyi $DIR/python/py_package/core
cp -r /tmp/stubs/sapien/core-stubs/pysapien $DIR/python/py_package/core
