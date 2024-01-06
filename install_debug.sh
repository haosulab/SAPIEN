#!/bin/bash

rm -rf sapien.egg-info
rm -f dist/*.whl

python3 setup.py bdist_wheel --debug --profile

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd dist
pip3 uninstall -y sapien
pip3 install *

cd /tmp && rm stubs -rf && python3 ${DIR}/python/stubgen.py sapien --ignore-invalid all
cp -r /tmp/stubs/sapien-stubs/__init__.pyi ${DIR}/python/py_package
cp -r /tmp/stubs/sapien-stubs/pysapien ${DIR}/python/py_package
cp -r /tmp/stubs/sapien-stubs/render_server ${DIR}/python/py_package
