#!/bin/bash
if [ -z ${VERSION} ]
then
    echo VERSION variable is not specified
    VERSION=2.0.0.dev$(date +"%Y%m%d")
    echo VERSION defatuls to ${VERSION}
    sleep 3
fi

echo ${VERSION} > python/VERSION
echo __version__=\"${VERSION}\" > python/py_package/version.py

docker run -v `pwd`:/workspace/SAPIEN -it --rm \
       -u $(id -u ${USER}):$(id -g ${USER}) \
       fxiangucsd/sapien-build-env:latest bash -c "cd /workspace/SAPIEN && ./build_all.sh"
