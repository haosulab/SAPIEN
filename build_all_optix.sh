#!/usr/bin/env bash
function build_manylinux14_wheel() {
  PY_VERSION=$1
  echo "Start deal with python verion $PY_VERSION now"
  if [ "$PY_VERSION" -eq 35 ]; then
      PY_DOT=3.5
      EXT="m"
  elif [ "$PY_VERSION" -eq 36 ]; then
      PY_DOT=3.6
      EXT="m"
  elif [ "$PY_VERSION" -eq 37 ]; then
      PY_DOT=3.7
      EXT="m"
  elif [ "$PY_VERSION" -eq 38 ]; then
      PY_DOT=3.8
      EXT=""
  else
    echo "Error, python version not found!"
  fi

  INCLUDE_PATH=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/include/python${PY_DOT}${EXT}
  BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/bin/python
  echo "Using bin path ${BIN}"
  echo "Using include path ${INCLUDE_PATH}"

  export CPLUS_INCLUDE_PATH=$INCLUDE_PATH
  COMMAND="${BIN} setup.py bdist_wheel --optix-home=`pwd`/../OptiX --cuda-include-path=`pwd`/../cuda/include"
  echo "Running command ${COMMNAD}"
  eval "$COMMAND"

  PACKAGE_VERSION=$(<./python/VERSION)
  echo "SAPIEN verion ${PACKAGE_VERSION}"
  WHEEL_NAME="./dist/sapien-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}${EXT}-linux_x86_64.whl"
  if test -f "$WHEEL_NAME"; then
    echo "$FILE exist, begin audit and repair"
  fi
  WHEEL_COMMAND="auditwheel repair ${WHEEL_NAME}"
  eval "$WHEEL_COMMAND"
}

build_manylinux14_wheel 35
build_manylinux14_wheel 36
build_manylinux14_wheel 37
build_manylinux14_wheel 38
rm -rf build
