#!/usr/bin/env bash

DEBUG=false
PROFILE=false
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --debug) DEBUG=true;;
        --profile) PROFILE=true;;
    esac
    shift
done

[ $DEBUG == true ] && echo "Debug Mode" || [ $PROFILE == true ] && echo "Profile Mode"  || echo "Release Mode"

function build_manylinux14_wheel() {
  PY_VERSION=$1
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
  elif [ "$PY_VERSION" -eq 39 ]; then
      PY_DOT=3.9
      EXT=""
  else
    echo "Error, python version not found!"
  fi

  INCLUDE_PATH=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/include/python${PY_DOT}${EXT}
  BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/bin/python
  echo "Using bin path ${BIN}"
  echo "Using include path ${INCLUDE_PATH}"

  export CPLUS_INCLUDE_PATH=$INCLUDE_PATH
  COMMAND="${BIN} setup.py bdist_wheel"
  [ $PROFILE == true ] && COMMAND="${BIN} setup.py bdist_wheel --profile"
  [ $DEBUG == true ] && COMMAND="${BIN} setup.py bdist_wheel --debug"
  echo "Running command ${COMMNAD}"
  eval "$COMMAND"

  PACKAGE_VERSION=$(<./python/VERSION)
  echo "SAPIEN verion ${PACKAGE_VERSION}"
  WHEEL_NAME="./dist/sapien-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}${EXT}-linux_x86_64.whl"
  if test -f "$WHEEL_NAME"; then
    echo "$FILE exist, begin audit and repair"
  fi
  WHEEL_COMMAND="auditwheel repair ${WHEEL_NAME} --exclude libvulkan"
  eval "$WHEEL_COMMAND"
}

build_manylinux14_wheel 37
build_manylinux14_wheel 38
build_manylinux14_wheel 39
rm -rf build
