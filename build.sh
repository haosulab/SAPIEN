#!/usr/bin/env bash

VERSION=""
DEBUG=false
PROFILE=false
KUAFU=true
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --debug) DEBUG=true;;
        --profile) PROFILE=true;;
        --no-kuafu) KUAFU=false;;
        35) VERSION="35";;
        36) VERSION="36";;
        37) VERSION="37";;
        38) VERSION="38";;
        39) VERSION="39";;
        310) VERSION="310";;
        311) VERSION="311";;
    esac
    shift
done

[ -z $VERSION ] && echo "invalid version" && exit || echo "Compile for Python ${VERSION}"
[ $DEBUG == true ] && echo "Debug Mode" || [ $PROFILE == true ] && echo "Profile Mode"  || echo "Release Mode"

KUAFU_ARG=""
[ $KUAFU == false ] && echo "Kuafu disabled" && KUAFU_ARG=" --no-kuafu"

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
  elif [ "$PY_VERSION" -eq 310 ]; then
      PY_DOT=3.10
      EXT=""
  elif [ "$PY_VERSION" -eq 311 ]; then
      PY_DOT=3.11
      EXT=""
  else
    echo "Error, python version not found!"
  fi

  INCLUDE_PATH=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/include/python${PY_DOT}${EXT}
  BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/bin/python
  echo "Using bin path ${BIN}"
  echo "Using include path ${INCLUDE_PATH}"

  export CPLUS_INCLUDE_PATH=$INCLUDE_PATH
  COMMAND="${BIN} setup.py bdist_wheel ${KUAFU_ARG}"
  [ $PROFILE == true ] && COMMAND="${BIN} setup.py bdist_wheel --profile ${KUAFU_ARG}"
  [ $DEBUG == true ] && COMMAND="${BIN} setup.py bdist_wheel --debug ${KUAFU_ARG}"
  echo "Running command ${COMMNAD}"
  eval "$COMMAND"

  PACKAGE_VERSION=$(<./python/VERSION)
  echo "SAPIEN version ${PACKAGE_VERSION}"
  WHEEL_NAME="./dist/sapien-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}${EXT}-linux_x86_64.whl"
  if test -f "$WHEEL_NAME"; then
    echo "$FILE exist, begin audit and repair"
  fi
  auditwheel repair ${WHEEL_NAME} --exclude libvulkan --internal libsapien
}

build_manylinux14_wheel $VERSION
