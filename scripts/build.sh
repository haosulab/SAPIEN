#!/usr/bin/env bash

VERSION=
DEBUG=
PROFILE=
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --debug) DEBUG=1;;
        --profile) PROFILE=1;;
        311) VERSION="311";;
        312) VERSION="312";;
        313) VERSION="313";;
        314) VERSION="314";;
        315) VERSION="315";;
    esac
    shift
done

[ -z $VERSION ] && echo "Version not specified, building for all versions" || echo "Compile for Python ${VERSION}"
( [ $DEBUG ] && echo "Debug Mode" ) || ( [ $PROFILE ] && echo "Profile Mode" )  || echo "Release Mode"

function build_sapien() {
  echo "Building SAPIEN"
  BIN=/opt/python/cp311-cp311/bin/python
  COMMAND="${BIN} setup.py bdist_wheel"
  [ $PROFILE ] && COMMAND="${BIN} setup.py bdist_wheel --profile"
  [ $DEBUG ] && COMMAND="${BIN} setup.py bdist_wheel --debug"
  eval "${COMMAND} --sapien-only --build-dir=docker_sapien_build"
}

function build_pybind() {
  echo "Building Pybind"

  PY_VERSION=$1
  if [ "$PY_VERSION" -eq 311 ]; then
      PY_DOT=3.11
      EXT=""
  elif [ "$PY_VERSION" -eq 312 ]; then
      PY_DOT=3.12
      EXT=""
  elif [ "$PY_VERSION" -eq 313 ]; then
      PY_DOT=3.13
      EXT=""
  elif [ "$PY_VERSION" -eq 314 ]; then
      PY_DOT=3.14
      EXT=""
  elif [ "$PY_VERSION" -eq 315 ]; then
      PY_DOT=3.15
      EXT=""
  else
    echo "Error, python version not found!"
  fi

  INCLUDE_PATH=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/include/python${PY_DOT}${EXT}
  BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/bin/python
  export CPLUS_INCLUDE_PATH=${INCLUDE_PATH}
  COMMAND="${BIN} setup.py bdist_wheel"
  eval "${COMMAND} --pybind-only --build-dir=docker_sapien_build"

  PACKAGE_VERSION=$(${BIN} setup.py --get-version)
  WHEEL_NAME="./dist/sapien-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}${EXT}-linux_x86_64.whl"
  if test -f "$WHEEL_NAME"; then
    echo "$FILE exist, begin audit and repair"
  fi
  # The X11 stack must stay external: a hash-mangled vendored libX11 beside
  # the system libxcb runs two different xcb implementations over one X
  # connection and corrupts the heap inside libGLX_nvidia. The C/C++ runtime
  # is excluded for the same single-instance reason. (Same configuration as
  # the internal wheel build.)
  auditwheel repair "${WHEEL_NAME}" \
    --exclude 'libvulkan*' --exclude 'libOpenImageDenoise*' \
    --exclude 'libstdc++*' --exclude 'libgcc_s*' \
    --exclude 'libc.so*' --exclude 'libm.so*' --exclude 'libpthread*' \
    --exclude 'libdl*' --exclude 'librt*' --exclude 'ld-linux*' \
    --exclude 'libX11*' --exclude 'libxcb*' --exclude 'libXau*' \
    --exclude 'libXdmcp*' --exclude 'libbsd*' --exclude 'libmd*' \
    --internal libsapien --internal libsvulkan2
}

build_sapien
if [ -z "${VERSION}" ]
then
   build_pybind 311
   build_pybind 312
   build_pybind 313
   build_pybind 314
   build_pybind 315
else
   build_pybind $VERSION
fi
