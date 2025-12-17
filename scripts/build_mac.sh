#!/usr/bin/env bash

function build() {
  echo "Building wheel"

  PY_VERSION=$1
  if [ "$PY_VERSION" -eq 39 ]; then
      PY_DOT=3.9.20
  elif [ "$PY_VERSION" -eq 310 ]; then
      PY_DOT=3.10.16
  elif [ "$PY_VERSION" -eq 311 ]; then
      PY_DOT=3.11.10
  elif [ "$PY_VERSION" -eq 312 ]; then
      PY_DOT=3.12.4
  elif [ "$PY_VERSION" -eq 313 ]; then
      PY_DOT=3.13.2
  elif [ "$PY_VERSION" -eq 314 ]; then
      PY_DOT=3.14.2
  else
    echo "Error, python version not found!"
  fi
  
  if pyenv versions | grep -q "${PY_DOT}"; then
    echo "Version ${PY_DOT} is installed."
  else
    pyenv install "${PY_DOT}"
  fi
  pyenv global "${PY_DOT}"
  pyenv rehash
  pyenv exec python --version

  pyenv exec pip install setuptools wheel
  pyenv exec python setup.py bdist_wheel --build-dir=build --plat-name macosx_12_0_universal2
}

build 310
build 311
build 312
build 313
build 314