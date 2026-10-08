#!/usr/bin/env bash

function build() {
  echo "Building wheel"

  PY_VERSION=$1
  case "$PY_VERSION" in
    311) PY_MAJOR_MINOR="3.11";;
    312) PY_MAJOR_MINOR="3.12";;
    313) PY_MAJOR_MINOR="3.13";;
    314) PY_MAJOR_MINOR="3.14";;
    315) PY_MAJOR_MINOR="3.15";;
    *)
      echo "Error, python version not supported!"
      return 1
      ;;
  esac
  PY_DOT=$(pyenv latest -k "${PY_MAJOR_MINOR}")
  
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

build 311
build 312
build 313
build 314
build 315