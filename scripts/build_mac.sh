#!/usr/bin/env bash
set -euo pipefail

function build() {
  PY_MAJOR_MINOR=$1
  PY_DOT=$(pyenv latest -k "${PY_MAJOR_MINOR}")
  echo "Building wheel for ${PY_DOT}"

  if ! pyenv versions | grep -q "${PY_DOT}"; then
    echo "Version ${PY_DOT} is not installed, installing"
    pyenv install "${PY_DOT}"
  fi
  pyenv global "${PY_DOT}"
  pyenv rehash
  pyenv exec python --version

  ACTUAL_MINOR=$(pyenv exec python -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')")
  if [[ "${ACTUAL_MINOR}" != "${PY_MAJOR_MINOR}" ]]; then
    echo "ERROR: pyenv exec resolved to Python ${ACTUAL_MINOR}, expected ${PY_MAJOR_MINOR}" >&2
    exit 1
  fi

  pyenv exec pip install setuptools wheel
  pyenv exec python setup.py bdist_wheel --build-dir=build --plat-name macosx_12_0_universal2

  CPTAG="cp${PY_MAJOR_MINOR/./}"
  if ! ls dist/*-${CPTAG}-${CPTAG}-macosx*.whl >/dev/null 2>&1; then
    echo "ERROR: no ${CPTAG} macOS wheel produced" >&2
    exit 1
  fi
}

build 3.11
build 3.12
build 3.13
build 3.14
build 3.15
