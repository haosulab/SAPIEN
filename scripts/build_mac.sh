#!/usr/bin/env bash
set -euo pipefail

function build() {
  PY_MAJOR_MINOR=$1
  PY_DOT=$(pyenv latest -k "${PY_MAJOR_MINOR}") || {
    echo "ERROR: pyenv has no version matching ${PY_MAJOR_MINOR}" >&2
    return 1
  }
  echo "Building wheel for ${PY_DOT}"

  if ! pyenv versions | grep -q "${PY_DOT}"; then
    echo "Version ${PY_DOT} is not installed, installing"
    pyenv install "${PY_DOT}" || return 1
  fi
  pyenv global "${PY_DOT}" || return 1
  pyenv rehash
  pyenv exec python --version

  ACTUAL_MINOR=$(pyenv exec python -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')") || return 1
  if [[ "${ACTUAL_MINOR}" != "${PY_MAJOR_MINOR}" ]]; then
    echo "ERROR: pyenv exec resolved to Python ${ACTUAL_MINOR}, expected ${PY_MAJOR_MINOR}" >&2
    return 1
  fi

  pyenv exec pip install setuptools wheel || return 1
  pyenv exec python setup.py bdist_wheel --build-dir=build --plat-name macosx_12_0_universal2 || return 1

  CPTAG="cp${PY_MAJOR_MINOR/./}"
  ls "dist/"*-"${CPTAG}"-"${CPTAG}"-macosx*.whl || {
    echo "ERROR: no ${CPTAG} macOS wheel produced" >&2
    return 1
  }
}

# Each version gets one retry: transient upstream failures (gitlab.com
# overload killed the eigen FetchContent clone in a past run) abort the leg,
# and the CMake stamp mechanism resumes cleanly on the next attempt. The
# "|| return 1" guards carry the failures because set -e is suppressed inside
# functions invoked in a tested context such as "f || g".
function build_with_retry() {
  build "$1" || {
    echo "Leg $1 failed, retrying once after 2 minutes" >&2
    sleep 120
    build "$1"
  }
}

build_with_retry 3.11
build_with_retry 3.12
build_with_retry 3.13
build_with_retry 3.14
build_with_retry 3.15
