# SAPIEN wheel build images, used by the public linux wheel builds
# (.github/workflows/build.yml) and published to GHCR:
#   ghcr.io/haosulab/sapien-build-env:<tag>            x86_64  (CUDA 12.8)
#   ghcr.io/haosulab/sapien-build-env:<tag>-aarch64    aarch64 (CPU-only)
#
# Built by the "Build Build-Env Image" workflow in this repo
# (.github/workflows/docker-image.yml) with the built-in GITHUB_TOKEN -
# no repository secrets anywhere. Replaces the legacy fxiangucsd Docker Hub
# images (CUDA 11.8 runfile at /workspace/cuda, auditwheel fe61bcdf hack
# fork baked on cp313).
#
# Why manylinux_2_28 (AlmaLinux 8, glibc 2.28): the repaired wheels must
# honestly carry the manylinux_2_28 tags (PEP 600). Building in the pypa
# base image caps every artifact at glibc 2.28.
#
# Contents:
#   - gcc-toolset-11 toolchain (CC/CXX pinned to it)
#   - GL / X11 / Vulkan development headers + loaders. The full X11
#     family (incl. the glfw extras libXrandr/libXinerama/libXcursor/
#     libXi) must be present so svulkan2 builds against them and
#     auditwheel then resolves-and-excludes them instead of vendoring:
#     a vendored libX11 beside the system libxcb runs two xcb
#     implementations over one X connection and corrupts the heap inside
#     libGLX_nvidia.
#   - CUDA 12.8 (x86_64 only, via the same rhel8 yum repo as the PhysX
#     build image): nvcc compiles src/*.cu and OpenImageDenoise's device
#     library; sapien links $CUDA_PATH/lib64/libcudart_static.a.
#     CUDA_PATH is deliberately NOT baked as an ENV here: setup.py enables
#     SAPIEN_CUDA iff CUDA_PATH is set in the environment, so the aarch64
#     image (which has no CUDA at all) must never see it. The x86_64 CI
#     job sets CUDA_PATH itself.
#   - Python 3.11-3.15 straight from the base image (/opt/python/cp3XX):
#     current pypa manylinux images ship CPython 3.9-3.15 natively, and
#     cp315 is ABI-frozen (rc and final share the cp315 ABI), so no source
#     build or version pin is needed - the image tracks whatever cp315
#     the base ships at build time.
#   - auditwheel from the fbxiang fork (branch 6.8.2-internal): adds
#     --internal, which grafts libsapien/libsvulkan2 from the build tree
#     while keeping their original sonames (upstream renames them to
#     lib<name>-<hash>.so).

ARG BASE=quay.io/pypa/manylinux_2_28_x86_64:latest
FROM ${BASE}

# x86_64 image compiles CUDA sources; aarch64 is CPU-only
ARG WITH_CUDA=true

RUN yum install -y gcc-toolset-11 git make zip unzip wget \
      mesa-libGL-devel \
      libX11-devel libxcb-devel libXau-devel libXdmcp-devel \
      libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
      vulkan-loader-devel vulkan-headers \
      openssl-devel zlib-devel libffi-devel bzip2-devel sqlite-devel \
      readline-devel xz-devel ncurses-devel gdbm-devel libuuid-devel \
    && yum clean all

# cmake and ninja ship with the base image via its own pipx
# (/opt/_internal/pipx). Do NOT install cmake via pipx here: pipx refuses
# to replace a venv it did not create in this session and the install fails.

# keep setuptools/wheel current in every python the wheel matrix uses
# (the base ships CPython 3.9-3.15 incl. free-threaded variants under
# /opt/python; we only build wheels for cp311-cp315)
RUN /opt/python/cp311-cp311/bin/python -m pip install -U setuptools wheel \
    && /opt/python/cp312-cp312/bin/python -m pip install -U setuptools wheel \
    && /opt/python/cp313-cp313/bin/python -m pip install -U setuptools wheel \
    && /opt/python/cp314-cp314/bin/python -m pip install -U setuptools wheel \
    && /opt/python/cp315-cp315/bin/python -m pip install -U setuptools wheel

# CUDA 12.8 (x86_64 only): nvcc + runtime headers + static cudart + curand
# (simsense includes <curand.h> and links it; the old full-toolkit image had
# it, the trimmed one did not) + nvtx (parity with the windows installer
# list). The rhel8 repo's cuda-cudart-devel-12-8 carries libcudart_static.a
# (there is no separate -static package; the explicit check fails the image
# build if that ever changes). No driver bits anywhere - linking never needs
# them.
RUN if [ "${WITH_CUDA}" = "true" ]; then \
      curl -fsSL -o /etc/yum.repos.d/cuda-rhel8.repo \
        https://developer.download.nvidia.com/compute/cuda/repos/rhel8/x86_64/cuda-rhel8.repo \
      && dnf install -y cuda-nvcc-12-8 cuda-cudart-devel-12-8 cuda-curand-devel-12-8 cuda-nvtx-12-8 \
      && if ! find /usr/local/cuda-12.8 -name 'libcudart_static.a' -print -quit | grep -q .; then \
           echo "libcudart_static.a missing from cuda-cudart-devel - dnf providers:"; \
           dnf -q whatprovides '*/libcudart_static.a' || true; \
           exit 1; \
         fi \
      && dnf clean all; \
    fi

# The cuda bin dir is harmless on the CUDA-less aarch64 image. cp315's bin
# is first so plain `python`/`pip`/`auditwheel` resolve to 3.15.
ENV PATH="/opt/python/cp315-cp315/bin:/usr/local/cuda-12.8/bin:/opt/_internal/pipx/bin:/root/.local/bin:/opt/rh/gcc-toolset-11/root/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
ENV LD_LIBRARY_PATH="/opt/rh/gcc-toolset-11/root/usr/lib64:/opt/rh/gcc-toolset-11/root/usr/lib:${LD_LIBRARY_PATH}"
ENV CC=/opt/rh/gcc-toolset-11/root/usr/bin/cc CXX=/opt/rh/gcc-toolset-11/root/usr/bin/c++

# auditwheel fork: --internal keeps graft sonames intact. The console
# script lands in /opt/python/cp315-cp315/bin, which is on PATH above.
RUN /opt/python/cp315-cp315/bin/pip install "git+https://github.com/fbxiang/auditwheel@6.8.2-internal" \
    && auditwheel --version

# sanity: the toolchain must resolve from the final image PATH
RUN cmake --version && python --version \
    && { [ "${WITH_CUDA}" != "true" ] || nvcc --version; } \
    && auditwheel --version
