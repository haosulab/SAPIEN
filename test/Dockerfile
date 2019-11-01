FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.0

RUN apt update && \
  apt install -y git libeigen3-dev clang software-properties-common \
  apt-transport-https ca-certificates gnupg software-properties-common wget vim

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - && \
  apt-add-repository -y 'deb https://apt.kitware.com/ubuntu/ xenial main' && \
  apt update && apt install -y cmake

RUN apt install -y xserver-xorg-dev libxt-dev \
  libglew-dev libpython3.6-dev xorg-dev python3-distutils \
  zip libglm-dev python3-pip xvfb && \
  pip3 install numpy tifffile Pillow transforms3d

# libgl1-mesa-dev libgl1-mesa-glx libglu1-mesa libosmesa6-dev 

# WORKDIR /tmp
# RUN wget https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4.tar.gz && \
#   tar -xf cmake-3.15.4.tar.gz

# WORKDIR /tmp/cmake-3.15.4
# RUN ./configure && make -j16 install

ENV CC=clang CXX=clang++ 

VOLUME /sapien-source
WORKDIR /sapien-source

CMD ["tail", "-f", "/dev/null"]
