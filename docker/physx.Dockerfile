FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive
RUN mkdir /workspace
RUN apt update && apt install -y git cmake curl libstdc++6 clang-8 g++-9 libx11-dev

WORKDIR /workspace
RUN git clone https://github.com/NVIDIA-Omniverse/PhysX.git
