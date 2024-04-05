FROM ubuntu:22.04
RUN apt update -y && apt upgrade -y
RUN apt install -y g++ git cmake python3 python3-pip
RUN python3 -m pip install -U pip

WORKDIR /root
RUN git clone --depth 1 --recursive --shallow-submodules https://github.com/google/swiftshader.git \
    && cd swiftshader \
    && cd build \
    && cmake .. \
    && cmake --build . --parallel 8 --target vk_swiftshader \
    && mkdir -p /usr/share/vulkan/icd.d \
    && cp Linux/vk_swiftshader_icd.json /usr/share/vulkan/icd.d \
    && cp Linux/libvk_swiftshader.so /usr/share/vulkan/icd.d \
    && cd /root \
    && rm -rf swiftshader
