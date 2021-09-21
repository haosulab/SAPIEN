FROM quay.io/pypa/manylinux2014_x86_64:2021-03-12-76cb5c3
RUN yum update -y && mkdir /workspace

WORKDIR /workspace
COPY PhysX /workspace/PhysX

WORKDIR /workspace
RUN git clone --single-branch -b 20210000.6 --depth 1 https://github.com/coin-or/CppAD.git
WORKDIR /workspace/CppAD
RUN mkdir /workspace/CppAD/build
WORKDIR /workspace/CppAD/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN git clone --single-branch -b 0.3.2 --depth 1 https://github.com/ros/console_bridge.git
RUN mkdir /workspace/console_bridge/build
WORKDIR /workspace/console_bridge/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN git clone --single-branch -b 1.0.5 --depth 1 https://github.com/ros/urdfdom_headers.git
RUN mkdir /workspace/urdfdom_headers/build
WORKDIR /workspace/urdfdom_headers/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN yum install -y tinyxml-devel
RUN git clone --single-branch -b 1.0.4 --depth 1 https://github.com/ros/urdfdom.git
RUN mkdir /workspace/urdfdom/build
WORKDIR /workspace/urdfdom/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN yum install -y eigen3-devel urdfdom-devel boost-devel
RUN git clone --single-branch -b v2.5.6 --depth 1 https://github.com/stack-of-tasks/pinocchio.git
WORKDIR /workspace/pinocchio
RUN git submodule update --init --recursive && mkdir /workspace/pinocchio/build
WORKDIR /workspace/pinocchio/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_AUTODIFF_SUPPORT=ON -DBUILD_WITH_URDF_SUPPORT=ON && make -j && make install

WORKDIR /workspace
RUN git clone --single-branch -b v1.8.2 --depth 1 https://github.com/gabime/spdlog.git
RUN mkdir /workspace/spdlog/build
WORKDIR /workspace/spdlog/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DSPDLOG_BUILD_EXAMPLE=OFF && make -j && make install

WORKDIR /workspace
RUN yum install -y libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel
RUN git clone --single-branch -b 3.3.3 --depth 1 https://github.com/glfw/glfw.git
RUN mkdir /workspace/glfw/build
WORKDIR /workspace/glfw/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DGLFW_BUILD_DOCS=OFF -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_VULKAN_STATIC=OFF -DBUILD_SHARED_LIBS=ON && make -j && make install

WORKDIR /workspace
RUN curl -O https://sdk.lunarg.com/sdk/download/1.2.189.0/linux/vulkan_sdk.tar.gz && tar -xf vulkan_sdk.tar.gz && rm -f vulkan_sdk.tar.gz
ENV VULKAN_SDK=/workspace/1.2.189.0/x86_64 LD_LIBRARY_PATH=/workspace/1.2.189.0/x86_64/lib VK_LAYER_PATH=/workspace/1.2.189.0/x86_64/etc/vulkan/explicit_layer.d PATH=/workspace/1.2.189.0/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

WORKDIR /workspace
RUN git clone --single-branch -b 0.9.9.8 --depth 1 https://github.com/g-truc/glm.git
WORKDIR /workspace/glm
RUN cp -r glm /usr/local/include

RUN yum install -y glew-devel
ENV LD_LIBRARY_PATH=/opt/rh/devtoolset-9/root/usr/lib64:/opt/rh/devtoolset-9/root/usr/lib:/opt/rh/devtoolset-9/root/usr/lib64/dyninst:/opt/rh/devtoolset-9/root/usr/lib/dyninst:/opt/rh/devtoolset-9/root/usr/lib64:/opt/rh/devtoolset-9/root/usr/lib:/workspace/1.2.189.0/x86_64/lib PCP_DIR=/opt/rh/devtoolset-9/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-9/root MANPATH=/opt/rh/devtoolset-9/root/usr/share/man: PATH=/opt/rh/devtoolset-9/root/usr/bin:/workspace/1.2.189.0/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

WORKDIR /workspace
RUN git clone --single-branch -b v2.1.0 --depth 1 https://github.com/yse/easy_profiler.git
RUN mkdir /workspace/easy_profiler/build
WORKDIR /workspace/easy_profiler/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

RUN yum install -y zip

WORKDIR /workspace
RUN git clone --single-branch -b v4.0.0 --depth 1 https://github.com/KhronosGroup/KTX-Software.git
RUN mkdir /workspace/KTX-Software/build
WORKDIR /workspace/KTX-Software/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN git clone --single-branch -b release-2.0.16 --depth 1 https://github.com/libsdl-org/SDL.git
RUN mkdir /workspace/SDL/build
WORKDIR /workspace/SDL/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN git clone --single-branch -b v5.0.1 --depth 1 https://github.com/assimp/assimp.git
RUN awk '{sub("-g ","")}1' /workspace/assimp/CMakeLists.txt > /tmp/CMakeLists.txt && mv /tmp/CMakeLists.txt /workspace/assimp/CMakeLists.txt
RUN mkdir /workspace/assimp/build
WORKDIR /workspace/assimp/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DASSIMP_BUILD_TESTS=OFF && make -j && make install

RUN rpm -i http://mirror.ghettoforge.org/distributions/gf/el/7/gf/x86_64/gcc10-libstdc++-10.2.1-7.gf.el7.x86_64.rpm
ENV LD_LIBRARY_PATH=/opt/gcc-10.2.1/usr/lib64:/opt/rh/devtoolset-9/root/usr/lib64:/opt/rh/devtoolset-9/root/usr/lib:/opt/rh/devtoolset-9/root/usr/lib64/dyninst:/opt/rh/devtoolset-9/root/usr/lib/dyninst:/opt/rh/devtoolset-9/root/usr/lib64:/opt/rh/devtoolset-9/root/usr/lib:/workspace/1.2.189.0/x86_64/lib:/workspace/cuda/lib64:/workspace/cuda/lib64/stubs/ PCP_DIR=/opt/rh/devtoolset-9/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-9/root MANPATH=/opt/rh/devtoolset-9/root/usr/share/man: PATH=/opt/rh/devtoolset-9/root/usr/bin:/workspace/1.2.189.0/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN /opt/python/cp38-cp38/bin/pip install git+https://github.com/fbxiang/auditwheel.git@3a723fa01f87b04b5ce4a8e7f81f6a142ac3044f
RUN echo "#!/opt/python/cp38-cp38/bin/python" > /usr/local/bin/auditwheel \
 && echo "import re, sys" >> /usr/local/bin/auditwheel \
 && echo "from auditwheel.main import main" >> /usr/local/bin/auditwheel \
 && echo "if __name__ == '__main__':" >> /usr/local/bin/auditwheel \
 && echo "    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])" >> /usr/local/bin/auditwheel \
 && echo "    sys.exit(main())" >> /usr/local/bin/auditwheel

WORKDIR /workspace
COPY cuda-11.4 /workspace/cuda
ENV CUDA_PATH=/workspace/cuda

WORKDIR /workspace
