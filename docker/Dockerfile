FROM quay.io/pypa/manylinux2014_x86_64:2022-11-14-1226cfc
RUN yum update -y && mkdir /workspace

WORKDIR /workspace
COPY PhysX /workspace/PhysX

ENV CC=/opt/rh/devtoolset-10/root/usr/bin/cc CXX=/opt/rh/devtoolset-10/root/usr/bin/c++

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
RUN git clone --single-branch -b 3.4.0 --depth 1 https://gitlab.com/libeigen/eigen.git
RUN mkdir /workspace/eigen/build
WORKDIR /workspace/eigen/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

WORKDIR /workspace
RUN yum install -y urdfdom-devel boost-devel
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
RUN curl -O https://sdk.lunarg.com/sdk/download/1.3.224.1/linux/vulkan_sdk.tar.gz && tar -xf vulkan_sdk.tar.gz && rm -f vulkan_sdk.tar.gz
ENV VULKAN_SDK=/workspace/1.3.224.1/x86_64 LD_LIBRARY_PATH=/workspace/1.3.224.1/x86_64/lib VK_LAYER_PATH=/workspace/1.3.224.1/x86_64/etc/vulkan/explicit_layer.d PATH=/workspace/1.3.224.1/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

WORKDIR /workspace
RUN git clone --single-branch -b 0.9.9.8 --depth 1 https://github.com/g-truc/glm.git
WORKDIR /workspace/glm
RUN cp -r glm /usr/local/include

RUN yum install -y glew-devel
ENV LD_LIBRARY_PATH=/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/opt/rh/devtoolset-10/root/usr/lib64/dyninst:/opt/rh/devtoolset-10/root/usr/lib/dyninst:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/workspace/1.3.224.1/x86_64/lib PCP_DIR=/opt/rh/devtoolset-10/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-10/root MANPATH=/opt/rh/devtoolset-10/root/usr/share/man: PATH=/opt/rh/devtoolset-10/root/usr/bin:/workspace/1.3.224.1/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

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
RUN git clone --single-branch -b v5.2.3 --depth 1 https://github.com/assimp/assimp.git
RUN sh -c 'printf "@@ -198,3 +198,5 @@\n     mapping.zopen_file = (open_file_func)open;\n+#ifdef ZOPENDISK64\n     mapping.zopendisk_file = (opendisk_file_func)opendisk;\n+#endif\n     mapping.zread_file = (read_file_func)read;" | patch -ulbf /workspace/assimp/code/Common/ZipArchiveIOSystem.cpp'
RUN mkdir /workspace/assimp/build
WORKDIR /workspace/assimp/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DASSIMP_BUILD_TESTS=OFF && make -j4 && make install

RUN rpm -i http://mirror.ghettoforge.org/distributions/gf/el/7/gf/x86_64/gcc10-libstdc++-10.2.1-7.gf.el7.x86_64.rpm
ENV LD_LIBRARY_PATH=/opt/gcc-10.2.1/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/opt/rh/devtoolset-10/root/usr/lib64/dyninst:/opt/rh/devtoolset-10/root/usr/lib/dyninst:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/workspace/1.3.224.1/x86_64/lib:/workspace/cuda/lib64:/workspace/cuda/lib64/stubs/ PCP_DIR=/opt/rh/devtoolset-10/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-10/root MANPATH=/opt/rh/devtoolset-10/root/usr/share/man: PATH=/opt/rh/devtoolset-10/root/usr/bin:/workspace/1.3.224.1/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN /opt/python/cp38-cp38/bin/pip install git+https://github.com/fbxiang/auditwheel.git@088b034ae497b0add794da13f8e0f3b0a3b2d651
RUN echo "#!/opt/python/cp38-cp38/bin/python" > /usr/local/bin/auditwheel \
 && echo "import re, sys" >> /usr/local/bin/auditwheel \
 && echo "from auditwheel.main import main" >> /usr/local/bin/auditwheel \
 && echo "if __name__ == '__main__':" >> /usr/local/bin/auditwheel \
 && echo "    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])" >> /usr/local/bin/auditwheel \
 && echo "    sys.exit(main())" >> /usr/local/bin/auditwheel

WORKDIR /workspace
RUN yum install -y wget
RUN wget https://developer.download.nvidia.com/compute/cuda/11.7.1/local_installers/cuda_11.7.1_515.65.01_linux.run \
 && sh cuda_11.7.1_515.65.01_linux.run --silent --toolkit --toolkitpath=/workspace/cuda --override && rm -f cuda_11.7.1_515.65.01_linux.run
ENV CUDA_PATH=/workspace/cuda
ENV PATH="/workspace/cuda/bin:$PATH"
ENV LD_LIBRARY_PATH="/workspace/cuda/lib64:$LD_LIBRARY_PATH"
