FROM quay.io/pypa/manylinux2014_x86_64:2022-11-14-1226cfc
RUN yum update -y && mkdir /workspace

ENV CC=/opt/rh/devtoolset-10/root/usr/bin/cc CXX=/opt/rh/devtoolset-10/root/usr/bin/c++

RUN yum install -y tinyxml-devel boost169-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel glew-devel zip wget
RUN ln -s /usr/lib64/boost169/* /usr/lib64 && ln -s /usr/include/boost169/boost/ /usr/include/boost

RUN cd /workspace && git clone --single-branch -b 20210000.6 --depth 1 https://github.com/coin-or/CppAD.git && \
    cd /workspace/CppAD && mkdir /workspace/CppAD/build && \
    cd /workspace/CppAD/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf CppAD

RUN cd /workspace && git clone --single-branch -b 0.3.2 --depth 1 https://github.com/ros/console_bridge.git && \
    mkdir /workspace/console_bridge/build && cd /workspace/console_bridge/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf console_bridge

RUN cd /workspace && git clone --single-branch -b 1.0.5 --depth 1 https://github.com/ros/urdfdom_headers.git && \
    mkdir /workspace/urdfdom_headers/build && cd /workspace/urdfdom_headers/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf urdfdom_headers

RUN cd /workspace && git clone --single-branch -b 1.0.4 --depth 1 https://github.com/ros/urdfdom.git && \
    mkdir /workspace/urdfdom/build && cd /workspace/urdfdom/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf urdfdom

RUN cd /workspace && git clone --single-branch -b 3.4.0 --depth 1 https://gitlab.com/libeigen/eigen.git && \
    mkdir /workspace/eigen/build && cd /workspace/eigen/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf eigen

RUN cd /workspace && git clone --single-branch -b v2.5.6 --depth 1 https://github.com/stack-of-tasks/pinocchio.git && \
    cd /workspace/pinocchio && git submodule update --init --recursive && mkdir /workspace/pinocchio/build && \
    cd /workspace/pinocchio/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_AUTODIFF_SUPPORT=ON -DBUILD_WITH_URDF_SUPPORT=ON && make -j && make install && \
    cd /workspace && rm -rf pinocchio

RUN cd /workspace && git clone --single-branch -b v1.8.2 --depth 1 https://github.com/gabime/spdlog.git && \
    mkdir /workspace/spdlog/build && cd /workspace/spdlog/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DSPDLOG_BUILD_EXAMPLE=OFF && make -j && make install && \
    cd /workspace && rm -rf spdlog

RUN cd /workspace && git clone --single-branch -b 3.3.3 --depth 1 https://github.com/glfw/glfw.git && \
    mkdir /workspace/glfw/build && cd /workspace/glfw/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DGLFW_BUILD_DOCS=OFF -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_VULKAN_STATIC=OFF -DBUILD_SHARED_LIBS=ON && make -j && make install && \
    cd /workspace && rm -rf glfw

RUN cd /workspace && git clone --single-branch -b 0.9.9.8 --depth 1 https://github.com/g-truc/glm.git && \
    cd /workspace/glm && cp -r glm /usr/local/include && \
    cd /workspace && rm -rf glm

ENV LD_LIBRARY_PATH=/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/opt/rh/devtoolset-10/root/usr/lib64/dyninst:/opt/rh/devtoolset-10/root/usr/lib/dyninst:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/workspace/1.3.224.1/x86_64/lib PCP_DIR=/opt/rh/devtoolset-10/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-10/root MANPATH=/opt/rh/devtoolset-10/root/usr/share/man: PATH=/opt/rh/devtoolset-10/root/usr/bin:/workspace/1.3.224.1/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN cd /workspace && \
    git clone --single-branch -b v2.1.0 --depth 1 https://github.com/yse/easy_profiler.git && \
    mkdir /workspace/easy_profiler/build && \
    cd /workspace/easy_profiler/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf easy_profiler

RUN cd /workspace && git clone --single-branch -b v4.0.0 --depth 1 https://github.com/KhronosGroup/KTX-Software.git && \
    mkdir /workspace/KTX-Software/build && cd /workspace/KTX-Software/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf KTX-Software

RUN cd /workspace && git clone --single-branch -b release-2.0.16 --depth 1 https://github.com/libsdl-org/SDL.git && \
    mkdir /workspace/SDL/build && cd /workspace/SDL/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    cd /workspace && rm -rf SDL

RUN cd /workspace && git clone --single-branch -b v5.2.3 --depth 1 https://github.com/assimp/assimp.git && \
    sh -c 'printf "@@ -198,3 +198,5 @@\n     mapping.zopen_file = (open_file_func)open;\n+#ifdef ZOPENDISK64\n     mapping.zopendisk_file = (opendisk_file_func)opendisk;\n+#endif\n     mapping.zread_file = (read_file_func)read;" | patch -ulbf /workspace/assimp/code/Common/ZipArchiveIOSystem.cpp' && \
    mkdir /workspace/assimp/build && cd /workspace/assimp/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DASSIMP_BUILD_TESTS=OFF && make -j4 && make install && \
    cd /workspace && rm -rf assimp

RUN cd /workspace && git clone --single-branch -b v1.51.1 --depth 1 https://github.com/grpc/grpc.git && \
    cd /workspace/grpc && git submodule update --init --recursive && mkdir -p build && \
    cd /workspace/grpc/build && cmake .. -DCMAKE_INSTALL_PREFIX=/workspace/grpc_install/install && make -j8 && make install && \
    cd /workspace && rm -rf grpc && mv grpc_install grpc

RUN cd /workspace && wget https://storage1.ucsd.edu/datasets/PhysX5_compiled.zip && unzip PhysX5_compiled.zip && mv PhysX5_compiled PhysX && rm -f PhysX5_compiled.zip

RUN cd /workspace && curl -O https://sdk.lunarg.com/sdk/download/1.3.224.1/linux/vulkan_sdk.tar.gz && tar -xf vulkan_sdk.tar.gz && rm -f vulkan_sdk.tar.gz
ENV VULKAN_SDK=/workspace/1.3.224.1/x86_64 LD_LIBRARY_PATH=/workspace/1.3.224.1/x86_64/lib VK_LAYER_PATH=/workspace/1.3.224.1/x86_64/etc/vulkan/explicit_layer.d PATH=/workspace/1.3.224.1/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN rpm -i http://mirror.ghettoforge.org/distributions/gf/el/7/gf/x86_64/gcc10-libstdc++-10.2.1-7.gf.el7.x86_64.rpm
ENV LD_LIBRARY_PATH=/opt/gcc-10.2.1/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/opt/rh/devtoolset-10/root/usr/lib64/dyninst:/opt/rh/devtoolset-10/root/usr/lib/dyninst:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/workspace/1.3.224.1/x86_64/lib:/workspace/cuda/lib64:/workspace/cuda/lib64/stubs/ PCP_DIR=/opt/rh/devtoolset-10/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-10/root MANPATH=/opt/rh/devtoolset-10/root/usr/share/man: PATH=/opt/rh/devtoolset-10/root/usr/bin:/workspace/1.3.224.1/x86_64/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN /opt/python/cp38-cp38/bin/pip install git+https://github.com/fbxiang/auditwheel.git@088b034ae497b0add794da13f8e0f3b0a3b2d651
RUN echo "#!/opt/python/cp38-cp38/bin/python" > /usr/local/bin/auditwheel && \
    echo "import re, sys" >> /usr/local/bin/auditwheel && \
    echo "from auditwheel.main import main" >> /usr/local/bin/auditwheel && \
    echo "if __name__ == '__main__':" >> /usr/local/bin/auditwheel && \
    echo "    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])" >> /usr/local/bin/auditwheel && \
    echo "    sys.exit(main())" >> /usr/local/bin/auditwheel

RUN cd /workspace && wget https://developer.download.nvidia.com/compute/cuda/11.7.1/local_installers/cuda_11.7.1_515.65.01_linux.run && \
    sh cuda_11.7.1_515.65.01_linux.run --silent --toolkit --toolkitpath=/workspace/cuda --override && rm -f cuda_11.7.1_515.65.01_linux.run
ENV CUDA_PATH=/workspace/cuda PATH="/workspace/cuda/bin:$PATH" LD_LIBRARY_PATH="/workspace/cuda/lib64:$LD_LIBRARY_PATH"

RUN cd /workspace && git clone --single-branch -b v2021.8.0 --depth 1 https://github.com/oneapi-src/oneTBB.git && \
    mkdir /workspace/oneTBB/build && cd /workspace/oneTBB/build && \
    cmake .. -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-DTBB_ALLOCATOR_TRAITS_BROKEN && make -j && make install && \
    cd /workspace && rm -rf oneTBB

RUN cd /workspace && git clone --single-branch -b v1.5.0 --depth 1 https://github.com/Blosc/c-blosc.git && \
    mkdir /workspace/c-blosc/build && cd /workspace/c-blosc/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED=OFF -DBUILD_TESTS=OFF && make -j && make install && \
    cd /workspace && rm -rf c-blosc

RUN yum install -y zlib-static

RUN cd /workspace && git clone --single-branch -b v8.2.0 --depth 1 https://github.com/AcademySoftwareFoundation/openvdb.git && \
    mkdir /workspace/openvdb/build && cd /workspace/openvdb/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DTBB_USE_STATIC_LIBS=ON -DBLOSC_USE_STATIC_LIBS=ON && make -j && make install && \
    cd /workspace && rm -rf openvdb
