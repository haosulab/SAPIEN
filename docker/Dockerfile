FROM quay.io/pypa/manylinux_2_28_x86_64:latest

RUN yum install -y libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel zip wget openssh-clients git-lfs gcc-toolset-11

RUN mkdir -p /workspace && cd /workspace && wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run  && \
    sh cuda_11.8.0_520.61.05_linux.run --silent --toolkit --toolkitpath=/workspace/cuda --override && rm -f cuda_11.8.0_520.61.05_linux.run
ENV CUDA_PATH=/workspace/cuda PATH="/workspace/cuda/bin:$PATH" LD_LIBRARY_PATH="/workspace/cuda/lib64:$LD_LIBRARY_PATH"

RUN pipx install cmake==3.26.3 --force

RUN /opt/python/cp313-cp313/bin/python -m pip install setuptools wheel
RUN /opt/python/cp312-cp312/bin/python -m pip install setuptools wheel

# custom auditwheel
RUN /opt/python/cp313-cp313/bin/pip install git+https://github.com/fbxiang/auditwheel.git@fe61bcdfb78f3955d9a3b6fbb05e3852c32239b0
RUN echo "#!/opt/python/cp313-cp313/bin/python" > /usr/local/bin/auditwheel && \
    echo "import re, sys" >> /usr/local/bin/auditwheel && \
    echo "from auditwheel.main import main" >> /usr/local/bin/auditwheel && \
    echo "if __name__ == '__main__':" >> /usr/local/bin/auditwheel && \
    echo "    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])" >> /usr/local/bin/auditwheel && \
    echo "    sys.exit(main())" >> /usr/local/bin/auditwheel


ENV CC=/opt/rh/gcc-toolset-11/root/usr/bin/cc CXX=/opt/rh/gcc-toolset-11/root/usr/bin/c++
ENV LD_LIBRARY_PATH=/opt/rh/gcc-toolset-11/root/usr/lib64:/opt/rh/gcc-toolset-11/root/usr/lib:/opt/rh/gcc-toolset-11/root/usr/lib64/dyninst:/opt/rh/gcc-toolset-11/root/usr/lib/dyninst PCP_DIR=/opt/rh/gcc-toolset-11/root/ DEVTOOLSET_ROOTPATH=/opt/rh/gcc-toolset-11/root MANPATH=/opt/rh/gcc-toolset-11/root/usr/share/man PATH=/opt/rh/gcc-toolset-11/root/usr/bin:/usr/share/Modules/bin:/opt/rh/gcc-toolset-11/root/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

ENV PATH=/opt/python/cp313-cp313/bin:$PATH
