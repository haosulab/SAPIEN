FROM quay.io/pypa/manylinux2014_x86_64:2022-11-14-1226cfc
RUN yum update -y && mkdir /workspace

ENV CC=/opt/rh/devtoolset-10/root/usr/bin/cc CXX=/opt/rh/devtoolset-10/root/usr/bin/c++
ENV LD_LIBRARY_PATH=/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/opt/rh/devtoolset-10/root/usr/lib64/dyninst:/opt/rh/devtoolset-10/root/usr/lib/dyninst:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib PCP_DIR=/opt/rh/devtoolset-10/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-10/root MANPATH=/opt/rh/devtoolset-10/root/usr/share/man: PATH=/opt/rh/devtoolset-10/root/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# tinyxml, boost, x
RUN yum install -y tinyxml-devel boost169-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel zip wget
RUN ln -s /usr/lib64/boost169/* /usr/lib64 && ln -s /usr/include/boost169/boost/ /usr/include/boost

# custom auditwheel
RUN /opt/python/cp38-cp38/bin/pip install git+https://github.com/fbxiang/auditwheel.git@088b034ae497b0add794da13f8e0f3b0a3b2d651
RUN echo "#!/opt/python/cp38-cp38/bin/python" > /usr/local/bin/auditwheel && \
    echo "import re, sys" >> /usr/local/bin/auditwheel && \
    echo "from auditwheel.main import main" >> /usr/local/bin/auditwheel && \
    echo "if __name__ == '__main__':" >> /usr/local/bin/auditwheel && \
    echo "    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])" >> /usr/local/bin/auditwheel && \
    echo "    sys.exit(main())" >> /usr/local/bin/auditwheel

# CUDA Toolkit
RUN cd /workspace && wget https://developer.download.nvidia.com/compute/cuda/11.7.1/local_installers/cuda_11.7.1_515.65.01_linux.run && \
    sh cuda_11.7.1_515.65.01_linux.run --silent --toolkit --toolkitpath=/workspace/cuda --override && rm -f cuda_11.7.1_515.65.01_linux.run
ENV CUDA_PATH=/workspace/cuda PATH="/workspace/cuda/bin:$PATH" LD_LIBRARY_PATH="/workspace/cuda/lib64:$LD_LIBRARY_PATH"

