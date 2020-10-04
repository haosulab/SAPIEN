#!/usr/bin/env bash
PACKAGE_VERSION=$(<./python/VERSION)
python3 setup_ros2.py bdist_wheel
cd dist && rm -rf wheelhouse
auditwheel repair sapien_robot-*.whl --plat linux_x86_64
cd wheelhouse && unzip sapien_robot-*.whl
cp sapien_robot.libs/librmw_fastrtps_cpp-*.so sapien_robot.libs/librmw_fastrtps_cpp.so
rm "sapien_robot-${PACKAGE_VERSION}-cp36-cp36m-linux_x86_64.whl"
zip -r "sapien_robot-${PACKAGE_VERSION}-cp36-cp36m-linux_x86_64.whl" sapien "sapien_robot-${PACKAGE_VERSION}.dist-info" sapien_robot.libs

