from ubuntu:18.04

run apt update && apt upgrade -y && \
  apt install -y python3-pip libx11-6 libassimp4 libxcb1 libglew2.0 xvfb && \
  python3 -m pip install http://sapien.ucsd.edu/api/data/sapien-0.2.0.dev0-cp36-cp36m-linux_x86_64.whl gym torch tensorboardx
