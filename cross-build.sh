#!/bin/bash

set -ex

apt-get update
apt-get install --no-install-recommends -y cmake ninja-build
rm -rf cmake_build
cmake -B cmake_build -DCMAKE_BUILD_TYPE=Release -G Ninja --toolchain /usr/local/toolchain-config.cmake -DOPENCV_ARCH=linuxarm64
cmake --build cmake_build
