#!/usr/bin/env bash
# shellcheck disable=SC2164  # set -e aborts on failed cd
set -e
echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM3
mkdir -p build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j"$(nproc)"
