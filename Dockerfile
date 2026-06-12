# Reproducible build environment for ORB-SLAM3-DJI-AMtown
# Ubuntu 20.04 + ROS Noetic + OpenCV 4.2 (system) + Pangolin v0.6
#
#   docker build -t orbslam3-dji-amtown .
#   docker run -it --rm -v /path/to/datasets:/work/ORB-SLAM3-DJI-AMtown/data orbslam3-dji-amtown
#
# The offline pipeline (scripts/run_amtown.sh + scripts/evaluate.sh) is fully
# headless; no X server is required.

FROM ros:noetic-perception-focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget unzip \
        libeigen3-dev libboost-serialization-dev libssl-dev \
        libgl1-mesa-dev libglew-dev \
        ros-noetic-tf ros-noetic-rosbuild ros-noetic-rosbash ros-noetic-mk \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Pangolin v0.6 (known-good with ORB-SLAM3 on focal)
RUN git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin \
    && cmake -S /tmp/Pangolin -B /tmp/Pangolin/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/Pangolin/build -j"$(nproc)" --target install \
    && ldconfig \
    && rm -rf /tmp/Pangolin

RUN pip3 install --no-cache-dir evo numpy matplotlib

WORKDIR /work/ORB-SLAM3-DJI-AMtown
COPY . .

# Core library + offline examples (uncompresses the ORB vocabulary)
RUN ./build.sh

# ROS nodes (online mode); rosbuild needs the package on ROS_PACKAGE_PATH
RUN bash -c "source /opt/ros/noetic/setup.bash \
    && export ROS_PACKAGE_PATH=/work/ORB-SLAM3-DJI-AMtown/Examples/ROS:\$ROS_PACKAGE_PATH \
    && ./build_ros.sh"

# Make the ROS package visible in interactive shells too
RUN echo "export ROS_PACKAGE_PATH=/work/ORB-SLAM3-DJI-AMtown/Examples/ROS:\$ROS_PACKAGE_PATH" >> /root/.bashrc

CMD ["bash"]
