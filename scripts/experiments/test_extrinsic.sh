#!/bin/bash
# Usage: ./test_extrinsic.sh <variant: A1|A2|A3|A4>
# Modifies AMtown_MonoIMU.yaml with the selected T_b_c1 rotation variant.
# DJI IMU body: FLU (x=Forward, y=Left, z=Up)
# Camera: OpenCV (x=Right, y=Down, z=IntoScene), looking downward
# All variants share: cam_z → -body_z (3rd column = [0,0,-1])

YAML="Examples/Monocular-Inertial/AMtown_MonoIMU.yaml"
VARIANT=${1:-A1}

case $VARIANT in
  A1)
    DESC="cam top → drone FORWARD"
    DATA="0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0"
    ;;
  A2)
    DESC="cam top → drone BACKWARD"
    DATA="0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0"
    ;;
  A3)
    DESC="cam top → drone LEFT"
    DATA="1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0"
    ;;
  A4)
    DESC="cam top → drone RIGHT"
    DATA="-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0"
    ;;
  *)
    echo "Unknown variant: $VARIANT (use A1, A2, A3, or A4)"
    exit 1
    ;;
esac

python3 -c "
import re
with open('$YAML', 'r') as f:
    content = f.read()
content = re.sub(
    r'(data: \[)[^\]]+(\])',
    r'\g<1> $DATA\2',
    content
)
with open('$YAML', 'w') as f:
    f.write(content)
"

echo "=== Set T_b_c1 to variant $VARIANT: $DESC ==="
echo "Now run:"
echo "  Terminal 1: ./Examples_old/ROS/ORB_SLAM3/Mono_Inertial_Compressed Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/AMtown_MonoIMU.yaml"
echo "  Terminal 2: rosbag play --rate 0.3 --duration 120 data/AMtown02.bag /left_camera/image/compressed:=/camera/image_raw/compressed /dji_osdk_ros/imu:=/imu"
