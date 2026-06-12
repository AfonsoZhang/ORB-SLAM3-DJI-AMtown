#!/bin/bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3

YAML="Examples/Monocular-Inertial/AMtown_MonoIMU.yaml"
SLAM="./Examples_old/ROS/ORB_SLAM3/Mono_Inertial_Compressed"
VOCAB="Vocabulary/ORBvoc.txt"
BAG="data/AMtown02.bag"

declare -A ROTATIONS
ROTATIONS["A"]="0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.3, 0.0, 0.0, 0.0, 1.0"
ROTATIONS["B"]="0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.3, 0.0, 0.0, 0.0, 1.0"
ROTATIONS["C"]="1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.3, 0.0, 0.0, 0.0, 1.0"
ROTATIONS["D"]="-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.3, 0.0, 0.0, 0.0, 1.0"

for OPT in A B C D; do
    echo ""
    echo "============================================"
    echo "Testing Rotation $OPT"
    echo "============================================"

    DATA="${ROTATIONS[$OPT]}"
    python3 -c "
import re
with open('$YAML','r') as f:
    content = f.read()
old = re.search(r'(   data: \[)[^\]]+(\])', content)
if old:
    new_data = '   data: [$DATA]'
    content = content[:old.start()] + new_data + content[old.end():]
with open('$YAML','w') as f:
    f.write(content)
print('Config updated for option $OPT')
"

    rosnode cleanup <<< "y" 2>/dev/null
    sleep 1

    $SLAM $VOCAB $YAML &>/tmp/slam_output_${OPT}.txt &
    SLAM_PID=$!
    sleep 8

    rosbag play $BAG \
        /left_camera/image/compressed:=/camera/image_raw/compressed \
        /dji_osdk_ros/imu:=/imu \
        --duration=60 &>/dev/null &
    BAG_PID=$!

    sleep 45

    kill $BAG_PID 2>/dev/null
    kill $SLAM_PID 2>/dev/null
    sleep 2
    kill -9 $SLAM_PID 2>/dev/null
    kill -9 $BAG_PID 2>/dev/null
    sleep 1

    echo "--- Output for Rotation $OPT ---"
    grep -E "VIBA|Fail to track|Reseting|New Map|Not enough|Less than|scale too" /tmp/slam_output_${OPT}.txt | head -20
    echo "---"
done

echo ""
echo "ALL TESTS COMPLETE"
