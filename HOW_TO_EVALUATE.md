# ORB_SLAM3 Trajectory Evaluation Guide

## Quick Start

### Step 1: Extract RTK Ground Truth from ROS Bag

```python
python3 << 'EOF'
import rosbag
import numpy as np

bag = rosbag.Bag('data/HKisland_GNSS03.bag')
rtk_data = []

for topic, msg, t in bag.read_messages(topics=['/dji_osdk_ros/rtk_position']):
    timestamp = msg.header.stamp.to_sec()
    lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
    rtk_data.append([timestamp, lat, lon, alt])

bag.close()
rtk_data = np.array(rtk_data)

# Convert to local ENU coordinates
lat0, lon0, alt0 = rtk_data[0, 1], rtk_data[0, 2], rtk_data[0, 3]
R = 6378137.0

x = R * np.radians(rtk_data[:, 2] - lon0) * np.cos(np.radians(lat0))
y = R * np.radians(rtk_data[:, 1] - lat0)
z = rtk_data[:, 3] - alt0

# Save in TUM format
with open('ground_truth.txt', 'w') as f:
    for i in range(len(rtk_data)):
        f.write(f"{rtk_data[i,0]:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f} 0 0 0 1\n")

print(f"Saved {len(rtk_data)} ground truth poses")
EOF
```

### Step 2: Evaluate with evo Tool

**Install evo (if not already installed):**
```bash
pip3 install evo --upgrade
```

**Evaluate ATE (Absolute Trajectory Error):**
```bash
evo_ape tum ground_truth.txt KeyFrameTrajectory.txt -va --align --correct_scale
```

**Evaluate RPE (Relative Pose Error):**
```bash
evo_rpe tum ground_truth.txt KeyFrameTrajectory.txt -va --align --correct_scale
```

**Generate plots:**
```bash
evo_ape tum ground_truth.txt KeyFrameTrajectory.txt -va --align --correct_scale --plot --plot_mode xyz
```

## File Structure

```
/root/ORB_SLAM3/
├── KeyFrameTrajectory.txt          # ORB_SLAM3 output trajectory (TUM format)
├── ground_truth.txt                 # RTK ground truth (TUM format)
├── FINAL_EVALUATION_RESULTS.txt    # Evaluation summary
└── HOW_TO_EVALUATE.md              # This guide
```

## TUM Format Specification

Each line: `timestamp tx ty tz qx qy qz qw`
- timestamp: seconds
- tx, ty, tz: position in meters
- qx, qy, qz, qw: orientation quaternion

## Understanding Metrics

### ATE (Absolute Trajectory Error)
- Measures global consistency
- Lower is better
- Good: < 3m for outdoor scenarios

### RPE (Relative Pose Error)  
- Measures local accuracy and drift
- Lower is better
- Good: < 5m for 1-frame delta

### Scale Error
- Percentage difference from true scale
- Only relevant for monocular SLAM
- Good: < 10%

## Tips

1. **Low alignment rate?** Check if tracking was lost frequently
2. **High RPE?** Indicates drift accumulation or tracking loss
3. **High scale error?** Check initialization quality or add IMU
4. **Want better results?** Increase ORB features or lower FAST threshold

