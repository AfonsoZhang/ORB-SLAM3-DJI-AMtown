# AAE5303 Assignment 2: Visual Odometry with ORB-SLAM3

**Course:** AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle  
**Department:** Aeronautical and Aviation Engineering, The Hong Kong Polytechnic University  
**Repository:** https://github.com/AfonsoZhang/assignment2

---

## Overview

Monocular Visual Odometry evaluation using ORB-SLAM3 on the **HKisland_GNSS03** UAV aerial imagery dataset from MARS-LVIG. RTK GPS provides centimeter-level ground truth for trajectory accuracy assessment.

## Evaluation Results

| Metric | Value | Baseline | vs Baseline |
|--------|-------|----------|-------------|
| **ATE RMSE** | **1.7203 m** | 88.2281 m | 98% better |
| **RPE Trans Drift** | **1.3803 m/m** | 2.04084 m/m | 32% better |
| **RPE Rot Drift** | **109.52 deg/100m** | 76.70 deg/100m | Higher (trade-off) |
| **Completeness** | **96.32%** | 95.73% | Comparable |
| **Estimated poses** | 3,764 | 2,826 | More coverage |
| **Matched poses** | 1,883 / 1,955 | 1,701 / 1,955 | Better match rate |

### Evaluation Protocol

- Alignment: Sim(3) with scale correction (`--align --correct_scale`)
- Timestamp association: `t_max_diff = 0.1 s`
- RPE distance delta: `delta = 10 m`
- Trajectory file: `CameraTrajectory.txt` (full-frame, TUM format)

## Dataset

| Property | Value |
|----------|-------|
| **Dataset** | HKisland_GNSS03 (MARS-LVIG) |
| **Duration** | ~390 seconds (~6.5 minutes) |
| **Total Images** | ~3,833 frames |
| **Image Resolution** | 2448 x 2048 (downsampled to 1224 x 1024) |
| **Frame Rate** | 10 Hz |
| **Ground Truth** | RTK GPS, 1,955 poses at 5 Hz |

## Modifications and Optimizations

### 1. Image Downsampling (0.5x)

Modified `ros_mono_compressed.cc` to resize images by 0.5x before tracking, reducing computational load and improving real-time performance.

- Original: 2448 x 2048
- Downsampled: 1224 x 1024
- Camera intrinsics scaled accordingly in the YAML config

### 2. Camera Calibration (Official Values)

Used the official calibration from `calib_yaml/HKisland.yaml` (scaled for 0.5x):

```
Camera1.fx: 722.215    Camera1.fy: 722.17
Camera1.cx: 588.90     Camera1.cy: 521.80
Camera1.k1: -0.0530    Camera1.k2: 0.1210
Camera1.p1: 0.00127    Camera1.p2: 0.00043    Camera1.k3: -0.06495
```

### 3. ORB Feature Extraction Tuning

| Parameter | Default | Tuned | Rationale |
|-----------|---------|-------|-----------|
| `nFeatures` | 1500 | 2500 | More features for robust tracking in aerial scenes |
| `scaleFactor` | 1.2 | 1.15 | Finer scale pyramid for better multi-scale matching |
| `nLevels` | 8 | 10 | More pyramid levels for larger scale variation |
| `iniThFAST` | 20 | 12 | Lower threshold to detect features in low-contrast regions |
| `minThFAST` | 7 | 5 | Further lowered for challenging areas (sky, sea) |

### 4. Tracking Robustness (Source Code Modifications)

**`src/Tracking.cc`:**

- Reduced `TrackLocalMap` inlier threshold from 30 to 20 to tolerate fewer matches in sparse scenes without triggering map resets
- Increased `TrackWithMotionModel` search window (`th`) from 15 to 20 for better feature matching under large inter-frame displacement

**`src/System.cc`:**

- Enabled `SaveTrajectoryTUM` for monocular mode (removed the monocular-blocking check) to export full-frame trajectory

### 5. Rosbag Playback Optimization

```bash
rosbag play --pause --rate 0.25 data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
```

- `--rate 0.5`: Half speed to ensure the system can process every frame without dropping

## How to Run

### Prerequisites

- ORB-SLAM3 (compiled with modifications)
- ROS Noetic
- `evo` toolkit (`pip install evo`)

### Step 1: Build

```bash
cd /root/ORB_SLAM3
./build.sh
cd Examples_old/ROS/ORB_SLAM3/build
make -j$(nproc)
```

### Step 2: Run ORB-SLAM3

**Terminal 1 — roscore:**
```bash
source /opt/ros/noetic/setup.bash
roscore
```

**Terminal 2 — ORB-SLAM3:**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml
```

**Terminal 3 — Play bag:**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play --pause --rate 0.5 data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
```

Wait for ORB-SLAM3 to load the vocabulary, then press **Space** to start playback.

### Step 3: Evaluate

```bash
python3 AAE5303_assignment2_orbslam3_demo/scripts/evaluate_vo_accuracy.py \
  --groundtruth ground_truth.txt \
  --estimated CameraTrajectory.txt \
  --t-max-diff 0.1 \
  --delta-m 10 \
  --workdir evaluation_results \
  --json-out evaluation_report.json
```

## File Structure

```
/root/ORB_SLAM3/
├── CameraTrajectory.txt                    # Full-frame trajectory output (TUM format)
├── KeyFrameTrajectory.txt                  # Keyframe-only trajectory
├── ground_truth.txt                        # RTK ground truth (TUM format)
├── evaluation_report.json                  # Evaluation metrics (JSON)
├── evaluation_results/                     # evo intermediate results
├── Examples/Monocular/
│   └── HKisland_Mono.yaml                 # Camera config (tuned)
├── Examples_old/ROS/ORB_SLAM3/src/
│   └── ros_mono_compressed.cc             # ROS node (with downsampling)
├── src/
│   ├── System.cc                          # Modified for monocular trajectory export
│   └── Tracking.cc                        # Modified tracking thresholds
└── calib_yaml/
    └── HKisland.yaml                      # Official camera calibration
```

## References

1. Campos, C., et al. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874-1890.
2. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html
3. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3
