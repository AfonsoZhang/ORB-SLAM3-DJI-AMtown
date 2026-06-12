# TUM-VI Mono-Inertial VIO Run Guide

## Dataset Information

| Property | Value |
|----------|-------|
| **Dataset** | TUM-VI `dataset-room1_512_16` |
| **Source** | [TUM Visual-Inertial Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) |
| **Camera** | 512×512, 20 Hz, global shutter |
| **IMU** | BMI160, 200 Hz |
| **Duration** | ~140 seconds |
| **Ground Truth** | Motion capture (OptiTrack), ~120 Hz |
| **Format** | ASL / EuRoC format |

## Download Dataset

```bash
mkdir -p data/TUM-VI
cd data/TUM-VI

# Download room1 sequence (1.6 GB)
wget https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room1_512_16.tar

# Extract
tar xf dataset-room1_512_16.tar
```

## Prepare Timestamps

The ORB-SLAM3 `mono_inertial_tum_vi` executable expects a plain timestamps file (one timestamp per line), not the ASL CSV format:

```bash
awk -F',' 'NR>1 {print $1}' \
  data/TUM-VI/dataset-room1_512_16/mav0/cam0/data.csv \
  > data/TUM-VI/dataset-room1_512_16/mav0/cam0/times.txt
```

## Run VIO

```bash
cd /root/ORB_SLAM3

./Examples/Monocular-Inertial/mono_inertial_tum_vi \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular-Inertial/TUM-VI.yaml \
  data/TUM-VI/dataset-room1_512_16/mav0/cam0/data \
  data/TUM-VI/dataset-room1_512_16/mav0/cam0/times.txt \
  data/TUM-VI/dataset-room1_512_16/mav0/imu0/data.csv \
  dataset-room1_512_16
```

Output files:
- `f_dataset-room1_512_16.txt` — Full trajectory (all tracked frames)
- `kf_dataset-room1_512_16.txt` — Keyframe trajectory

## Prepare Ground Truth for Evaluation

Convert mocap CSV (nanosecond timestamps, `qw qx qy qz` order) to TUM format (seconds, `qx qy qz qw` order):

```python
python3 -c "
import csv
with open('data/TUM-VI/dataset-room1_512_16/mav0/mocap0/data.csv') as f, \
     open('data/TUM-VI/room1_groundtruth.txt', 'w') as out:
    for row in csv.reader(f):
        if row[0].startswith('#'): continue
        ts = int(row[0]) / 1e9
        out.write(f'{ts:.9f} {row[1]} {row[2]} {row[3]} {row[5]} {row[6]} {row[7]} {row[4]}\n')
"
```

Convert estimated trajectory timestamps from nanoseconds to seconds:

```python
python3 -c "
with open('f_dataset-room1_512_16.txt') as f, \
     open('data/TUM-VI/room1_estimated.txt', 'w') as out:
    for line in f:
        p = line.split()
        if len(p) == 8:
            out.write(f'{float(p[0])/1e9:.9f} {\" \".join(p[1:])}\n')
"
```

## Evaluate with evo

### ATE (Absolute Trajectory Error)

```bash
evo_ape tum data/TUM-VI/room1_groundtruth.txt data/TUM-VI/room1_estimated.txt \
  --align --correct_scale -v
```

### ATE without scale correction (VIO true scale)

```bash
evo_ape tum data/TUM-VI/room1_groundtruth.txt data/TUM-VI/room1_estimated.txt \
  --align -v
```

### RPE (Relative Pose Error)

```bash
evo_rpe tum data/TUM-VI/room1_groundtruth.txt data/TUM-VI/room1_estimated.txt \
  --align --correct_scale -v
```

### Generate plots

```bash
mkdir -p data/TUM-VI/figures

evo_traj tum data/TUM-VI/room1_groundtruth.txt data/TUM-VI/room1_estimated.txt \
  --align --correct_scale --ref data/TUM-VI/room1_groundtruth.txt \
  --plot_mode xyz --save_plot data/TUM-VI/figures/trajectory_comparison

evo_ape tum data/TUM-VI/room1_groundtruth.txt data/TUM-VI/room1_estimated.txt \
  --align --correct_scale --plot --save_plot data/TUM-VI/figures/ate_map

evo_rpe tum data/TUM-VI/room1_groundtruth.txt data/TUM-VI/room1_estimated.txt \
  --align --correct_scale --plot --save_plot data/TUM-VI/figures/rpe_map
```

## Expected Results

| Metric | Value |
|--------|-------|
| ATE RMSE | 0.011 m (1.1 cm) |
| ATE Mean | 0.0075 m |
| RPE RMSE | 0.0097 m/frame |
| RPE Median | 0.0016 m (1.6 mm) |
| Scale Factor | 0.9986 |
| Tracking Rate | 97.9% |

## Comparison: VIO vs Pure Mono

| Aspect | Mono VO (AMtown02) | VIO (TUM-VI) |
|--------|-------------------|---------------|
| ATE RMSE | 106.3 m | 0.011 m |
| Scale | Arbitrary (needs correction) | Metric (true scale) |
| Completeness | 98.6% | 97.9% |
| IMU Fusion | No | Yes |
| Scale Correction Needed | Yes (`--correct_scale`) | No |

The key advantage of VIO: **metric scale recovery** without external reference.
