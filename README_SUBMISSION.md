# AMtown02 VO Leaderboard Submission

## Group Information

| Field | Value |
|-------|-------|
| **Student** | ZHANG Shuyang |
| **Course** | AAE5303 — Robust Control Technology in Low-Altitude Aerial Vehicle |
| **Institution** | The Hong Kong Polytechnic University |
| **Repository** | https://github.com/AfonsoZhang/ORB-SLAM3-DJI-AMtown |

## Results

### Against SfM Ground Truth (sampleinfos_interpolated.json)

| Metric | Value |
|--------|-------|
| **ATE RMSE** | **2.310 m** |
| **RPE Trans Drift** | **0.0103 m/m** |
| **Completeness** | **100%** (6899/6899) |

### Against RTK GPS Ground Truth

| Metric | Value |
|--------|-------|
| **ATE RMSE** | **2.647 m** |
| **RPE Trans Drift** | **2.065 m/m** |
| **Completeness** | **98.7%** (7402/7500) |

## Method Summary

- **SLAM System**: ORB-SLAM3 (Monocular, offline `mono_euroc`)
- **Calibration**: HK_GNSS intrinsics + HKisland distortion (`AMtown_Mono_MARSLVIG.yaml`)
- **Image Resolution**: 1224×1024 (0.5× downsampled)
- **ORB Parameters**: nFeatures=2000, nLevels=8, iniThFAST=15
- **Loop Closure**: Detected ✓
- **Best of 6 runs** (ORB-SLAM3 multi-threading non-determinism)

## Leaderboard JSON

```json
{
  "group_name": "Debuggers",
  "project_private_repo_url": "https://github.com/AfonsoZhang/ORB-SLAM3-DJI-AMtown",
  "metrics": {
    "ate_rmse_m": 2.310,
    "rpe_trans_drift_m_per_m": 0.0103,
    "completeness_pct": 100.0
  }
}
```

## Reproduce

```bash
# 1. Extract images
python3 data/extract_images.py --bag data/AMtown02.bag

# 2. Run ORB-SLAM3 offline
./Examples/Monocular/mono_euroc \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/AMtown_Mono_MARSLVIG.yaml \
  data/AMtown02_offline \
  data/AMtown02_offline/times.txt \
  AMtown02

# 3. Evaluate
awk '{printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2,$3,$4,$5,$6,$7,$8}' \
  f_AMtown02.txt > traj_sec.txt
evo_ape tum data/ground_truth_sfm.txt traj_sec.txt --align --correct_scale --t_max_diff 0.1 -va
```
