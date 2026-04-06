# AAE5303 - Leaderboard Submission Guide

## 📁 Evaluation Dataset

The evaluation dataset (bag / images / ground truth) is provided by the instructor.

The leaderboard expects an estimated trajectory in **TUM format** and computes metrics against the provided **ground truth**.

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset (AMtown02) | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

All sequences used in this course are sourced from the **official MARS-LVIG AMtown02** dataset.

---

## 📊 Evaluation Metrics

| Metric | Direction | Unit | Description |
|--------|-----------|------|-------------|
| **ATE RMSE** | ↓ Lower is better | meters (m) | Global accuracy after Sim(3) alignment + scale correction |
| **RPE Trans Drift** | ↓ Lower is better | meters per meter (m/m) | Translation drift rate (distance-based RPE) |
| **RPE Rot Drift** | ↓ Lower is better | degrees per 100 meters (deg/100m) | Rotation drift rate (distance-based RPE) |
| **Completeness** | ↑ Higher is better | percent (%) | Matched poses / total ground-truth poses |

### Fixed Evaluation Parameters

To make submissions comparable, the leaderboard uses the following fixed parameters:

- **Trajectory format**: TUM (`t tx ty tz qx qy qz qw`)
- **Timestamp association**: `t_max_diff = 0.1 s`
- **Alignment**: Sim(3) with scale correction (`--align --correct_scale`)
- **RPE delta**: `delta = 10 m` (distance domain)

---

## 📄 JSON Submission Format

Submit your results using the following JSON format:

```json
{
  "group_name": "Team Alpha",
  "project_private_repo_url": "https://github.com/yourusername/project.git",
  "metrics": {
    "ate_rmse_m": 88.2281,
    "rpe_trans_drift_m_per_m": 2.04084,
    "rpe_rot_drift_deg_per_100m": 76.69911,
    "completeness_pct": 95.73
  }
}
```

### Field Descriptions

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `group_name` | string | Your group name (as shown on the leaderboard) | `"Team Alpha"` |
| `project_private_repo_url` | string | Private Git repository URL for your project | `"https://github.com/yourusername/project.git"` |
| `metrics.ate_rmse_m` | number | ATE RMSE in meters | `88.2281` |
| `metrics.rpe_trans_drift_m_per_m` | number | Translation drift rate | `2.04084` |
| `metrics.rpe_rot_drift_deg_per_100m` | number | Rotation drift rate | `76.69911` |
| `metrics.completeness_pct` | number | Completeness percentage | `95.73` |

### File Naming Convention

`{GroupName}_leaderboard.json`

Example: `Team_Alpha_leaderboard.json`

---

## 💡 How to Generate Submission

### Step 1: Run ORB-SLAM3 and export your trajectory

1. Run your monocular VO pipeline (ORB-SLAM3).
2. Export **full-frame** trajectory as `CameraTrajectory.txt` (not keyframes).
3. Ensure the trajectory is valid **TUM format**: `t tx ty tz qx qy qz qw` with timestamps in seconds.

If you are unsure about export pitfalls, see:

- `ORB_SLAM3_TIPS.md`

### Step 2: Compute metrics (matches leaderboard protocol)

Option A (recommended): use the provided evaluation script in this repository:

```bash
# Run from the repository root (so "scripts/" exists)
python scripts/evaluate_vo_accuracy.py \
  --groundtruth ground_truth.txt \
  --estimated CameraTrajectory.txt \
  --t-max-diff 0.1 \
  --delta-m 10 \
  --workdir evaluation_results \
  --json-out evaluation_results/metrics.json
```

Option B: use native `evo` CLI commands (see the Metric Calculation section below).

### Step 3: Create `{GroupName}_leaderboard.json`

Fill the four metrics in the required JSON schema (see `submission_template.json`):

- `ate_rmse_m`
- `rpe_trans_drift_m_per_m`
- `rpe_rot_drift_deg_per_100m`
- `completeness_pct`

### Step 4: Verify JSON format (recommended)

```python
import json

with open("Team_Alpha_leaderboard.json", "r", encoding="utf-8") as f:
    submission = json.load(f)

assert "group_name" in submission, "Missing 'group_name'"
assert "project_private_repo_url" in submission, "Missing 'project_private_repo_url'"
assert "metrics" in submission, "Missing 'metrics'"

metrics = submission["metrics"]
required = [
    "ate_rmse_m",
    "rpe_trans_drift_m_per_m",
    "rpe_rot_drift_deg_per_100m",
    "completeness_pct",
]
for key in required:
    assert key in metrics, f"Missing metrics.{key}"
    assert isinstance(metrics[key], (int, float)), f"metrics.{key} must be a number"

assert submission["project_private_repo_url"].startswith("https://github.com/"), "Invalid GitHub URL"
assert submission["project_private_repo_url"].endswith(".git"), "URL should end with .git"

print("✅ Submission format is valid!")
print(f"Group: {submission['group_name']}")
print(f"ATE RMSE: {metrics['ate_rmse_m']} m")
```

## ✅ Trajectory Requirements (read this before evaluating)

- Use **`CameraTrajectory.txt`** (full-frame trajectory), not `KeyFrameTrajectory.txt`.
- Ensure your file is valid **TUM format**: `t tx ty tz qx qy qz qw` with timestamps in seconds.

For a practical “do this, avoid that” guide, see:

- `ORB_SLAM3_TIPS.md`

---

## 🔢 Metric Calculation

This section describes how to compute the metrics **locally** in a way that matches the leaderboard.

### 1. ATE RMSE (Sim(3) aligned, scale corrected)

```python
# After Sim(3) alignment
errors = np.linalg.norm(P_gt - P_aligned, axis=1)
ate = np.sqrt(np.mean(errors ** 2))  # RMSE in meters
```

Recommended `evo` command:

```bash
evo_ape tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va
```

### 2. RPE Translation Drift (m/m)

```python
# Over a distance interval of delta_d meters (delta_d = 10 m)
# evo reports mean RPE in meters over delta_d
rpe_trans_drift_m_per_m = rpe_trans_mean_m / delta_d
```

Recommended `evo` command:

```bash
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation trans_part -va
```

### 3. RPE Rotation Drift (deg/100m)

```python
# evo reports mean rotation angle error in degrees over delta_d
rpe_rot_drift_deg_per_100m = (rpe_rot_mean_deg / delta_d) * 100.0
```

Recommended `evo` command:

```bash
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation angle_deg -va
```

### 4. Completeness (%)

```python
completeness_pct = matched_poses / gt_poses * 100.0
```

Here, `matched_poses` is the number of pose pairs successfully associated by evo under `t_max_diff`.

---

## 📊 Baseline Results

| Metric | Baseline Value |
|--------|----------------|
| **ATE RMSE** | **88.2281 m** |
| **RPE Trans Drift** | **2.04084 m/m** |
| **RPE Rot Drift** | **76.69911 deg/100m** |
| **Completeness** | **95.73 %** |

---

## 🧠 Tips for Improvement

### Easy (fix common mistakes first)

1. Evaluate `CameraTrajectory.txt` (full-frame), not `KeyFrameTrajectory.txt`.
2. Verify timestamps are **seconds** and monotonically increasing.
3. Confirm camera intrinsics / distortion and the RGB/BGR setting in your camera config.

### Medium (typical VO tuning)

4. Increase ORB feature count (e.g., `nFeatures` ≈ 2000–2500).
5. Lower FAST thresholds to detect more features in low-texture regions.
6. Reduce motion blur and improve image quality (sharp frames = better tracking).

### Advanced (if permitted by your setup)

7. Use stereo or visual-inertial mode if the assignment allows it.
8. Use relocalization / loop-closure features when running SLAM (if allowed by the evaluation protocol).
9. Add image preprocessing (contrast normalization) for challenging lighting.

## 🌐 Leaderboard Website & Baseline

### Website

Leaderboard URL: `https://qian9921.github.io/leaderboard_web/`

### Baseline (AMtown02 sequence)

The following baseline is computed on the AMtown02 sequence using the fixed evaluation protocol above:

- `ate_rmse_m`: **88.2281**
- `rpe_trans_drift_m_per_m`: **2.04084**
- `rpe_rot_drift_deg_per_100m`: **76.69911**
- `completeness_pct`: **95.73**

