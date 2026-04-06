# 🏆 AAE5303 Visual Odometry – Leaderboard

This folder contains the **student-facing** leaderboard specification:

- What to submit
- Which metrics are used (four *parallel* metrics, no weighting)
- The fixed evaluation protocol (association + alignment), so results are comparable across teams

If you are looking for a “how to run ORB-SLAM3 without pitfalls” guide, see:

- `ORB_SLAM3_TIPS.md`

## 📌 Evaluation Dataset

The evaluation dataset (bag / images / ground truth) is provided by the instructor.

The leaderboard evaluates your submission by comparing:

- **Ground truth trajectory** (provided) in **TUM format**
- **Estimated trajectory** (your output) in **TUM format**

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset (AMtown02) | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

All sequences used in this course are sourced from the **official MARS-LVIG AMtown02** dataset.

## 📌 What you submit

Submit **one JSON file per group**:

- File name: `{GroupName}_leaderboard.json`
- Format template: `submission_template.json`

The leaderboard will parse your JSON file and display rankings **separately for each metric**.

## 📊 Metrics (four parallel metrics)

All metrics are computed by comparing an **estimated TUM trajectory** against the **provided ground truth**, using a fixed evaluation protocol (alignment + association).

| Metric | Direction | Unit | Description |
|--------|-----------|------|-------------|
| **ATE RMSE** | ↓ | m | Global accuracy after Sim(3) alignment + scale correction |
| **RPE Trans Drift** | ↓ | m/m | Translation drift rate (distance-based RPE, delta = 10 m) |
| **RPE Rot Drift** | ↓ | deg/100m | Rotation drift rate (distance-based RPE, delta = 10 m) |
| **Completeness** | ↑ | % | Matched poses / total ground-truth poses |

---

### 1. ATE RMSE (Absolute Trajectory Error) ➜

**Lower is better** | Unit: meters (m)

#### Definition

Absolute Trajectory Error (ATE) measures the **global** discrepancy between the estimated trajectory and the ground truth trajectory **after** applying a single best-fit **Sim(3)** alignment (rotation + translation + scale).

This Sim(3) alignment is required because monocular VO cannot observe absolute metric scale; otherwise, errors would be dominated by an arbitrary scale factor.

#### Mathematical Formula

Let \( \mathbf{p}^{gt}_i \in \mathbb{R}^3 \) be the i-th ground-truth position and \( \mathbf{p}^{est}_i \) be the aligned estimated position.

$$e_i = \|\mathbf{p}^{gt}_i - \mathbf{p}^{est}_i\|_2$$
$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N} e_i^2}$$

#### Reference Code

```python
import numpy as np

def ate_rmse(P_gt: np.ndarray, P_est_aligned: np.ndarray) -> float:
    """
    ATE RMSE in meters after timestamp association + Sim(3) alignment.

    Args:
        P_gt: (N, 3) ground-truth positions
        P_est_aligned: (N, 3) aligned estimated positions
    """
    errors = np.linalg.norm(P_gt - P_est_aligned, axis=1)
    return float(np.sqrt(np.mean(errors ** 2)))
```

---

### 2. RPE Translation Drift (m/m) ➜

**Lower is better** | Unit: meters per meter (m/m)

#### Definition

Relative Pose Error (RPE) translation is computed over a fixed **distance interval** \(\Delta d\) (the leaderboard uses \(\Delta d = 10\) m).

`evo_rpe` reports the mean translation error (meters) over that interval. The leaderboard converts it into a **drift rate** (m/m) by dividing by \(\Delta d\).

#### Mathematical Formula

$$RPE_{trans\_drift_{m/m}} = \frac{mean\_RPE_{trans_m}}{\Delta d}$$

#### Reference Code

```python
def rpe_trans_drift_m_per_m(rpe_trans_mean_m: float, delta_m: float = 10.0) -> float:
    return float(rpe_trans_mean_m / delta_m)
```

---

### 3. RPE Rotation Drift (deg/100m) ➜

**Lower is better** | Unit: degrees per 100 meters (deg/100m)

#### Definition

RPE rotation uses the mean **rotation angle error** (degrees) computed over the same distance interval \(\Delta d = 10\) m.

The leaderboard reports drift in degrees per 100 meters by scaling the mean-by-distance result.

#### Mathematical Formula

$$RPE_{rot\_drift_{deg/100m}} = \left(\frac{mean\_RPE_{rot_{deg}}}{\Delta d}\right)\times 100$$

#### Reference Code

```python
def rpe_rot_drift_deg_per_100m(rpe_rot_mean_deg: float, delta_m: float = 10.0) -> float:
    return float((rpe_rot_mean_deg / delta_m) * 100.0)
```

---

### 4. Completeness (%) ➜

**Higher is better** | Unit: percent (%)

#### Definition

Completeness measures how much of the ground-truth trajectory could be matched and evaluated under the fixed timestamp association tolerance (\(t_{max\_diff} = 0.1\) s).

#### Mathematical Formula

$$Completeness(\%) = \frac{N_{matched}}{N_{gt}} \times 100$$

#### Reference Code

```python
def completeness_pct(matched_poses: int, gt_poses: int) -> float:
    return float(0.0 if gt_poses <= 0 else 100.0 * matched_poses / gt_poses)
```

### What each metric measures (intuition)

#### ATE RMSE (m)

Absolute Trajectory Error measures the **global** discrepancy between the estimated trajectory and the ground truth **after** applying a single best-fit Sim(3) transform (rotation + translation + scale).

- Good ATE → your *overall* trajectory shape is close to ground truth.
- Bad ATE → strong accumulated drift, wrong relocalization, or inconsistent tracking.

#### RPE translation drift (m/m)

Relative Pose Error (translation) is computed over a fixed distance interval (10 m). `evo` reports mean translation error in meters over that interval, which we normalize into a drift rate:

```text
RPE_trans_drift_m_per_m = RPE_trans_mean_m / 10
```

This metric emphasizes **local drift** rather than cumulative error.

#### RPE rotation drift (deg/100m)

Relative Pose Error (rotation) uses the rotation angle error in degrees over the same 10 m distance interval, normalized as:

```text
RPE_rot_drift_deg_per_100m = (RPE_rot_mean_deg / 10) * 100
```

Large values typically indicate unstable orientation estimates and/or poor feature geometry.

#### Completeness (%)

Completeness measures how much of the ground-truth trajectory can be evaluated:

```text
Completeness (%) = matched_poses / gt_poses * 100
```

This discourages submissions that only output a short “easy” segment.

### Fixed evaluation parameters

- **Trajectory format**: TUM (`t tx ty tz qx qy qz qw`)
- **Timestamp association**: `t_max_diff = 0.1 s`
- **Alignment**: Sim(3) with scale correction (`--align --correct_scale`)
- **RPE delta**: `delta = 10 m` (distance domain)

### Why Sim(3) alignment is required for monocular VO

Monocular VO cannot observe absolute metric scale. Without Sim(3) alignment, metrics would be dominated by an arbitrary scale factor. Using Sim(3) with scale correction makes the metrics reflect:

- Trajectory **shape** consistency
- Drift and tracking quality

rather than the unknown global scale.

## ✅ How to compute the same numbers locally

See `LEADERBOARD_SUBMISSION_GUIDE.md` for:

- The exact `evo` commands used by the leaderboard
- How to compute drift rates from evo outputs
- The JSON schema and an example submission

## 📈 Complete Evaluation Script

This repository includes a ready-to-run script that computes all four leaderboard metrics using the **exact fixed protocol** (timestamp association + Sim(3) alignment + scale correction + distance-domain RPE):

- `scripts/evaluate_vo_accuracy.py` (run from the repository root)

Example usage:

```bash
python scripts/evaluate_vo_accuracy.py \
  --groundtruth ground_truth.txt \
  --estimated CameraTrajectory.txt \
  --t-max-diff 0.1 \
  --delta-m 10 \
  --workdir evaluation_results \
  --json-out evaluation_results/metrics.json
```

After that, generate your final `{GroupName}_leaderboard.json` using the required schema:

```python
import json
from pathlib import Path

metrics = json.loads(Path("evaluation_results/metrics.json").read_text(encoding="utf-8"))

submission = {
  "group_name": "Team Alpha",
  "project_private_repo_url": "https://github.com/yourusername/project.git",
  "metrics": {
    "ate_rmse_m": metrics["ate_rmse_m"],
    "rpe_trans_drift_m_per_m": metrics["rpe_trans_drift_m_per_m"],
    "rpe_rot_drift_deg_per_100m": metrics["rpe_rot_drift_deg_per_100m"],
    "completeness_pct": metrics["completeness_pct"],
  },
}

Path("TeamAlpha_leaderboard.json").write_text(json.dumps(submission, indent=2), encoding="utf-8")
print("Saved TeamAlpha_leaderboard.json")
```

## 📂 Submission Format

Submit **one JSON file per group** using the template `submission_template.json`.

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

**Required fields**

- `group_name`: Your group name (as shown on the leaderboard)
- `project_private_repo_url`: Private Git repository URL (GitHub, ending with `.git`)
- `metrics`: A dict containing the four leaderboard metrics

## 🧾 Submission example

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

## ❓ FAQ

### Q1: Can I submit `KeyFrameTrajectory.txt`?

No. Use `CameraTrajectory.txt` (full-frame trajectory). Keyframe-only trajectories distort completeness and drift-rate metrics.

### Q2: evo says “Found no matching timestamps”

Common causes:

- You evaluated with the wrong ground truth file.
- Your trajectory timestamps are not in seconds (e.g., frame indices).
- `t_max_diff` is too small (the leaderboard uses 0.1 s).

## 🏅 Current Leaderboard

| Rank | Group | ATE RMSE ↓ | RPE Trans Drift ↓ | RPE Rot Drift ↓ | Completeness ↑ | Date |
|------|-------|------------|-------------------|-----------------|----------------|------|
| - | **Baseline (AMtown02)** | 88.2281 m | 2.04084 m/m | 76.69911 deg/100m | 95.73 % | — |

### 📊 Baseline Details

Baseline numbers are computed on the **AMtown02** sequence using the fixed evaluation protocol:

- `t_max_diff = 0.1 s`
- Sim(3) alignment with scale correction (`--align --correct_scale`)
- distance-domain RPE with `delta = 10 m`

The leaderboard ranks teams **separately for each metric** (no weighting).

## 🚀 Leaderboard Website

**Live Rankings**: `https://qian9921.github.io/leaderboard_web/`

## 🌐 Website & Baseline

Leaderboard URL: `https://qian9921.github.io/leaderboard_web/`

Baseline (AMtown02):

- **ATE RMSE**: 88.2281 m
- **RPE Trans Drift**: 2.04084 m/m
- **RPE Rot Drift**: 76.69911 deg/100m
- **Completeness**: 95.73 %

