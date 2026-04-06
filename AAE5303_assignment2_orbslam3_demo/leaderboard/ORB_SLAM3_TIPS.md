## ORB-SLAM3 Tips (Run → Export Trajectory → Evaluate)

This document is a practical checklist for AAE5303 students to avoid common pitfalls when running ORB-SLAM3 (monocular VO) and evaluating trajectories with `evo`.

This course repository contains a ROS monocular node example under:

- `Examples_old/ROS/ORB_SLAM3/src/ros_mono_compressed.cc`

If your workflow is not ROS-based, the same principles still apply: **export a full-frame TUM trajectory with timestamps in seconds** and evaluate it consistently.

---

### 1) What you must output (critical)

For the leaderboard **you must evaluate a full-frame trajectory**, not a keyframe-only trajectory.

- **Use**: `CameraTrajectory.txt` (all frames / all tracked timestamps)
- **Do not use**: `KeyFrameTrajectory.txt` (only keyframes; completeness becomes meaningless and metrics are biased)

**Why?**

- RPE drift rates and completeness depend on how densely you output poses.
- Keyframes are a sparse subset and are chosen by internal heuristics; different tuning can change keyframe density and “game” the metrics.

---

### 2) Trajectory file format (TUM)

Your trajectory must follow **TUM format** with 8 columns per line:

```text
# timestamp tx ty tz qx qy qz qw
1698132964.499888 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
...
```

Notes:

- The **timestamp must be in seconds** (floating point).
- `evo` associates poses by timestamp. If you accidentally write frame indices instead of timestamps, `evo` will fail with “no matching timestamps”.

---

### 3) Exporting `CameraTrajectory.txt` from ORB-SLAM3 (example)

If you are using a ROS wrapper (or any custom main), make sure you call **both**:

- `SaveTrajectoryTUM("CameraTrajectory.txt")`  ← full trajectory
- `SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt")` (optional)

Example snippet (C++):

```cpp
// Stop all threads
SLAM.Shutdown();

// Save full camera trajectory (all frames) - required for evaluation
SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

// Save keyframe trajectory (optional)
SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
```

**Important**: call `Shutdown()` before saving trajectories to avoid data races (saving while mapping/loop threads are still running can crash).

#### Quick sanity check (is it really a full trajectory?)

- `CameraTrajectory.txt` should typically have **thousands of lines** (close to the number of input images that were processed).
- `KeyFrameTrajectory.txt` typically has **hundreds of lines** at most.

You can also open the file and confirm each line has 8 columns:

```text
t tx ty tz qx qy qz qw
```

---

### 4) Image color order: BGR vs RGB

If you decode images with OpenCV (e.g., `cv::imdecode`, `cv::imread`), the default channel order is typically **BGR**.

In ORB-SLAM3 settings:

- `Camera.RGB: 0` means input images are **BGR**
- `Camera.RGB: 1` means input images are **RGB**

Using the wrong value can hurt tracking because grayscale conversion will be wrong (`RGB2GRAY` vs `BGR2GRAY`).

---

### 5) ROS bag + compressed images (if applicable)

If you are using the provided `Mono_Compressed` node:

- The node subscribes to: `/camera/image_raw/compressed`
- If your bag topic is different, remap it when playing the bag.

Example pattern:

```bash
rosbag play your_dataset.bag /your/image/topic:=/camera/image_raw/compressed
```

**Tip**: start ORB-SLAM3 first, then start `rosbag play`, so the subscriber is ready.

---

### 5) Timestamp association parameters (`evo`)

The leaderboard uses:

- `t_max_diff = 0.1 s`

This is a trade-off:

- Too small → fewer matches → low completeness
- Too large → incorrect matches → misleading metrics

---

### 6) Correct evaluation commands (native `evo`)

Run these from the folder where your `ground_truth.txt` and `CameraTrajectory.txt` are located.

#### ATE (Sim(3) alignment + scale correction)

```bash
evo_ape tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va
```

#### RPE translation (distance-based, delta = 10 m)

```bash
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation trans_part -va
```

Convert to drift rate:

```text
RPE_trans_drift_m_per_m = RPE_trans_mean_m / 10
```

#### RPE rotation (distance-based, delta = 10 m)

```bash
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation angle_deg -va
```

Convert to drift rate:

```text
RPE_rot_drift_deg_per_100m = (RPE_rot_mean_deg / 10) * 100
```

#### Completeness

```text
Completeness (%) = matched_poses / gt_poses * 100
```

Where `matched_poses` is the number of successfully associated pose pairs under `t_max_diff`.

---

### 7) Common runtime pitfalls

- **“Segmentation fault when saving trajectory”**
  - Usually caused by saving while background threads are still running.
  - Always call `SLAM.Shutdown()` first (and ensure the shutdown really waits for threads to finish).

- **“Found no matching timestamps” (evo)**
  - Wrong dataset pair (GT and trajectory from different runs).
  - Timestamp units mismatch (frame index vs seconds).
  - Wrong `t_max_diff` (too small).

- **Completeness is unexpectedly low**
  - You evaluated `KeyFrameTrajectory.txt` instead of `CameraTrajectory.txt`.
  - Your trajectory is missing timestamps for a large part of the sequence (tracking lost or you did not log all frames).

---

### 8) Suggested debug checklist (fast)

1. Verify your trajectory file has 8 columns and increasing timestamps.
2. Confirm you are evaluating `CameraTrajectory.txt`.
3. Run `evo_ape` once and check that it prints “Found X of max. Y possible matching timestamps”.
4. If X is small, fix timestamps first before tuning any VO parameters.

---

### 9) What to submit (leaderboard JSON)

Use the template:

- `submission_template.json`

Required fields:

- `group_name`
- `project_private_repo_url`

Metrics to report:

- `ate_rmse_m`
- `rpe_trans_drift_m_per_m`
- `rpe_rot_drift_deg_per_100m`
- `completeness_pct`

If you are unsure which numbers to copy from evo:

- For ATE: use **rmse** from `evo_ape` output
- For RPE translation: compute `mean / 10` (m/m) from the `evo_rpe trans_part` output
- For RPE rotation: compute `(mean / 10) * 100` (deg/100m) from the `evo_rpe angle_deg` output

