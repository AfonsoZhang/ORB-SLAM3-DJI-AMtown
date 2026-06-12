# Reproducibility Notes

## Expected results

Running the offline pipeline (`scripts/run_amtown.sh` + `scripts/evaluate.sh`) with
`AMtown_Mono_MARSLVIG.yaml` should produce:

| Metric (vs SfM GT) | Expected |
|--------------------|----------|
| ATE RMSE | **2.3 – 2.9 m** (single run) |
| Best observed | 2.308 m |
| Completeness | ~7400 / 7499 frames |
| Maps in atlas | 1 (no resets), loop closures detected |

A single run landing anywhere in the 2.3–2.9 m band reproduces this project's result.
The headline number (2.310 m) is the best of 6 runs.

## Why runs differ: multi-threading non-determinism

ORB-SLAM3 runs tracking, local mapping, and loop closing in parallel threads.
Thread scheduling changes which keyframes get culled and when loop closure /
local BA fire, so two runs on identical input produce slightly different
trajectories. This is inherent to the architecture, not a configuration error.

A 5-run study on an earlier configuration of this project (ATE ≈ 6 m era,
[issue #16](https://github.com/AfonsoZhang/ORB-SLAM3-DJI-AMtown/issues/16)) measured:

- Runs 1–4: 6.11 – 6.26 m → consistent, ≈ ±1.6% relative variance
- Run 5: 8.05 m → occasional outlier (~+30%) when loop closure triggers suboptimally

With the final calibration, observed single-run ATEs were 2.31 – 2.60 m across
8 runs (6 original + 2 regression runs during repo restructuring).

**Recommendation:** run 3–5 times and report the distribution (or the best, stating so).

## Environment

Results were produced in the `liangyu99/orbslam3_ros1` Docker image; the
provided [`Dockerfile`](../Dockerfile) recreates an equivalent environment:

| Component | Version |
|-----------|---------|
| Ubuntu | 20.04 (focal) |
| ROS | Noetic |
| OpenCV | 4.2.0 (system, required by CMakeLists) |
| Eigen | 3.3.7 |
| Pangolin | v0.6 |
| evo | ≥ 1.31 |

The offline pipeline is fully headless (`mono_euroc` is built with the viewer
disabled); no X server or GPU is required. A full AMtown02 run takes ~13 minutes
plus a one-time ~10 minute image extraction from the rosbag.

## Evaluation protocol

- Trajectory timestamps are converted ns → s before evaluation.
- `evo_ape tum <GT> traj_sec.txt --align --correct_scale --t_max_diff 0.1`
  (Sim(3) Umeyama alignment — monocular scale is unobservable, so scale
  correction is required; expected scale factor ≈ 2.15).
- Primary GT: `data/ground_truth_sfm.txt` (SfM, 6899 poses).
  Secondary GT: `data/AMtown02_groundtruth.txt` (RTK GPS, 7500 poses, noisier).

## Dataset integrity

`scripts/download_dataset.sh` verifies the rosbag:

```
SHA256(AMtown02.bag) — see scripts/download_dataset.sh
```
