# ORB-SLAM3-DJI-AMtown

### v1.1.0, 12th June 2026 — Reproducibility & engineering overhaul

- Repository restructured: `results/` per dataset, `docs/`, `tools/`, `scripts/`; ROS package moved to `Examples/ROS` (fixes `build_ros.sh`); legacy `Examples_old` and course-demo directories removed.
- Salvaged previously uncommitted experiment material: k1 ablation configs, calibration variants A–E, full-resolution config and extractor, SfM sample-infos ground truth.
- README rewritten as a project showcase (key findings, figures, roadmap).
- Reproducible environment: `Dockerfile` (ROS Noetic + Pangolin v0.6), one-command `scripts/{download_dataset,run_amtown,evaluate}.sh` with dataset checksum; verified end-to-end in a fresh container (ATE 2.243 m vs SfM GT).
- CI: GitHub Actions builds core library + ROS nodes and lints shell scripts.
- Experiment conclusions consolidated in `docs/EXPERIMENTS.md`; reproduction protocol in `docs/REPRODUCIBILITY.md`.

### v1.0-release — AAE5303 course submission

- Monocular VO on AMtown02: ATE RMSE 2.310 m (SfM GT) / 2.647 m (RTK GPS GT), 100% completeness.
- Gimbal time-varying extrinsics analysis; Virtual IMU mono-inertial investigation; TUM-VI VIO validation (ATE 0.011 m).

---

# ORB-SLAM3 (upstream)
Details of changes between the different versions.

### V1.0, 22th December 2021

- OpenCV static matrices changed to Eigen matrices. The average code speed-up is 16% in tracking and 19% in mapping, w.r.t. times reported in the ORB-SLAM3 paper.

- New calibration file format, see file Calibration_Tutorial. Added options for stereo rectification and image resizing.

- Added load/save map functionalities.

- Added examples of live SLAM using Intel Realsense cameras.

- Fixed several bugs.

### V0.4: Beta version, 21st April 2021

- Changed OpenCV dynamic matrices to static matrices to speed up the code.

- Capability to measure running time of the system threads.

- Compatibility with OpenCV 4.0 (Requires at least OpenCV 3.0). 

- Fixed minor bugs.


### V0.3: Beta version, 4th Sep 2020

- RGB-D compatibility: the RGB-D examples have been adapted to the new version.

- Kitti and TUM dataset compatibility: these examples have been adapted to the new version.

- ROS compatibility: updated the old references in the code to work with this version.

- Config file parser: the YAML file contains the session configuration, a wrong parametrization may break the execution without any information to solve it. This version parses the file to read all the fields and give a proper answer if one of the fields have been wrongly deffined or does not exist.

- Fixed minor bugs.


### V0.2: Beta version, 7th Aug 2020
Initial release. It has these capabilities:

- Multiple-Map capabilities: it is able to handle multiple maps in the same session and merge them when a common area is detected with a seamless fussion.

- Inertial sensor: the IMU initialization takes 2 seconds to achieve a scale error less than 5\% and it is reffined in the next 10 seconds until it is around 1\%. Inertial measures are integrated at frame rate to estimate the scale, gravity and velocity in order to improve the visual features detection and make the system robust to temporal occlusions.

- Fisheye cameras: cameras with wide-angle and fisheye lenses are now fully supported in monocular and stereo. 


