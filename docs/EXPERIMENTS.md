# Experiment Log

Consolidated findings from the systematic experiments tracked in issues
[#13](https://github.com/AfonsoZhang/ORB-SLAM3-DJI-AMtown/issues/13)–[#18](https://github.com/AfonsoZhang/ORB-SLAM3-DJI-AMtown/issues/18).
Full calibration narrative in [CALIBRATION_ANALYSIS.md](CALIBRATION_ANALYSIS.md);
reproduction protocol in [REPRODUCIBILITY.md](REPRODUCIBILITY.md).

## 1. Calibration sensitivity: k1 dominates ATE (#13)

**Question:** Which input dominates trajectory accuracy — calibration or ORB parameters?

**Method:** Compare self-calibrated vs. dataset-official intrinsics/distortion, holding ORB settings constant; separately sweep ORB parameters with calibration fixed.

| Calibration | k1 | ATE RMSE |
|-------------|------|----------|
| Self-calibrated | -0.121 | 113 m |
| MARS-LVIG official | -0.056 | **6.1 m** |

**Conclusion:** A 2× error in k1 caused 18.6× worse ATE, while ORB parameter tuning moved ATE by only ~7%. Verify calibration against the dataset provider's official parameters before any parameter tuning.

## 2. k1 ablation: 46× ATE swing from one coefficient (#18)

**Question:** Isolate the effect of the first-order radial distortion coefficient k1.

**Method:** Vary k1 only; intrinsics, k2/p1/p2/k3, and ORB settings held constant (configs: `Examples/Monocular/ablation_k1_*.yaml`).

| k1 | ATE RMSE (m) | Relative |
|--------|-------------|----------|
| -0.121 | 110.7 | 46.3× worse |
| -0.070 | — | tracking failure |
| -0.056 | 5.25 | 2.2× worse |
| **-0.053** | **2.39** | **optimal** |
| -0.045 | 6.45 | 2.7× worse |
| -0.035 | 22.1 | 9.3× worse |

**Conclusion:** k1 alone changes ATE by 46×, with a sharp optimum at −0.053 (±0.003 doubles the error) and a tracking-failure cliff near −0.070. The distortion model is decisive for monocular VO accuracy. Figure: `figures/presentation/k1_ablation.png`.

## 3. Dual ground truth: SfM vs RTK GPS (#14)

**Question:** Do independent ground truth sources agree on trajectory quality?

**Method:** Evaluate the same trajectory against SfM reconstruction GT (`data/ground_truth_sfm.txt`, from `sampleinfos_interpolated.json`) and RTK GPS GT (`data/AMtown02_groundtruth.txt`).

| Metric | SfM GT | RTK GPS GT |
|--------|--------|------------|
| ATE RMSE | consistent | consistent (within ~10%) |
| RPE (10 m) | reliable | dominated by GPS noise |

**Conclusion:** ATE is consistent across both GTs, validating trajectory quality. RPE over short baselines is only meaningful against SfM GT — RTK GPS high-frequency noise distorts relative-pose comparisons. Report ATE against both, RPE against SfM only.

## 4. Offline vs online processing (#15)

**Question:** Does ROS real-time processing cost accuracy?

**Method:** Same data and config; online `ros_mono_compressed` + `rosbag play --rate 0.5` vs offline `mono_euroc`.

| Mode | ATE RMSE | Completeness |
|------|----------|--------------|
| Online (ROS) | 90.2 m | 98.7% |
| **Offline (mono_euroc)** | **6.1 m** | **100%** |

**Conclusion:** Offline processing eliminates frame drops, real-time pressure, and message-queue reordering — a 15× ATE improvement at identical configuration. Use offline mode for accuracy benchmarks; online mode only to demonstrate real-time viability.

## 5. Multi-threading non-determinism (#16)

**Question:** How much do identical runs differ?

**Method:** 5 identical offline runs (MARS-LVIG calibration era, ATE ≈ 6 m).

| Run | ATE RMSE (m) |
|-----|-------------|
| 1 | 6.256 |
| 2 | **6.114** (best) |
| 3 | 6.235 |
| 4 | 6.253 |
| 5 | 8.051 (outlier) |

**Conclusion:** Runs cluster within ±1.6%, with occasional ~+30% outliers when loop closure triggers suboptimally. Thread scheduling perturbs keyframe culling and loop-closure timing. Run 3–5 times; report the distribution or the best while saying so. With the final calibration, single runs land in 2.2–2.9 m ([REPRODUCIBILITY.md](REPRODUCIBILITY.md)).

## 6. Full resolution is not better (#17)

**Question:** Does 4× more pixels (2448×2048) improve accuracy over 0.5× downsampling?

**Method:** Full-resolution sequence (`tools/extract_fullres.py`, `Examples/Monocular/AMtown_Mono_Full.yaml`, 8000 features) vs 1224×1024 (4000 features), self-calibrated intrinsics era.

| Resolution | nFeatures | ATE RMSE | Processing time |
|------------|-----------|----------|-----------------|
| 1224×1024 (0.5×) | 4000 | 113.4 m | ~15 min |
| 2448×2048 (full) | 8000 | 121.0 m | ~35 min |

**Conclusion:** Full resolution was slightly *worse* (+7%) at 2.3× the cost: more pixels add detection noise, extra features add low-quality matches, and the flat high-altitude scene gains nothing from finer detail — while slower frames starve the mapping threads. 0.5× downsampling is the right trade-off here.
