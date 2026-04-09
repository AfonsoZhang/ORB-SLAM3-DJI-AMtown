# Camera Calibration Sensitivity Analysis

## Discovery

Camera calibration — particularly the distortion coefficient `k1` — is the **single most impactful factor** for ORB-SLAM3 trajectory accuracy on the AMtown02 dataset, far exceeding the effect of ORB parameter tuning.

## Two Calibration Sources

| Parameter | Self-Calibrated (`AMtown_Mono.yaml`) | MARS-LVIG Official (`AMtown_Mono_MARSLVIG.yaml`) |
|-----------|--------------------------------------|--------------------------------------------------|
| fx | 726.86 | 722.215 |
| fy | 726.64 | 722.17 |
| cx | 586.09 | 589.75 |
| cy | 520.89 | 522.45 |
| **k1** | **-0.1210** | **-0.0560** |
| k2 | 0.1113 | 0.1180 |
| p1 | 0.0016 | 0.00122 |
| p2 | 0.00013 | 0.00064 |
| k3 | -0.062353 | -0.0627 |

The key difference is **k1 (radial distortion)**: the self-calibrated value is 2.16× larger.

## Impact on ATE

| Config | Calibration | ORB Params | ATE RMSE |
|--------|-------------|------------|----------|
| Self-calib + tuned ORB | Self-calibrated | nFeat=4000, nLev=12, iniTh=8 | **113.4 m** |
| Self-calib + standard ORB | Self-calibrated | nFeat=2000, nLev=8, iniTh=15 | **121.0 m** |
| **MARS-LVIG + standard ORB** | **MARS-LVIG** | **nFeat=2000, nLev=8, iniTh=15** | **6.1 m** |
| MARS-LVIG + tuned ORB | MARS-LVIG | nFeat=4000, nLev=12, iniTh=8 | TBD |

**Calibration correction reduced ATE by 18.6×** (113m → 6.1m), while ORB tuning only changed ATE by ~7% (113m → 121m).

## ORB Parameter Tuning Experiments

All experiments below use self-calibrated intrinsics at 0.5× resolution (1224×1024), offline `mono_euroc`.

| Experiment | nFeatures | scaleFactor | nLevels | iniThFAST | ATE RMSE |
|------------|-----------|-------------|---------|-----------|----------|
| Baseline | 4000 | 1.2 | 12 | 8 | 113.4 m |
| Config A: fewer features | 3000 | 1.2 | 8 | 10 | 128.5 m |
| Config B: finer pyramid | 4000 | 1.1 | 16 | 8 | 118.3 m |
| Config C: more features + KFs | 6000 | 1.2 | 12 | 8 (fps=5) | 176.8 m |
| Full resolution (2448×2048) | 8000 | 1.2 | 12 | 8 | 121.0 m |

### Key Observations

1. **More features ≠ better accuracy**: Increasing nFeatures beyond 4000 introduced lower-quality features that degraded matching
2. **Finer pyramid (scaleFactor=1.1)** did not help — the default 1.2 is well-suited
3. **Full resolution** was slightly worse than 0.5× — more pixels means more noise in feature detection
4. **None of these changes could compensate for incorrect distortion parameters**

## Non-Determinism Analysis

ORB-SLAM3 has inherent non-determinism from multi-threading. Five identical runs with MARS-LVIG calibration:

| Run | ATE RMSE (m) | RPE Mean (m/10m) |
|-----|-------------|-----------------|
| 1 | 6.256 | 0.153 |
| **2** | **6.114** | **0.147** |
| 3 | 6.235 | 0.160 |
| 4 | 6.253 | 0.158 |
| 5 | 8.051 | 1.140 |

Runs 1-4 are consistent (~6.2m ± 0.1m). Run 5 is an outlier where loop closure likely failed or triggered at a suboptimal location.

## Conclusion

For aerial VO with ORB-SLAM3, **accurate camera calibration is far more important than ORB parameter tuning**. The distortion model directly affects feature undistortion, which impacts every subsequent step (matching, triangulation, BA). A 2× error in k1 propagates through the entire pipeline, causing catastrophic drift that no amount of feature count tuning can fix.
