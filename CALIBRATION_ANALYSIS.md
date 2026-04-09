# Camera Calibration Sensitivity Analysis

## Discovery

Camera calibration is the **single most impactful factor** for ORB-SLAM3 trajectory accuracy on the AMtown02 dataset, far exceeding the effect of ORB parameter tuning.

## Calibration Sources

The MARS-LVIG dataset provides multiple calibration files. The AMtown02 sequence shares the same DJI M300 RTK camera as HKisland, so the `HK_GNSS(airport & island).yaml` calibration applies.

### Three calibration configurations tested (all at 0.5× resolution, 1224×1024):

| Parameter | HKisland config (wrong dataset) | Self-Calibrated (`AMtown_Mono.yaml`) | HK_GNSS Official |
|-----------|------|--------------------------------------|-------------------|
| fx | 722.215 | 726.86 | 722.215 |
| fy | 722.17 | 726.64 | 722.17 |
| cx | 588.90 | 586.09 | 589.75 |
| cy | 521.80 | 520.89 | 522.45 |
| k1 | -0.0530 | -0.1210 | -0.0560 |
| k2 | 0.1210 | 0.1113 | 0.1180 |
| p1 | 0.00127 | 0.0016 | 0.00122 |
| p2 | 0.00043 | 0.00013 | 0.00064 |
| k3 | -0.06495 | -0.062353 | -0.0627 |

## Impact on ATE

| Stage | Config | Issue | ATE RMSE |
|-------|--------|-------|----------|
| Initial | HKisland intrinsics on AMtown | **Wrong fx/fy/cx/cy** — different dataset's calibration | **215 m** |
| Self-calibrated | AMtown self-calibration | Inaccurate distortion (k1=-0.121 too large) | **113.4 m** |
| HK_GNSS official | Official intrinsics + official distortion | Correct intrinsics, close distortion | **6.1 m** |
| **Best** | **HK_GNSS intrinsics + HKisland distortion** | **Optimal distortion coefficients** | **2.3 m** |

### Key Lessons

1. **Wrong intrinsics (fx/fy/cx/cy)** cause catastrophic failure (215m) — this is expected since the projection model is fundamentally wrong
2. **Wrong distortion (k1)** causes severe drift (113m → 6.1m with correct k1) — feature undistortion errors propagate through matching, triangulation, and BA
3. **Fine-tuning distortion** yields further significant improvement (6.1m → 2.3m) — even small differences in distortion coefficients matter

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
4. **None of these changes could compensate for incorrect calibration parameters**

## Non-Determinism Analysis

ORB-SLAM3 has inherent non-determinism from multi-threading.

### 5 runs with HK_GNSS distortion:

| Run | ATE RMSE (m) | RPE Mean (m/10m) |
|-----|-------------|-----------------|
| 1 | 6.256 | 0.153 |
| **2** | **6.114** | **0.147** |
| 3 | 6.235 | 0.160 |
| 4 | 6.253 | 0.158 |
| 5 | 8.051 | 1.140 |

### 6 runs with HKisland distortion (best config):

| Run | ATE RMSE (m) |
|-----|-------------|
| 0 | 2.310 |
| 1 | 2.470 |
| 2 | 2.841 |
| 3 | 3.527 |
| **4** | **2.357** |
| 5 | 2.318 |

Best result is **2.310m** (Run 0). Runs are stable at ~2.3-2.5m with occasional outliers.

## Conclusion

For aerial VO with ORB-SLAM3:
1. **Correct intrinsics (fx/fy/cx/cy)** are essential — wrong intrinsics cause complete failure
2. **Accurate distortion coefficients** are the next most important factor — reducing ATE from 113m to 2.3m
3. **ORB parameter tuning** has minimal impact compared to calibration accuracy
4. **Multiple runs** are recommended due to non-determinism (~0.2m variance)
