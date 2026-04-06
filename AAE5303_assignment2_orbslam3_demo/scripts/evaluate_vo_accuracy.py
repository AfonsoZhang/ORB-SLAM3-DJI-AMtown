#!/usr/bin/env python3
"""
AAE5303 Visual Odometry Accuracy Evaluation Script

Evaluates monocular VO trajectories using evo with four parallel metrics:
1. ATE RMSE (m) with Sim(3) alignment + scale correction (monocular-friendly)
2. RPE translation drift (m/m) computed over a fixed distance delta (meters)
3. RPE rotation drift (deg/100m) computed over the same distance delta
4. Completeness (%) = matched poses / total ground-truth poses

Usage:
    python3 evaluate_vo_accuracy.py \
        --groundtruth rtk_groundtruth.txt \
        --estimated CameraTrajectory.txt
"""

import argparse
import json
import os
import subprocess
import sys
import zipfile
from dataclasses import dataclass
from io import BytesIO
from typing import Dict, List, Tuple

import numpy as np


@dataclass(frozen=True)
class EvoStats:
    """Summary statistics returned by evo (stats.json)."""

    rmse: float
    mean: float
    std: float

def _count_valid_tum_poses(path: str) -> int:
    """Count valid TUM pose lines: 't tx ty tz qx qy qz qw' (8 columns)."""
    count = 0
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                float(parts[0])
            except ValueError:
                continue
            count += 1
    return count


def _read_evo_stats(zip_path: str) -> EvoStats:
    """Read evo stats.json from a --save_results zip file."""
    with zipfile.ZipFile(zip_path, "r") as zf:
        stats = json.loads(zf.read("stats.json").decode("utf-8"))
    return EvoStats(rmse=float(stats["rmse"]), mean=float(stats["mean"]), std=float(stats["std"]))


def _read_evo_timestamps_count(zip_path: str) -> int:
    """Read timestamps.npy length from an evo --save_results zip file."""
    with zipfile.ZipFile(zip_path, "r") as zf:
        data = zf.read("timestamps.npy")
    arr = np.load(BytesIO(data), allow_pickle=False)
    return int(arr.shape[0])


def _run(cmd: List[str]) -> None:
    """Run a command and stream its output."""
    proc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"Command failed (exit={proc.returncode}): {' '.join(cmd)}")


def evaluate_with_evo(
    gt_path: str,
    est_path: str,
    t_max_diff_s: float,
    delta_m: float,
    workdir: str,
) -> Dict[str, float]:
    """
    Compute metrics by calling evo_ape/evo_rpe and parsing --save_results outputs.

    Returns a flat metrics dict suitable for logging or JSON export.
    """
    os.makedirs(workdir, exist_ok=True)
    ate_zip = os.path.join(workdir, "ate.zip")
    rpe_trans_zip = os.path.join(workdir, "rpe_trans.zip")
    rpe_rot_zip = os.path.join(workdir, "rpe_rot.zip")

    # ATE (Sim3 + scale correction)
    _run(
        [
            "evo_ape",
            "tum",
            gt_path,
            est_path,
            "--align",
            "--correct_scale",
            "--t_max_diff",
            str(t_max_diff_s),
            "--save_results",
            ate_zip,
            "--no_warnings",
            "-va",
        ]
    )
    ate = _read_evo_stats(ate_zip)

    # RPE translation over distance delta (Sim3 + scale correction)
    _run(
        [
            "evo_rpe",
            "tum",
            gt_path,
            est_path,
            "--align",
            "--correct_scale",
            "--t_max_diff",
            str(t_max_diff_s),
            "--delta",
            str(delta_m),
            "--delta_unit",
            "m",
            "--pose_relation",
            "trans_part",
            "--save_results",
            rpe_trans_zip,
            "--no_warnings",
            "-va",
        ]
    )
    rpe_trans = _read_evo_stats(rpe_trans_zip)

    # RPE rotation angle over distance delta (degrees)
    _run(
        [
            "evo_rpe",
            "tum",
            gt_path,
            est_path,
            "--align",
            "--correct_scale",
            "--t_max_diff",
            str(t_max_diff_s),
            "--delta",
            str(delta_m),
            "--delta_unit",
            "m",
            "--pose_relation",
            "angle_deg",
            "--save_results",
            rpe_rot_zip,
            "--no_warnings",
            "-va",
        ]
    )
    rpe_rot = _read_evo_stats(rpe_rot_zip)

    # Completeness: how many poses were actually associated and evaluated
    gt_total = _count_valid_tum_poses(gt_path)
    matched = _read_evo_timestamps_count(ate_zip)
    completeness = 0.0 if gt_total <= 0 else 100.0 * matched / gt_total

    # Drift rates (normalize by delta distance)
    rpe_trans_drift_m_per_m = rpe_trans.mean / delta_m
    rpe_rot_drift_deg_per_100m = (rpe_rot.mean / delta_m) * 100.0

    return {
        "ate_rmse_m": float(ate.rmse),
        "ate_mean_m": float(ate.mean),
        "ate_std_m": float(ate.std),
        "rpe_trans_mean_m": float(rpe_trans.mean),
        "rpe_trans_rmse_m": float(rpe_trans.rmse),
        "rpe_trans_drift_m_per_m": float(rpe_trans_drift_m_per_m),
        "rpe_rot_mean_deg": float(rpe_rot.mean),
        "rpe_rot_rmse_deg": float(rpe_rot.rmse),
        "rpe_rot_drift_deg_per_100m": float(rpe_rot_drift_deg_per_100m),
        "matched_poses": int(matched),
        "gt_poses": int(gt_total),
        "completeness_pct": float(completeness),
        "t_max_diff_s": float(t_max_diff_s),
        "delta_m": float(delta_m),
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate monocular VO with evo (ATE/RPE/Completeness).")
    parser.add_argument("--groundtruth", required=True, help="Ground truth trajectory (TUM format).")
    parser.add_argument("--estimated", required=True, help="Estimated trajectory (TUM format).")
    parser.add_argument("--t-max-diff", type=float, default=0.1, help="Max timestamp association difference (seconds).")
    parser.add_argument("--delta-m", type=float, default=10.0, help="Distance delta for RPE (meters).")
    parser.add_argument("--workdir", default="evaluation_results", help="Directory to store evo result zips.")
    parser.add_argument("--json-out", default="", help="Optional path to write a JSON report.")

    args = parser.parse_args()

    print("=" * 80)
    print("AAE5303 MONOCULAR VO EVALUATION (evo)")
    print("=" * 80)
    print(f"Ground truth: {args.groundtruth}")
    print(f"Estimated:    {args.estimated}")
    print(f"Association:  t_max_diff = {args.t_max_diff:.3f} s")
    print(f"RPE delta:    {args.delta_m:.3f} m")
    print("")

    try:
        metrics = evaluate_with_evo(
            gt_path=args.groundtruth,
            est_path=args.estimated,
            t_max_diff_s=args.t_max_diff,
            delta_m=args.delta_m,
            workdir=args.workdir,
        )
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        print("Hint: install evo (pip install evo) and ensure evo_ape/evo_rpe are on PATH.")
        return 1
    except RuntimeError as e:
        print(f"ERROR: {e}")
        return 1

    print("")
    print("=" * 80)
    print("PARALLEL METRICS (NO WEIGHTING)")
    print("=" * 80)
    print(f"ATE RMSE (m):                 {metrics['ate_rmse_m']:.6f}")
    print(f"RPE trans drift (m/m):        {metrics['rpe_trans_drift_m_per_m']:.6f}")
    print(f"RPE rot drift (deg/100m):     {metrics['rpe_rot_drift_deg_per_100m']:.6f}")
    print(f"Completeness (%):             {metrics['completeness_pct']:.2f}  "
          f"({metrics['matched_poses']} / {metrics['gt_poses']})")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2, sort_keys=True)
        print("")
        print(f"Saved JSON report to: {args.json_out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

