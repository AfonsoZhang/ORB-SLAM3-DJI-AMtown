#!/usr/bin/env python3
"""
Generate report figures for AAE5303 monocular VO demo.

This script is intentionally standalone and does NOT rely on any shell scripts.
It reads TUM trajectories, matches poses by timestamp, applies the Sim(3)
alignment estimated by evo (stored in evo_ape --save_results zip), and produces
a 2x2 report figure:

1) 2D trajectory before alignment (matched poses only)
2) 2D trajectory after Sim(3) alignment
3) ATE error histogram (from evo error_array.npy)
4) ATE error along trajectory (from evo error_array.npy)

Example:
  python3 scripts/generate_report_figures.py \
    --gt /root/ORB_SLAM3/ground_truth.txt \
    --est /root/ORB_SLAM3/CameraTrajectory.txt \
    --evo-ape-zip /tmp/aae5303_eval/ate.zip \
    --out figures/trajectory_evaluation.png
"""

from __future__ import annotations

import argparse
import zipfile
from dataclasses import dataclass
from io import BytesIO
from typing import List, Tuple

import numpy as np


@dataclass(frozen=True)
class TumTrajectory:
    """TUM trajectory with timestamps and positions."""

    t: np.ndarray  # shape (N,)
    p: np.ndarray  # shape (N, 3)


def generate_trajectory_evaluation_figure(
    gt_path: str,
    est_path: str,
    evo_ape_zip_path: str,
    out_path: str,
    t_max_diff_s: float,
    title_suffix: str,
) -> None:
    """
    Generate and save the standard 2x2 evaluation figure.

    Args:
        gt_path: Ground truth trajectory path (TUM format).
        est_path: Estimated trajectory path (TUM format).
        evo_ape_zip_path: evo_ape --save_results zip containing alignment + errors.
        out_path: Output PNG path.
        t_max_diff_s: Association threshold (seconds). Used for matching poses
            for visualization. Should match the one used in evo.
        title_suffix: Optional string to append to titles (e.g., dataset name).
    """
    import matplotlib.pyplot as plt  # local import to keep base deps minimal

    gt = _load_tum_positions(gt_path)
    est = _load_tum_positions(est_path)

    gt_idx, est_idx = _associate_by_time(gt.t, est.t, t_max_diff_s)
    if len(gt_idx) < 5:
        raise RuntimeError(f"Too few matched poses ({len(gt_idx)}) for plotting.")

    gt_m = gt.p[gt_idx]
    est_m = est.p[est_idx]

    sim3, ate_errors = _load_sim3_and_errors(evo_ape_zip_path)
    est_aligned = _apply_sim3(sim3, est_m)

    # Plot setup
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))

    # 1) Before alignment
    ax = axes[0, 0]
    ax.set_title(f"2D Trajectory - Before Alignment{title_suffix}")
    ax.plot(gt_m[:, 0], gt_m[:, 1], color="green", label="Ground Truth", linewidth=2)
    ax.plot(est_m[:, 0], est_m[:, 1], color="red", linestyle="--", label="VO (Unaligned)", linewidth=1.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 2) After Sim(3) alignment
    ax = axes[0, 1]
    ax.set_title(f"2D Trajectory - After Sim(3) Alignment{title_suffix}")
    ax.plot(gt_m[:, 0], gt_m[:, 1], color="green", label="Ground Truth", linewidth=2)
    ax.plot(est_aligned[:, 0], est_aligned[:, 1], color="blue", label="VO (Aligned)", linewidth=1.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 3) ATE histogram
    ax = axes[1, 0]
    ax.set_title("Absolute Trajectory Error Distribution")
    ax.hist(ate_errors, bins=40, color="#4C78A8", edgecolor="black", alpha=0.75)
    mean = float(np.mean(ate_errors))
    median = float(np.median(ate_errors))
    ax.axvline(mean, color="red", linestyle="--", linewidth=2, label=f"Mean: {mean:.2f} m")
    ax.axvline(median, color="orange", linestyle="--", linewidth=2, label=f"Median: {median:.2f} m")
    ax.set_xlabel("ATE [m]")
    ax.set_ylabel("Frequency")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 4) ATE over index
    ax = axes[1, 1]
    ax.set_title("ATE Error Along Trajectory")
    x = np.arange(len(ate_errors))
    ax.plot(x, ate_errors, color="blue", linewidth=1)
    ax.fill_between(x, ate_errors, color="#72B7B2", alpha=0.25)
    ax.set_xlabel("Matched Pose Index")
    ax.set_ylabel("ATE [m]")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate AAE5303 report figures (trajectory + ATE).")
    parser.add_argument("--gt", required=True, help="Ground truth trajectory (TUM format).")
    parser.add_argument("--est", required=True, help="Estimated trajectory (TUM format).")
    parser.add_argument("--evo-ape-zip", required=True, help="evo_ape --save_results zip path.")
    parser.add_argument("--out", required=True, help="Output figure path (.png).")
    parser.add_argument("--t-max-diff", type=float, default=0.1, help="Timestamp association threshold (seconds).")
    parser.add_argument("--title-suffix", default="", help="Optional title suffix, e.g., ' (HKisland_GNSS03)'.")
    args = parser.parse_args()

    title_suffix = f" {args.title_suffix}".rstrip()
    if title_suffix and not title_suffix.startswith("("):
        title_suffix = f"({title_suffix})"

    generate_trajectory_evaluation_figure(
        gt_path=args.gt,
        est_path=args.est,
        evo_ape_zip_path=args.evo_ape_zip,
        out_path=args.out,
        t_max_diff_s=args.t_max_diff,
        title_suffix=("" if not title_suffix else f" {title_suffix}"),
    )
    return 0


def _load_tum_positions(path: str) -> TumTrajectory:
    data = np.loadtxt(path)
    t = data[:, 0].astype(float)
    p = data[:, 1:4].astype(float)
    return TumTrajectory(t=t, p=p)


def _associate_by_time(t_gt: np.ndarray, t_est: np.ndarray, t_max_diff_s: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Greedy two-pointer association (monotonic timestamps).

    Returns:
        (gt_indices, est_indices) of matched poses.
    """
    gt_idx: List[int] = []
    est_idx: List[int] = []

    i = 0
    j = 0
    n = len(t_gt)
    m = len(t_est)

    while i < n and j < m:
        dt = t_est[j] - t_gt[i]
        if abs(dt) <= t_max_diff_s:
            gt_idx.append(i)
            est_idx.append(j)
            i += 1
            j += 1
            continue
        if dt < -t_max_diff_s:
            j += 1
        else:
            i += 1

    return np.array(gt_idx, dtype=int), np.array(est_idx, dtype=int)


def _load_sim3_and_errors(evo_ape_zip_path: str) -> Tuple[np.ndarray, np.ndarray]:
    with zipfile.ZipFile(evo_ape_zip_path, "r") as zf:
        sim3 = np.load(zf.open("alignment_transformation_sim3.npy"), allow_pickle=False)
        err = np.load(zf.open("error_array.npy"), allow_pickle=False)
    return sim3.astype(float), err.astype(float)


def _apply_sim3(sim3: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    ones = np.ones((xyz.shape[0], 1), dtype=float)
    pts = np.hstack([xyz, ones])  # (N,4)
    out = (sim3 @ pts.T).T  # (N,4)
    return out[:, :3]


if __name__ == "__main__":
    raise SystemExit(main())

