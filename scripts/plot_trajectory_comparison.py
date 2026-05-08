#!/usr/bin/env python3
"""Plot VO trajectories aligned to a ground-truth pose subset on the same axes."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from evaluate_trajectory import (
    apply_similarity,
    load_estimated_positions,
    load_polar_gt_positions,
    umeyama_align,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, required=True, help="POLAR refined_poses.tsv")
    parser.add_argument(
        "--trajectory",
        action="append",
        nargs=3,
        metavar=("LABEL", "PATH", "MODE"),
        required=True,
        help="Add an estimated trajectory: LABEL PATH MODE (MODE: sim3 or se3). Repeatable.",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", type=str, default="POLAR Traverse VO")
    args = parser.parse_args()

    gt = load_polar_gt_positions(args.ground_truth)
    plt.figure(figsize=(8, 6))
    plt.plot(gt[:, 0], gt[:, 1], color="black", linewidth=2.5, label="ground truth", zorder=10)
    plt.scatter(gt[0, 0], gt[0, 1], color="black", s=80, zorder=11, marker="s", label="start")

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    for idx, (label, path_str, mode) in enumerate(args.trajectory):
        path = Path(path_str)
        estimated = load_estimated_positions(path)
        n = min(len(estimated), len(gt))
        estimated = estimated[:n]
        gt_subset = gt[:n]
        scale, rotation, translation = umeyama_align(
            estimated, gt_subset, estimate_scale=(mode == "sim3")
        )
        aligned = apply_similarity(estimated, scale, rotation, translation)
        residuals = np.linalg.norm(aligned - gt_subset, axis=1)
        ate_rmse = float(np.sqrt(np.mean(residuals * residuals)))
        plt.plot(
            aligned[:, 0],
            aligned[:, 1],
            marker="o",
            markersize=5,
            linewidth=1.5,
            color=colors[idx % len(colors)],
            label=f"{label}  ATE RMSE = {ate_rmse:.3f} m",
        )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(args.title)
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.output, dpi=160)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
