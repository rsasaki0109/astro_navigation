#!/usr/bin/env python3
"""Evaluate a VO trajectory against POLAR Traverse pose rows."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def load_estimated_positions(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    if path.suffix == ".csv":
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                rows.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])
    else:
        with path.open(encoding="utf-8") as handle:
            for line in handle:
                if not line.strip() or line.startswith("#"):
                    continue
                fields = line.split()
                if len(fields) < 4:
                    continue
                rows.append([float(fields[1]), float(fields[2]), float(fields[3])])
    if not rows:
        raise RuntimeError(f"no estimated positions found in {path}")
    return np.asarray(rows, dtype=float)


def load_polar_gt_positions(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open(encoding="utf-8") as handle:
        reader = csv.DictReader(handle, delimiter="\t")
        for row in reader:
            rows.append([float(row["X"]), float(row["Y"]), float(row["Z"])])
    if not rows:
        raise RuntimeError(f"no ground-truth positions found in {path}")
    return np.asarray(rows, dtype=float)


def load_vo_status(path: Path | None) -> dict[str, int]:
    if path is None:
        return {}
    counts = {"frames": 0, "initialized": 0, "ok": 0, "failed_motion": 0}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            counts["frames"] += 1
            status = row.get("status", "")
            if status == "initialized":
                counts["initialized"] += 1
            elif status == "ok":
                counts["ok"] += 1
            else:
                counts["failed_motion"] += 1
    return counts


def umeyama_align(source: np.ndarray, target: np.ndarray, estimate_scale: bool) -> tuple[float, np.ndarray, np.ndarray]:
    if source.shape != target.shape:
        raise ValueError("source and target must have the same shape")
    if source.shape[0] < 3:
        raise ValueError("at least three positions are required for Sim(3) alignment")

    mean_source = source.mean(axis=0)
    mean_target = target.mean(axis=0)
    centered_source = source - mean_source
    centered_target = target - mean_target

    covariance = centered_target.T @ centered_source / source.shape[0]
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0.0:
        correction[2, 2] = -1.0

    rotation = u @ correction @ vt
    source_variance = np.mean(np.sum(centered_source * centered_source, axis=1))
    scale = 1.0
    if estimate_scale:
        scale = float(np.trace(np.diag(singular_values) @ correction) / source_variance)
    translation = mean_target - scale * rotation @ mean_source
    return scale, rotation, translation


def apply_similarity(points: np.ndarray, scale: float, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (scale * (rotation @ points.T)).T + translation


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values)))


def relative_metrics(aligned: np.ndarray, gt: np.ndarray) -> dict[str, float | int]:
    est_delta = np.diff(aligned, axis=0)
    gt_delta = np.diff(gt, axis=0)
    est_norm = np.linalg.norm(est_delta, axis=1)
    gt_norm = np.linalg.norm(gt_delta, axis=1)
    valid = (est_norm > 1e-9) & (gt_norm > 1e-9)

    translation_error = np.linalg.norm(est_delta - gt_delta, axis=1)
    metrics: dict[str, float | int] = {
        "rpe_translation_rmse_m": rmse(translation_error),
        "rpe_translation_mean_m": float(np.mean(translation_error)),
        "path_length_gt_m": float(np.sum(gt_norm)),
        "path_length_aligned_m": float(np.sum(est_norm)),
    }

    if np.any(valid):
        cosines = np.sum(est_delta[valid] * gt_delta[valid], axis=1) / (est_norm[valid] * gt_norm[valid])
        cosines = np.clip(cosines, -1.0, 1.0)
        angles_deg = np.degrees(np.arccos(cosines))
        metrics["relative_direction_mean_deg"] = float(np.mean(angles_deg))
        metrics["relative_direction_max_deg"] = float(np.max(angles_deg))
        metrics["relative_direction_pairs"] = int(angles_deg.size)
    else:
        metrics["relative_direction_mean_deg"] = math.nan
        metrics["relative_direction_max_deg"] = math.nan
        metrics["relative_direction_pairs"] = 0
    return metrics


def write_aligned_csv(path: Path, estimated: np.ndarray, gt: np.ndarray, aligned: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "est_x", "est_y", "est_z", "aligned_x", "aligned_y", "aligned_z", "gt_x", "gt_y", "gt_z"])
        for index, (est, ali, truth) in enumerate(zip(estimated, aligned, gt, strict=True)):
            writer.writerow([index, *est.tolist(), *ali.tolist(), *truth.tolist()])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimated", type=Path, required=True, help="estimated TUM or CSV trajectory")
    parser.add_argument("--ground-truth", type=Path, required=True, help="POLAR pose subset TSV")
    parser.add_argument("--vo-log", type=Path, help="optional lunar_visual_odometry stdout CSV log")
    parser.add_argument(
        "--alignment",
        choices=["sim3", "se3"],
        default="sim3",
        help="use sim3 for monocular trajectories and se3 for metric stereo/depth trajectories",
    )
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--aligned-csv", type=Path)
    args = parser.parse_args()

    estimated = load_estimated_positions(args.estimated)
    gt = load_polar_gt_positions(args.ground_truth)
    count = min(len(estimated), len(gt))
    if len(estimated) != len(gt):
        print(f"warning: using first {count} poses; estimated={len(estimated)} gt={len(gt)}")
    estimated = estimated[:count]
    gt = gt[:count]

    scale, rotation, translation = umeyama_align(estimated, gt, estimate_scale=args.alignment == "sim3")
    aligned = apply_similarity(estimated, scale, rotation, translation)
    ate = np.linalg.norm(aligned - gt, axis=1)

    metrics: dict[str, object] = {
        "estimated": str(args.estimated),
        "ground_truth": str(args.ground_truth),
        "pose_count": int(count),
        "alignment": {
            "type": f"{args.alignment}_umeyama",
            "scale": scale,
            "rotation": rotation.tolist(),
            "translation": translation.tolist(),
        },
        "ate_rmse_m": rmse(ate),
        "ate_mean_m": float(np.mean(ate)),
        "ate_median_m": float(np.median(ate)),
        "ate_max_m": float(np.max(ate)),
        "vo_status": load_vo_status(args.vo_log),
    }
    metrics.update(relative_metrics(aligned, gt))

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    if args.aligned_csv:
        write_aligned_csv(args.aligned_csv, estimated, gt, aligned)

    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
