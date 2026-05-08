#!/usr/bin/env python3
"""Run synthetic star tracker attitude benchmarks."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def attitude_error_deg(q_est: np.ndarray, q_true: np.ndarray) -> float:
    q_est = normalize_quaternion(q_est)
    q_true = normalize_quaternion(q_true)
    dot = abs(float(np.dot(q_est, q_true)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def run_case(binary: Path, case_dir: Path, truth: dict) -> dict[str, str]:
    intrinsics = truth["intrinsics"]
    result = subprocess.run(
        [
            str(binary),
            "--observations",
            str(case_dir / "observations.csv"),
            "--catalog",
            str(case_dir / "catalog.csv"),
            "--fx",
            str(intrinsics["fx"]),
            "--fy",
            str(intrinsics["fy"]),
            "--cx",
            str(intrinsics["cx"]),
            "--cy",
            str(intrinsics["cy"]),
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    rows = list(csv.DictReader(result.stdout.splitlines()))
    if not rows:
        raise RuntimeError(f"no star tracker output for {case_dir}")
    row = rows[0]
    q_est = np.array([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])])
    q_true = np.array(truth["q_camera_inertial_xyzw"], dtype=float)
    row["attitude_error_deg"] = f"{attitude_error_deg(q_est, q_true):.9f}"
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("build/apps/star_tracker_attitude"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/star_tracker_benchmark"))
    parser.add_argument("--noise-px", nargs="+", type=float, default=[0.0, 0.1, 0.5, 1.0])
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--stars", type=int, default=30)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, str]] = []
    for noise in args.noise_px:
        for trial in range(args.trials):
            case_dir = args.output_dir / f"noise_{noise:g}" / f"trial_{trial:03d}"
            subprocess.run(
                [
                    sys.executable,
                    "scripts/generate_star_tracker_case.py",
                    "--output-dir",
                    str(case_dir),
                    "--noise-px",
                    str(noise),
                    "--stars",
                    str(args.stars),
                    "--seed",
                    str(1000 + trial),
                ],
                check=True,
            )
            truth = json.loads((case_dir / "truth.json").read_text(encoding="utf-8"))
            row = run_case(args.binary, case_dir, truth)
            row["noise_px"] = f"{noise:g}"
            row["trial"] = str(trial)
            rows.append(row)

    summary_csv = args.output_dir / "summary.csv"
    fieldnames = [
        "noise_px",
        "trial",
        "success",
        "correspondences",
        "rms_direction_error_rad",
        "attitude_error_deg",
        "qx",
        "qy",
        "qz",
        "qw",
        "status",
    ]
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary_md = args.output_dir / "summary.md"
    lines = [
        "# Star Tracker Benchmark",
        "",
        "| Noise [px] | Trials | Mean attitude error [deg] | Max attitude error [deg] | Mean RMS bearing error [rad] |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ]
    for noise in args.noise_px:
        subset = [row for row in rows if row["noise_px"] == f"{noise:g}"]
        attitude_errors = [float(row["attitude_error_deg"]) for row in subset]
        bearing_errors = [float(row["rms_direction_error_rad"]) for row in subset]
        lines.append(
            f"| {noise:g} | {len(subset)} | {np.mean(attitude_errors):.9f} | "
            f"{np.max(attitude_errors):.9f} | {np.mean(bearing_errors):.9f} |"
        )
    summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"wrote {summary_csv}")
    print(f"wrote {summary_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

