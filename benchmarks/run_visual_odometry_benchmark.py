#!/usr/bin/env python3
"""Run a small repeatable ORB/SIFT visual odometry comparison."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("build/apps/lunar_visual_odometry"))
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--fx", required=True)
    parser.add_argument("--fy", required=True)
    parser.add_argument("--cx", required=True)
    parser.add_argument("--cy", required=True)
    parser.add_argument("--ground-truth", type=Path, help="optional POLAR pose subset TSV")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/benchmarks"))
    parser.add_argument("--clahe", action="store_true", help="enable CLAHE preprocessing")
    parser.add_argument("--clahe-clip-limit", default="2.0")
    parser.add_argument("--clahe-tile-grid-size", default="8")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for feature in ("orb", "sift"):
        trajectory = args.output_dir / f"trajectory_{feature}.tum"
        log = args.output_dir / f"vo_{feature}.csv"
        command = [
            str(args.binary),
            "--images",
            str(args.images),
            "--fx",
            args.fx,
            "--fy",
            args.fy,
            "--cx",
            args.cx,
            "--cy",
            args.cy,
            "--feature",
            feature,
            "--trajectory",
            str(trajectory),
        ]
        if args.clahe:
            command.extend(
                [
                    "--clahe",
                    "--clahe-clip-limit",
                    args.clahe_clip_limit,
                    "--clahe-tile-grid-size",
                    args.clahe_tile_grid_size,
                ]
            )
        with log.open("w", encoding="utf-8") as handle:
            subprocess.run(command, check=True, stdout=handle)
        print(f"{feature}: {trajectory} {log}")

        if args.ground_truth:
            eval_json = args.output_dir / f"eval_{feature}.json"
            aligned_csv = args.output_dir / f"aligned_{feature}.csv"
            subprocess.run(
                [
                    sys.executable,
                    "scripts/evaluate_trajectory.py",
                    "--estimated",
                    str(trajectory),
                    "--ground-truth",
                    str(args.ground_truth),
                    "--vo-log",
                    str(log),
                    "--output-json",
                    str(eval_json),
                    "--aligned-csv",
                    str(aligned_csv),
                ],
                check=True,
            )
            print(f"{feature}: {eval_json} {aligned_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
