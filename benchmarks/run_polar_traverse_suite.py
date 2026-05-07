#!/usr/bin/env python3
"""Run repeatable POLAR Traverse mono/stereo benchmarks across traverses."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path


def run(command: list[str], *, stdout: Path | None = None) -> None:
    if stdout:
        stdout.parent.mkdir(parents=True, exist_ok=True)
        with stdout.open("w", encoding="utf-8") as handle:
            subprocess.run(command, check=True, stdout=handle)
    else:
        subprocess.run(command, check=True)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def vo_status(eval_json: dict) -> tuple[int, int, int]:
    status = eval_json.get("vo_status", {})
    return (
        int(status.get("frames", 0)),
        int(status.get("ok", 0)) + int(status.get("initialized", 0)),
        int(status.get("failed_motion", 0)),
    )


def append_summary_row(rows: list[dict[str, str]], sequence: str, method: str, alignment: str, eval_path: Path) -> None:
    metrics = load_json(eval_path)
    frames, ok_frames, failed_motion = vo_status(metrics)
    rows.append(
        {
            "sequence": sequence,
            "method": method,
            "alignment": alignment,
            "frames": str(frames),
            "ok_or_initialized": str(ok_frames),
            "failed_motion": str(failed_motion),
            "ate_rmse_m": f"{metrics['ate_rmse_m']:.6f}",
            "rpe_translation_rmse_m": f"{metrics['rpe_translation_rmse_m']:.6f}",
            "path_length_gt_m": f"{metrics['path_length_gt_m']:.6f}",
            "path_length_estimated_m": f"{metrics['path_length_aligned_m']:.6f}",
            "eval_json": str(eval_path),
        }
    )


def rectified_stereo_command(prepared: Path, metadata: dict, *, clahe: bool) -> list[str]:
    rectified = metadata["rectified"]
    left = rectified["left"]
    right = rectified["right"]
    command = [
        "build/apps/stereo_visual_odometry",
        "--pairs",
        str(rectified["stereo_pairs_rectified"]),
        "--fx",
        str(left["fx"]),
        "--fy",
        str(left["fy"]),
        "--cx",
        str(left["cx"]),
        "--cy",
        str(left["cy"]),
        "--right-fx",
        str(right["fx"]),
        "--right-fy",
        str(right["fy"]),
        "--right-cx",
        str(right["cx"]),
        "--right-cy",
        str(right["cy"]),
        "--baseline",
        str(rectified["baseline_m"]),
        "--max-stereo-y-diff",
        "10",
        "--min-disparity",
        "2",
        "--trajectory",
        str(prepared / "trajectory_stereo_pnp_rectified.tum"),
    ]
    if clahe:
        command.extend(["--clahe", "--clahe-clip-limit", "2.0", "--clahe-tile-grid-size", "8"])
    return command


def write_summary_csv(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "sequence",
        "method",
        "alignment",
        "frames",
        "ok_or_initialized",
        "failed_motion",
        "ate_rmse_m",
        "rpe_translation_rmse_m",
        "path_length_gt_m",
        "path_length_estimated_m",
        "eval_json",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_md(path: Path, rows: list[dict[str, str]]) -> None:
    lines = [
        "# POLAR Traverse Suite Summary",
        "",
        "| Sequence | Method | Alignment | Frames OK | ATE RMSE [m] | RPE trans RMSE [m] | Path est / GT [m] |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {sequence} | {method} | {alignment} | {ok_or_initialized}/{frames} | "
            "{ate_rmse_m} | {rpe_translation_rmse_m} | {path_length_estimated_m} / {path_length_gt_m} |".format(
                **row
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("datasets/polar-traverse-view1/extracted"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/polar_view1_suite"))
    parser.add_argument("--binary", type=Path, default=Path("build/apps/lunar_visual_odometry"))
    parser.add_argument("--traverses", nargs="+", default=[f"Traverse{index}" for index in range(1, 7)])
    parser.add_argument("--exposure-ms", type=int, default=50)
    parser.add_argument(
        "--exposures-ms",
        nargs="+",
        type=int,
        help="run multiple exposure times; overrides --exposure-ms",
    )
    parser.add_argument("--skip-mono", action="store_true")
    parser.add_argument("--skip-stereo", action="store_true")
    parser.add_argument("--clahe", action="store_true", help="enable CLAHE preprocessing in VO apps")
    args = parser.parse_args()

    exposures = args.exposures_ms if args.exposures_ms else [args.exposure_ms]
    rows: list[dict[str, str]] = []
    for exposure_ms in exposures:
        for traverse in args.traverses:
            sequence = f"View1_{traverse}_L_{exposure_ms}ms"
            prepared = args.output_dir / f"{traverse}_{exposure_ms}ms"
            print(f"== {sequence} ==")

            run(
                [
                    sys.executable,
                    "scripts/prepare_polar_traverse.py",
                    "--root",
                    str(args.root),
                    "--camera",
                    "L",
                    "--exposure-ms",
                    str(exposure_ms),
                    "--traverse-filter",
                    traverse,
                    "--output",
                    str(prepared),
                    "--rectify-stereo",
                ]
            )

            metadata = load_json(prepared / "metadata.json")
            intrinsics = metadata["intrinsics"]
            pose_subset = Path(metadata["pose_subset"])

            if not args.skip_mono:
                mono_dir = prepared / "mono_benchmarks"
                run(
                    [
                        sys.executable,
                        "benchmarks/run_visual_odometry_benchmark.py",
                        "--binary",
                        str(args.binary),
                        "--images",
                        str(prepared / "images.txt"),
                        "--fx",
                        str(intrinsics["fx"]),
                        "--fy",
                        str(intrinsics["fy"]),
                        "--cx",
                        str(intrinsics["cx"]),
                        "--cy",
                        str(intrinsics["cy"]),
                        "--ground-truth",
                        str(pose_subset),
                        "--output-dir",
                        str(mono_dir),
                    ]
                    + (["--clahe"] if args.clahe else [])
                )
                append_summary_row(rows, sequence, "ORB essential", "sim3", mono_dir / "eval_orb.json")
                append_summary_row(rows, sequence, "SIFT essential", "sim3", mono_dir / "eval_sift.json")

            if not args.skip_stereo:
                stereo_log = prepared / "stereo_pnp_rectified_log.csv"
                run(rectified_stereo_command(prepared, metadata, clahe=args.clahe), stdout=stereo_log)
                stereo_eval = prepared / "eval_stereo_pnp_rectified_se3.json"
                run(
                    [
                        sys.executable,
                        "scripts/evaluate_trajectory.py",
                        "--estimated",
                        str(prepared / "trajectory_stereo_pnp_rectified.tum"),
                        "--ground-truth",
                        str(pose_subset),
                        "--vo-log",
                        str(stereo_log),
                        "--alignment",
                        "se3",
                        "--output-json",
                        str(stereo_eval),
                        "--aligned-csv",
                        str(prepared / "aligned_stereo_pnp_rectified_se3.csv"),
                    ]
                )
                append_summary_row(rows, sequence, "ORB rectified stereo PnP", "se3", stereo_eval)

            write_summary_csv(args.output_dir / "summary.csv", rows)
            write_summary_md(args.output_dir / "summary.md", rows)

    print(f"wrote {args.output_dir / 'summary.csv'}")
    print(f"wrote {args.output_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
