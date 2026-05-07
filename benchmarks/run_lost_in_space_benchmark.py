#!/usr/bin/env python3
"""Benchmark triangle-based lost-in-space identification on synthetic cases."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def count_correct(assignments: Path, truth_observations: Path) -> tuple[int, int]:
    truth = {
        int(row["observation_index"]): row["id"]
        for row in csv.DictReader(truth_observations.open(newline="", encoding="utf-8"))
        if row["id"]
    }
    assigned = {
        int(row["observation_index"]): row["id"]
        for row in csv.DictReader(assignments.open(newline="", encoding="utf-8"))
    }
    correct = sum(1 for index, star_id in truth.items() if assigned.get(index) == star_id)
    return correct, len(truth)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/lost_in_space_benchmark"))
    parser.add_argument("--noise-px", nargs="+", type=float, default=[0.0, 0.05, 0.1, 0.2])
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--stars", type=int, default=12)
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    for noise in args.noise_px:
        for trial in range(args.trials):
            case_dir = args.output_dir / f"noise_{noise:g}" / f"trial_{trial:03d}"
            run(
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
                    str(2000 + trial),
                ]
            )
            run(
                [
                    sys.executable,
                    "scripts/drop_star_ids.py",
                    "--input",
                    str(case_dir / "observations.csv"),
                    "--output",
                    str(case_dir / "observations_unlabeled.csv"),
                    "--truth-output",
                    str(case_dir / "observations_unlabeled_truth.csv"),
                ]
            )
            run(
                [
                    sys.executable,
                    "scripts/identify_stars_by_triangles.py",
                    "--observations",
                    str(case_dir / "observations_unlabeled.csv"),
                    "--catalog",
                    str(case_dir / "catalog.csv"),
                    "--output",
                    str(case_dir / "assignments.csv"),
                    "--fx",
                    "1000",
                    "--fy",
                    "1000",
                    "--cx",
                    "512",
                    "--cy",
                    "512",
                    "--tolerance-arcsec",
                    str(args.tolerance_arcsec),
                    "--max-catalog-stars",
                    str(args.stars),
                ]
            )
            correct, total = count_correct(
                case_dir / "assignments.csv",
                case_dir / "observations_unlabeled_truth.csv",
            )
            metadata = json.loads((case_dir / "assignments.json").read_text(encoding="utf-8"))
            rows.append(
                {
                    "noise_px": f"{noise:g}",
                    "trial": str(trial),
                    "correct": str(correct),
                    "total": str(total),
                    "assigned": str(metadata["assigned_observations"]),
                    "triangle_matches": str(metadata["triangle_matches"]),
                }
            )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = args.output_dir / "summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=["noise_px", "trial", "correct", "total", "assigned", "triangle_matches"]
        )
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# Lost-In-Space Triangle Benchmark",
        "",
        "| Noise [px] | Trials | Correct IDs | Assigned IDs |",
        "| ---: | ---: | ---: | ---: |",
    ]
    for noise in args.noise_px:
        subset = [row for row in rows if row["noise_px"] == f"{noise:g}"]
        correct = sum(int(row["correct"]) for row in subset)
        total = sum(int(row["total"]) for row in subset)
        assigned = sum(int(row["assigned"]) for row in subset)
        lines.append(f"| {noise:g} | {len(subset)} | {correct}/{total} | {assigned}/{total} |")
    (args.output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {summary_csv}")
    print(f"wrote {args.output_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
