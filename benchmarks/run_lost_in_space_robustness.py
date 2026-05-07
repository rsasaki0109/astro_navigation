#!/usr/bin/env python3
"""Benchmark lost-in-space triangle index under missing and false detections."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def count_correct(assignments: Path, truth_observations: Path) -> tuple[int, int, int]:
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
    wrong = sum(1 for index, star_id in assigned.items() if index not in truth or star_id != truth[index])
    return correct, wrong, len(assigned)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/lost_in_space_robustness"))
    parser.add_argument("--stars", type=int, default=16)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--drop-counts", nargs="+", type=int, default=[0, 2, 4])
    parser.add_argument("--false-counts", nargs="+", type=int, default=[0, 2, 4])
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    for drop_count in args.drop_counts:
        for false_count in args.false_counts:
            for trial in range(args.trials):
                case_dir = args.output_dir / f"drop_{drop_count}_false_{false_count}" / f"trial_{trial:03d}"
                run(
                    [
                        sys.executable,
                        "scripts/generate_star_tracker_case.py",
                        "--output-dir",
                        str(case_dir),
                        "--noise-px",
                        str(args.noise_px),
                        "--stars",
                        str(args.stars),
                        "--seed",
                        str(3000 + trial),
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
                        "--drop-count",
                        str(drop_count),
                        "--false-count",
                        str(false_count),
                        "--seed",
                        str(4000 + trial),
                    ]
                )
                run(
                    [
                        sys.executable,
                        "scripts/build_star_triangle_index.py",
                        "--catalog",
                        str(case_dir / "catalog.csv"),
                        "--output",
                        str(case_dir / "triangle_index.pkl"),
                        "--limit",
                        str(args.stars),
                        "--bin-arcsec",
                        "120",
                        "--max-edge-deg",
                        "80",
                    ]
                )
                run(
                    [
                        sys.executable,
                        "scripts/identify_stars_with_index.py",
                        "--observations",
                        str(case_dir / "observations_unlabeled.csv"),
                        "--index",
                        str(case_dir / "triangle_index.pkl"),
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
                        "--neighbor-bins",
                        "2",
                    ]
                )
                correct, wrong, assigned = count_correct(
                    case_dir / "assignments.csv",
                    case_dir / "observations_unlabeled_truth.csv",
                )
                metadata = json.loads((case_dir / "assignments.json").read_text(encoding="utf-8"))
                rows.append(
                    {
                        "drop_count": str(drop_count),
                        "false_count": str(false_count),
                        "trial": str(trial),
                        "correct": str(correct),
                        "wrong": str(wrong),
                        "assigned": str(assigned),
                        "true_remaining": str(args.stars - min(drop_count, args.stars)),
                        "triangle_matches": str(metadata["triangle_matches"]),
                    }
                )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = args.output_dir / "summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "drop_count",
                "false_count",
                "trial",
                "correct",
                "wrong",
                "assigned",
                "true_remaining",
                "triangle_matches",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# Lost-In-Space Robustness Benchmark",
        "",
        "| Dropped | False | Trials | Correct true IDs | Assigned IDs |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ]
    for drop_count in args.drop_counts:
        for false_count in args.false_counts:
            subset = [
                row
                for row in rows
                if row["drop_count"] == str(drop_count) and row["false_count"] == str(false_count)
            ]
            correct = sum(int(row["correct"]) for row in subset)
            wrong = sum(int(row["wrong"]) for row in subset)
            assigned = sum(int(row["assigned"]) for row in subset)
            true_remaining = sum(int(row["true_remaining"]) for row in subset)
            lines.append(
                f"| {drop_count} | {false_count} | {len(subset)} | {correct}/{true_remaining} | {assigned}/{true_remaining} ({wrong} wrong) |"
            )
    (args.output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {summary_csv}")
    print(f"wrote {args.output_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
