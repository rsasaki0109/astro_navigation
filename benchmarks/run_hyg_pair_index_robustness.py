#!/usr/bin/env python3
"""Benchmark HYG pair-index lost-in-space under missing and false detections."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

from run_hyg_ambiguity_benchmark import count_correct, generate_visible_case


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True, help="converted HYG unit-vector catalog")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/hyg_pair_index_robustness"))
    parser.add_argument("--index-size", type=int, default=2000)
    parser.add_argument("--stars", type=int, default=12)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--drop-counts", nargs="+", type=int, default=[0, 2, 4])
    parser.add_argument("--false-counts", nargs="+", type=int, default=[0, 2, 4])
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    parser.add_argument("--neighbor-bins", type=int, default=2)
    parser.add_argument("--max-edge-deg", type=float, default=80.0)
    parser.add_argument("--min-star-separation-arcsec", type=float, default=120.0)
    parser.add_argument("--max-generation-attempts", type=int, default=50)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    index_catalog = args.output_dir / f"hyg_brightest_resolved_{args.index_size}.csv"
    index_path = args.output_dir / f"hyg_pair_index_{args.index_size}.pkl"
    run(
        [
            sys.executable,
            "scripts/filter_star_catalog.py",
            "--input",
            str(args.catalog),
            "--output",
            str(index_catalog),
            "--limit",
            str(args.index_size),
            "--min-separation-arcsec",
            str(args.min_star_separation_arcsec),
        ]
    )
    build_start = time.perf_counter()
    run(
        [
            sys.executable,
            "scripts/build_star_pair_index.py",
            "--catalog",
            str(index_catalog),
            "--output",
            str(index_path),
            "--limit",
            str(args.index_size),
            "--bin-arcsec",
            "120",
            "--max-edge-deg",
            str(args.max_edge_deg),
        ]
    )
    build_seconds = time.perf_counter() - build_start
    index_size_mb = index_path.stat().st_size / (1024.0 * 1024.0)
    index_metadata = json.loads(index_path.with_suffix(".json").read_text(encoding="utf-8"))

    rows: list[dict[str, str]] = []
    for drop_count in args.drop_counts:
        for false_count in args.false_counts:
            for trial in range(args.trials):
                case_dir = args.output_dir / f"drop_{drop_count}_false_{false_count}" / f"trial_{trial:03d}"
                generate_visible_case(
                    catalog=index_catalog,
                    case_dir=case_dir,
                    stars=args.stars,
                    noise_px=args.noise_px,
                    seed=10000 + drop_count * 100 + false_count * 10 + trial,
                    max_attempts=args.max_generation_attempts,
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
                        str(11000 + trial),
                    ]
                )
                query_start = time.perf_counter()
                run(
                    [
                        sys.executable,
                        "scripts/identify_stars_with_pair_index.py",
                        "--observations",
                        str(case_dir / "observations_unlabeled.csv"),
                        "--index",
                        str(index_path),
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
                        str(args.neighbor_bins),
                    ]
                )
                query_seconds = time.perf_counter() - query_start
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
                        "observations": str(metadata["observations"]),
                        "triangle_matches": str(metadata["triangle_matches"]),
                        "verified_hypotheses": str(metadata["verified_hypotheses"]),
                        "query_seconds": f"{query_seconds:.3f}",
                        "indexed_pairs": str(index_metadata["pairs"]),
                        "index_size_mb": f"{index_size_mb:.3f}",
                        "build_seconds": f"{build_seconds:.3f}",
                    }
                )

    summary_csv = args.output_dir / "summary.csv"
    fieldnames = [
        "drop_count",
        "false_count",
        "trial",
        "correct",
        "wrong",
        "assigned",
        "true_remaining",
        "observations",
        "triangle_matches",
        "verified_hypotheses",
        "query_seconds",
        "indexed_pairs",
        "index_size_mb",
        "build_seconds",
    ]
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# HYG Pair-Index Robustness Benchmark",
        "",
        f"Indexed stars: {args.index_size}",
        f"Indexed pairs: {index_metadata['pairs']}",
        f"Index size: {index_size_mb:.1f} MB",
        f"Index build: {build_seconds:.3f} s",
        f"Minimum catalog star separation: {args.min_star_separation_arcsec:g} arcsec",
        "",
        "| Dropped | False | Trials | Correct true IDs | Assigned IDs | Wrong IDs | Query sec avg |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
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
            query_seconds = sum(float(row["query_seconds"]) for row in subset) / len(subset) if subset else 0.0
            lines.append(
                f"| {drop_count} | {false_count} | {len(subset)} | {correct}/{true_remaining} | {assigned}/{true_remaining} | {wrong} | {query_seconds:.3f} |"
            )
    summary_md = args.output_dir / "summary.md"
    summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {summary_csv}")
    print(f"wrote {summary_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
