#!/usr/bin/env python3
"""Benchmark HYG lost-in-space identification with an angular pair index."""

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
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/hyg_pair_index_benchmark"))
    parser.add_argument("--index-sizes", nargs="+", type=int, default=[500, 1000])
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--stars", type=int, default=8)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    parser.add_argument("--neighbor-bins", type=int, default=2)
    parser.add_argument("--max-generation-attempts", type=int, default=50)
    parser.add_argument("--max-edge-deg", type=float, default=80.0)
    parser.add_argument("--min-star-separation-arcsec", type=float, default=120.0)
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    for index_size in args.index_sizes:
        index_catalog = args.output_dir / f"hyg_brightest_resolved_{index_size}.csv"
        index_path = args.output_dir / f"hyg_pair_index_{index_size}.pkl"
        run(
            [
                sys.executable,
                "scripts/filter_star_catalog.py",
                "--input",
                str(args.catalog),
                "--output",
                str(index_catalog),
                "--limit",
                str(index_size),
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
                str(index_size),
                "--bin-arcsec",
                "120",
                "--max-edge-deg",
                str(args.max_edge_deg),
            ]
        )
        build_seconds = time.perf_counter() - build_start
        index_size_mb = index_path.stat().st_size / (1024.0 * 1024.0)
        index_metadata = json.loads(index_path.with_suffix(".json").read_text(encoding="utf-8"))

        for trial in range(args.trials):
            case_dir = args.output_dir / f"index_{index_size}" / f"trial_{trial:03d}"
            generate_visible_case(
                catalog=index_catalog,
                case_dir=case_dir,
                stars=args.stars,
                noise_px=args.noise_px,
                seed=8000 + index_size * 10 + trial,
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
                    "--seed",
                    str(9000 + trial),
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
            assignment_metadata = json.loads((case_dir / "assignments.json").read_text(encoding="utf-8"))
            rows.append(
                {
                    "index_size": str(index_size),
                    "trial": str(trial),
                    "correct": str(correct),
                    "wrong": str(wrong),
                    "assigned": str(assigned),
                    "total": str(args.stars),
                    "indexed_pairs": str(index_metadata["pairs"]),
                    "triangle_matches": str(assignment_metadata["triangle_matches"]),
                    "build_seconds": f"{build_seconds:.3f}",
                    "query_seconds": f"{query_seconds:.3f}",
                    "index_size_mb": f"{index_size_mb:.3f}",
                    "min_star_separation_arcsec": f"{args.min_star_separation_arcsec:.3f}",
                }
            )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = args.output_dir / "summary.csv"
    fieldnames = [
        "index_size",
        "trial",
        "correct",
        "wrong",
        "assigned",
        "total",
        "indexed_pairs",
        "triangle_matches",
        "build_seconds",
        "query_seconds",
        "index_size_mb",
        "min_star_separation_arcsec",
    ]
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# HYG Pair-Index Lost-In-Space Benchmark",
        "",
        f"Minimum catalog star separation: {args.min_star_separation_arcsec:g} arcsec",
        "",
        "| Indexed stars | Trials | Indexed pairs | Index MB | Build sec | Query sec avg | Correct IDs | Wrong IDs | Assigned IDs |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for index_size in args.index_sizes:
        subset = [row for row in rows if row["index_size"] == str(index_size)]
        correct = sum(int(row["correct"]) for row in subset)
        wrong = sum(int(row["wrong"]) for row in subset)
        assigned = sum(int(row["assigned"]) for row in subset)
        total = sum(int(row["total"]) for row in subset)
        pairs = subset[0]["indexed_pairs"] if subset else "0"
        index_mb = float(subset[0]["index_size_mb"]) if subset else 0.0
        build_seconds = float(subset[0]["build_seconds"]) if subset else 0.0
        query_seconds = sum(float(row["query_seconds"]) for row in subset) / len(subset) if subset else 0.0
        lines.append(
            f"| {index_size} | {len(subset)} | {pairs} | {index_mb:.1f} | {build_seconds:.3f} | {query_seconds:.3f} | {correct}/{total} | {wrong}/{total} | {assigned}/{total} |"
        )
    summary_md = args.output_dir / "summary.md"
    summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {summary_csv}")
    print(f"wrote {summary_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
