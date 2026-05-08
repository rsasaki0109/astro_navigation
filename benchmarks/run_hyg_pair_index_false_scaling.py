#!/usr/bin/env python3
"""Benchmark HYG pair-index lost-in-space as false detections increase."""

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
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/hyg_pair_index_false_scaling"))
    parser.add_argument("--index-size", type=int, default=2000)
    parser.add_argument("--stars", type=int, default=32)
    parser.add_argument("--false-counts", nargs="+", type=int, default=[0, 4, 8, 12])
    parser.add_argument("--trials", type=int, default=2)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    parser.add_argument("--neighbor-bins", type=int, default=2)
    parser.add_argument("--max-edge-deg", type=float, default=80.0)
    parser.add_argument("--min-star-separation-arcsec", type=float, default=120.0)
    parser.add_argument("--max-generation-attempts", type=int, default=80)
    parser.add_argument(
        "--pyramid-size",
        type=int,
        default=0,
        help="Pyramid mode: pass through to the identifier. 0 disables it.",
    )
    parser.add_argument(
        "--skip-pkl",
        action="store_true",
        help="Skip the pickle dict-of-tuples emission in the build step. Required for very "
        "large indices (40000+ stars) where the pickle path exhausts memory.",
    )
    parser.add_argument(
        "--limiting-magnitude",
        type=float,
        default=None,
        help="Forward to generate_star_tracker_observations_from_catalog.py for probabilistic "
        "magnitude-aware star detection. Default disabled (top-3N + uniform pick).",
    )
    parser.add_argument(
        "--mag-softness",
        type=float,
        default=0.5,
        help="Sigmoid width for --limiting-magnitude.",
    )
    parser.add_argument(
        "--false-near-fraction",
        type=float,
        default=0.0,
        help="Forward to drop_star_ids.py: fraction of false detections placed near a real star.",
    )
    parser.add_argument(
        "--false-near-sigma-px",
        type=float,
        default=20.0,
        help="Forward to drop_star_ids.py: Gaussian std-dev for near-star false detections.",
    )
    parser.add_argument(
        "--pyramid-restarts",
        type=int,
        default=0,
        help="Forward to identify_stars_with_pair_index.py: number of additional pyramid attempts "
        "with shuffled observations when assignments fall below --confidence-fraction.",
    )
    parser.add_argument(
        "--confidence-fraction",
        type=float,
        default=0.5,
        help="Forward to identify_stars_with_pair_index.py: restart-pyramid threshold.",
    )
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
    build_command = [
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
    if args.skip_pkl:
        build_command.append("--skip-pkl")
    build_start = time.perf_counter()
    run(build_command)
    build_seconds = time.perf_counter() - build_start
    sized_path = index_path.with_suffix(".npz") if args.skip_pkl else index_path
    index_size_mb = sized_path.stat().st_size / (1024.0 * 1024.0)
    index_metadata = json.loads(index_path.with_suffix(".json").read_text(encoding="utf-8"))

    rows: list[dict[str, str]] = []
    for false_count in args.false_counts:
        for trial in range(args.trials):
            case_dir = args.output_dir / f"false_{false_count}" / f"trial_{trial:03d}"
            generate_visible_case(
                catalog=index_catalog,
                case_dir=case_dir,
                stars=args.stars,
                noise_px=args.noise_px,
                seed=14000 + false_count * 100 + trial,
                max_attempts=args.max_generation_attempts,
                limiting_magnitude=args.limiting_magnitude,
                mag_softness=args.mag_softness,
            )
            drop_command = [
                sys.executable,
                "scripts/drop_star_ids.py",
                "--input",
                str(case_dir / "observations.csv"),
                "--output",
                str(case_dir / "observations_unlabeled.csv"),
                "--truth-output",
                str(case_dir / "observations_unlabeled_truth.csv"),
                "--false-count",
                str(false_count),
                "--seed",
                str(15000 + trial),
                "--false-near-fraction",
                f"{args.false_near_fraction:.6f}",
                "--false-near-sigma-px",
                f"{args.false_near_sigma_px:.6f}",
            ]
            run(drop_command)
            identify_command = [
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
            if args.pyramid_size > 0:
                identify_command.extend(["--pyramid-size", str(args.pyramid_size)])
            if args.pyramid_restarts > 0:
                identify_command.extend(
                    [
                        "--pyramid-restarts",
                        str(args.pyramid_restarts),
                        "--confidence-fraction",
                        f"{args.confidence_fraction:.6f}",
                        "--pyramid-restart-seed",
                        str(16000 + trial),
                    ]
                )
            query_start = time.perf_counter()
            run(identify_command)
            query_seconds = time.perf_counter() - query_start
            correct, wrong, assigned = count_correct(
                case_dir / "assignments.csv",
                case_dir / "observations_unlabeled_truth.csv",
            )
            metadata = json.loads((case_dir / "assignments.json").read_text(encoding="utf-8"))
            rows.append(
                {
                    "false_count": str(false_count),
                    "trial": str(trial),
                    "correct": str(correct),
                    "wrong": str(wrong),
                    "assigned": str(assigned),
                    "total": str(args.stars),
                    "observations": str(metadata["observations"]),
                    "triangle_matches": str(metadata["triangle_matches"]),
                    "candidate_hypotheses": str(metadata["candidate_hypotheses"]),
                    "verified_hypotheses": str(metadata["verified_hypotheses"]),
                    "query_seconds": f"{query_seconds:.3f}",
                    "candidate_generation_seconds": f"{metadata.get('candidate_generation_seconds', 0.0):.3f}",
                    "pruning_seconds": f"{metadata.get('pruning_seconds', 0.0):.3f}",
                    "verification_seconds": f"{metadata.get('verification_seconds', 0.0):.3f}",
                    "indexed_pairs": str(index_metadata["pairs"]),
                    "index_size_mb": f"{index_size_mb:.3f}",
                    "build_seconds": f"{build_seconds:.3f}",
                }
            )

    summary_csv = args.output_dir / "summary.csv"
    fieldnames = [
        "false_count",
        "trial",
        "correct",
        "wrong",
        "assigned",
        "total",
        "observations",
        "triangle_matches",
        "candidate_hypotheses",
        "verified_hypotheses",
        "query_seconds",
        "candidate_generation_seconds",
        "pruning_seconds",
        "verification_seconds",
        "indexed_pairs",
        "index_size_mb",
        "build_seconds",
    ]
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# HYG Pair-Index False Detection Scaling Benchmark",
        "",
        f"Indexed stars: {args.index_size}",
        f"Generated true stars: {args.stars}",
        f"Indexed pairs: {index_metadata['pairs']}",
        f"Index size: {index_size_mb:.1f} MB",
        f"Index build: {build_seconds:.3f} s",
        f"Minimum catalog star separation: {args.min_star_separation_arcsec:g} arcsec",
        "",
        "| False detections | Trials | Correct true IDs | Wrong IDs | Assigned IDs | Candidates avg | Verified avg | Query sec avg | Cand gen avg | Verify avg |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for false_count in args.false_counts:
        subset = [row for row in rows if row["false_count"] == str(false_count)]
        correct = sum(int(row["correct"]) for row in subset)
        wrong = sum(int(row["wrong"]) for row in subset)
        assigned = sum(int(row["assigned"]) for row in subset)
        total = sum(int(row["total"]) for row in subset)
        candidates = sum(int(row["candidate_hypotheses"]) for row in subset) / len(subset) if subset else 0.0
        verified = sum(int(row["verified_hypotheses"]) for row in subset) / len(subset) if subset else 0.0
        query_seconds = sum(float(row["query_seconds"]) for row in subset) / len(subset) if subset else 0.0
        candidate_gen_seconds = (
            sum(float(row["candidate_generation_seconds"]) for row in subset) / len(subset) if subset else 0.0
        )
        verify_seconds = (
            sum(float(row["verification_seconds"]) for row in subset) / len(subset) if subset else 0.0
        )
        lines.append(
            f"| {false_count} | {len(subset)} | {correct}/{total} | {wrong} | {assigned}/{total} | {candidates:.1f} | {verified:.1f} | {query_seconds:.3f} | {candidate_gen_seconds:.3f} | {verify_seconds:.3f} |"
        )
    summary_md = args.output_dir / "summary.md"
    summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {summary_csv}")
    print(f"wrote {summary_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
