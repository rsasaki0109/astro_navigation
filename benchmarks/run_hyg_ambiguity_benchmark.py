#!/usr/bin/env python3
"""Benchmark lost-in-space ambiguity as HYG catalog index size increases."""

from __future__ import annotations

import argparse
import csv
import json
import random
import subprocess
import sys
import time
from pathlib import Path


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def try_run(command: list[str]) -> bool:
    result = subprocess.run(command, check=False, stderr=subprocess.PIPE, text=True)
    return result.returncode == 0


def write_catalog_subset(catalog: Path, output: Path, limit: int) -> int:
    def magnitude(row: dict[str, str]) -> tuple[float, str]:
        try:
            mag = float(row.get("mag", "inf"))
        except ValueError:
            mag = float("inf")
        return mag, row.get("id", "")

    output.parent.mkdir(parents=True, exist_ok=True)
    with catalog.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{catalog} has no CSV header")
        rows = sorted(reader, key=magnitude)
    selected_rows = rows[:limit]
    if len(selected_rows) < limit:
        raise ValueError(f"{catalog} only contains {len(selected_rows)} rows; requested {limit}")

    with output.open("w", newline="", encoding="utf-8") as out:
        writer = csv.DictWriter(out, fieldnames=reader.fieldnames)
        writer.writeheader()
        for row in selected_rows:
            writer.writerow(row)
    return len(selected_rows)


def generate_visible_case(
    *,
    catalog: Path,
    case_dir: Path,
    stars: int,
    noise_px: float,
    seed: int,
    max_attempts: int,
    limiting_magnitude: float | None = None,
    mag_softness: float = 0.5,
) -> None:
    rng = random.Random(seed)
    for attempt in range(max_attempts):
        yaw = rng.uniform(0.0, 360.0)
        pitch = rng.uniform(-70.0, 70.0)
        roll = rng.uniform(-30.0, 30.0)
        command = [
            sys.executable,
            "scripts/generate_star_tracker_observations_from_catalog.py",
            "--catalog",
            str(catalog),
            "--output-dir",
            str(case_dir),
            "--stars",
            str(stars),
            "--noise-px",
            str(noise_px),
            "--seed",
            str(seed + attempt),
            "--yaw-deg",
            f"{yaw:.9f}",
            "--pitch-deg",
            f"{pitch:.9f}",
            "--roll-deg",
            f"{roll:.9f}",
        ]
        if limiting_magnitude is not None:
            command.extend(
                [
                    "--limiting-magnitude",
                    f"{limiting_magnitude:.6f}",
                    "--mag-softness",
                    f"{mag_softness:.6f}",
                ]
            )
        if try_run(command):
            return
    raise RuntimeError(f"could not generate {stars} visible stars from {catalog} after {max_attempts} attempts")


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
    wrong = sum(1 for index, star_id in assigned.items() if index in truth and star_id != truth[index])
    return correct, wrong, len(assigned)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True, help="converted HYG unit-vector catalog")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/hyg_ambiguity_benchmark"))
    parser.add_argument("--index-sizes", nargs="+", type=int, default=[120, 180, 240])
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--stars", type=int, default=12)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    parser.add_argument("--neighbor-bins", type=int, default=2)
    parser.add_argument("--max-generation-attempts", type=int, default=50)
    parser.add_argument("--max-edge-deg", type=float, default=80.0)
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    for index_size in args.index_sizes:
        index_catalog = args.output_dir / f"hyg_brightest_{index_size}.csv"
        index_path = args.output_dir / f"hyg_index_{index_size}.pkl"
        write_catalog_subset(args.catalog, index_catalog, index_size)
        build_start = time.perf_counter()
        run(
            [
                sys.executable,
                "scripts/build_star_triangle_index.py",
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
                seed=5000 + index_size * 10 + trial,
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
                    str(6000 + trial),
                ]
            )
            query_start = time.perf_counter()
            run(
                [
                    sys.executable,
                    "scripts/identify_stars_with_index.py",
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
                    "indexed_triangles": str(index_metadata["triangles"]),
                    "triangle_matches": str(assignment_metadata["triangle_matches"]),
                    "build_seconds": f"{build_seconds:.3f}",
                    "query_seconds": f"{query_seconds:.3f}",
                    "index_size_mb": f"{index_size_mb:.3f}",
                }
            )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = args.output_dir / "summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "index_size",
                "trial",
                "correct",
                "wrong",
                "assigned",
                "total",
                "indexed_triangles",
                "triangle_matches",
                "build_seconds",
                "query_seconds",
                "index_size_mb",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# HYG Lost-In-Space Ambiguity Benchmark",
        "",
        "| Indexed stars | Trials | Indexed triangles | Index MB | Build sec | Query sec avg | Correct IDs | Wrong IDs | Assigned IDs |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for index_size in args.index_sizes:
        subset = [row for row in rows if row["index_size"] == str(index_size)]
        correct = sum(int(row["correct"]) for row in subset)
        wrong = sum(int(row["wrong"]) for row in subset)
        assigned = sum(int(row["assigned"]) for row in subset)
        total = sum(int(row["total"]) for row in subset)
        triangles = subset[0]["indexed_triangles"] if subset else "0"
        index_mb = float(subset[0]["index_size_mb"]) if subset else 0.0
        build_seconds = float(subset[0]["build_seconds"]) if subset else 0.0
        query_seconds = sum(float(row["query_seconds"]) for row in subset) / len(subset) if subset else 0.0
        lines.append(
            f"| {index_size} | {len(subset)} | {triangles} | {index_mb:.1f} | {build_seconds:.1f} | {query_seconds:.3f} | {correct}/{total} | {wrong}/{total} | {assigned}/{total} |"
        )
    (args.output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {summary_csv}")
    print(f"wrote {args.output_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
