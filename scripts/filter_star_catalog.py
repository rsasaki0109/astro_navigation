#!/usr/bin/env python3
"""Filter a converted star catalog for benchmarkable star tracker observations."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def normalize(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def angular_distance(lhs: np.ndarray, rhs: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(lhs, rhs), -1.0, 1.0)))


def magnitude(row: dict[str, str]) -> tuple[float, str]:
    try:
        mag = float(row.get("mag", "inf"))
    except ValueError:
        mag = float("inf")
    return mag, row.get("id", "")


def direction(row: dict[str, str]) -> np.ndarray:
    return normalize(np.array([float(row["x"]), float(row["y"]), float(row["z"])], dtype=float))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, help="maximum number of filtered stars")
    parser.add_argument("--min-separation-arcsec", type=float, default=120.0)
    args = parser.parse_args()

    min_separation_rad = math.radians(args.min_separation_arcsec / 3600.0)
    # Closer than `min_separation_rad` ↔ cos(angle) > cos(min_separation_rad).
    # Compare dot products directly to skip per-pair arccos.
    min_separation_cos = math.cos(min_separation_rad)
    with args.input.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{args.input} has no CSV header")
        rows = sorted(reader, key=magnitude)
        fieldnames = reader.fieldnames

    selected_rows: list[dict[str, str]] = []
    rejected_close = 0
    capacity = args.limit if args.limit else len(rows)
    selected_directions = np.empty((capacity, 3), dtype=float)
    selected_count = 0
    for row in rows:
        current_direction = direction(row)
        if selected_count > 0:
            dots = selected_directions[:selected_count] @ current_direction
            if (dots > min_separation_cos).any():
                rejected_close += 1
                continue
        selected_directions[selected_count] = current_direction
        selected_rows.append(row)
        selected_count += 1
        if args.limit and selected_count >= args.limit:
            break

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in selected_rows:
            writer.writerow(row)

    print(
        f"wrote {len(selected_rows)} stars to {args.output}; "
        f"rejected {rejected_close} closer than {args.min_separation_arcsec:g} arcsec"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
