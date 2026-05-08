#!/usr/bin/env python3
"""Build an angular triangle index for star catalog identification."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import pickle
from pathlib import Path

import numpy as np


def normalize(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def load_catalog(path: Path, limit: int | None) -> list[tuple[str, np.ndarray, float]]:
    stars: list[tuple[str, np.ndarray, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                magnitude = float(row.get("mag", "0.0"))
            except ValueError:
                magnitude = 0.0
            stars.append(
                (
                    row["id"],
                    normalize(np.array([float(row["x"]), float(row["y"]), float(row["z"])])),
                    magnitude,
                )
            )
            if limit and len(stars) >= limit:
                break
    return stars


def angular_distance(lhs: np.ndarray, rhs: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(lhs, rhs), -1.0, 1.0)))


def edge_bins(edges: tuple[float, float, float], bin_size_rad: float) -> tuple[int, int, int]:
    return tuple(int(round(edge / bin_size_rad)) for edge in sorted(edges))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=120)
    parser.add_argument("--bin-arcsec", type=float, default=120.0)
    parser.add_argument("--min-edge-deg", type=float, default=0.2)
    parser.add_argument("--max-edge-deg", type=float, default=30.0)
    args = parser.parse_args()

    stars = load_catalog(args.catalog, args.limit)
    vectors = [star[1] for star in stars]
    bin_size_rad = math.radians(args.bin_arcsec / 3600.0)
    min_edge_rad = math.radians(args.min_edge_deg)
    max_edge_rad = math.radians(args.max_edge_deg)

    index: dict[tuple[int, int, int], list[tuple[int, int, int]]] = {}
    triangle_count = 0
    for indices in itertools.combinations(range(len(vectors)), 3):
        a, b, c = indices
        edges = (
            angular_distance(vectors[a], vectors[b]),
            angular_distance(vectors[a], vectors[c]),
            angular_distance(vectors[b], vectors[c]),
        )
        if min(edges) < min_edge_rad or max(edges) > max_edge_rad:
            continue
        key = edge_bins(edges, bin_size_rad)
        index.setdefault(key, []).append(indices)
        triangle_count += 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "catalog_path": str(args.catalog),
        "star_ids": [star[0] for star in stars],
        "vectors": [star[1].tolist() for star in stars],
        "magnitudes": [star[2] for star in stars],
        "bin_arcsec": args.bin_arcsec,
        "bin_size_rad": bin_size_rad,
        "min_edge_deg": args.min_edge_deg,
        "max_edge_deg": args.max_edge_deg,
        "index": index,
    }
    with args.output.open("wb") as handle:
        pickle.dump(payload, handle)

    metadata = {
        "catalog_path": str(args.catalog),
        "stars": len(stars),
        "triangles": triangle_count,
        "bins": len(index),
        "bin_arcsec": args.bin_arcsec,
        "min_edge_deg": args.min_edge_deg,
        "max_edge_deg": args.max_edge_deg,
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
