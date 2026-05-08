#!/usr/bin/env python3
"""Build an angular pair index for scalable lost-in-space star identification."""

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


def edge_bin(edge: float, bin_size_rad: float) -> int:
    return int(round(edge / bin_size_rad))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=1000)
    parser.add_argument("--bin-arcsec", type=float, default=120.0)
    parser.add_argument("--min-edge-deg", type=float, default=0.2)
    parser.add_argument("--max-edge-deg", type=float, default=80.0)
    parser.add_argument(
        "--skip-pkl",
        action="store_true",
        help="Emit only the .npz/.json artifacts. Required for very large indices where the "
        "Python dict-of-tuples pickle representation exhausts memory (e.g. 40000+ stars).",
    )
    args = parser.parse_args()

    stars = load_catalog(args.catalog, args.limit)
    vectors = np.asarray([star[1] for star in stars], dtype=np.float64)
    bin_size_rad = math.radians(args.bin_arcsec / 3600.0)
    min_edge_rad = math.radians(args.min_edge_deg)
    max_edge_rad = math.radians(args.max_edge_deg)
    star_count = len(vectors)

    # Vectorized pair enumeration: for each i, batch the (i, j>i) pair edges in numpy.
    pair_lhs_chunks: list[np.ndarray] = []
    pair_rhs_chunks: list[np.ndarray] = []
    pair_bin_chunks: list[np.ndarray] = []
    for i in range(star_count - 1):
        rest = vectors[i + 1 :]
        dots = np.clip(rest @ vectors[i], -1.0, 1.0)
        edges = np.arccos(dots)
        mask = (edges >= min_edge_rad) & (edges <= max_edge_rad)
        if not mask.any():
            continue
        valid_j = np.flatnonzero(mask) + (i + 1)
        valid_edges = edges[mask]
        valid_bins = np.round(valid_edges / bin_size_rad).astype(np.int64)
        pair_lhs_chunks.append(np.full(valid_j.shape, i, dtype=np.int64))
        pair_rhs_chunks.append(valid_j)
        pair_bin_chunks.append(valid_bins)

    if pair_lhs_chunks:
        all_lhs = np.concatenate(pair_lhs_chunks)
        all_rhs = np.concatenate(pair_rhs_chunks)
        all_bins = np.concatenate(pair_bin_chunks)
    else:
        all_lhs = np.empty(0, dtype=np.int64)
        all_rhs = np.empty(0, dtype=np.int64)
        all_bins = np.empty(0, dtype=np.int64)

    pair_count = int(all_lhs.size)
    # Group into bin buckets in one numpy-driven pass.
    sort_order = np.argsort(all_bins, kind="stable")
    sorted_bins = all_bins[sort_order]
    sorted_lhs = all_lhs[sort_order]
    sorted_rhs = all_rhs[sort_order]
    unique_bins, group_starts = np.unique(sorted_bins, return_index=True)
    group_ends = np.empty_like(group_starts)
    group_ends[:-1] = group_starts[1:]
    group_ends[-1] = sorted_bins.size

    args.output.parent.mkdir(parents=True, exist_ok=True)
    star_ids = [star[0] for star in stars]
    vectors_arr = vectors  # already (N, 3) float64
    magnitudes_arr = np.asarray([star[2] for star in stars], dtype=np.float64)

    bin_keys_arr = unique_bins.astype(np.int32, copy=False)
    pair_counts = (group_ends - group_starts).astype(np.int64, copy=False)
    bin_offsets_arr = np.empty(bin_keys_arr.size + 1, dtype=np.int64)
    bin_offsets_arr[0] = 0
    np.cumsum(pair_counts, out=bin_offsets_arr[1:])

    pair_endpoints_arr = np.empty((int(bin_offsets_arr[-1]), 2), dtype=np.int32)
    pair_endpoints_arr[:, 0] = sorted_lhs
    pair_endpoints_arr[:, 1] = sorted_rhs

    if not args.skip_pkl:
        index: dict[int, list[tuple[int, int]]] = {}
        for slot in range(bin_keys_arr.size):
            start = int(bin_offsets_arr[slot])
            end = int(bin_offsets_arr[slot + 1])
            index[int(bin_keys_arr[slot])] = list(
                zip(sorted_lhs[start:end].tolist(), sorted_rhs[start:end].tolist())
            )
        payload = {
            "index_type": "pair_angle_v1",
            "catalog_path": str(args.catalog),
            "star_ids": star_ids,
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
        del index, payload

    npz_path = args.output.with_suffix(".npz")
    np.savez_compressed(
        npz_path,
        index_type=np.array("pair_angle_v1"),
        star_ids=np.array(star_ids),
        vectors=vectors_arr,
        magnitudes=magnitudes_arr,
        bin_arcsec=np.float64(args.bin_arcsec),
        bin_size_rad=np.float64(bin_size_rad),
        min_edge_deg=np.float64(args.min_edge_deg),
        max_edge_deg=np.float64(args.max_edge_deg),
        bin_keys=bin_keys_arr,
        bin_offsets=bin_offsets_arr,
        pair_endpoints=pair_endpoints_arr,
    )

    metadata = {
        "catalog_path": str(args.catalog),
        "stars": len(stars),
        "pairs": pair_count,
        "bins": int(bin_keys_arr.size),
        "bin_arcsec": args.bin_arcsec,
        "min_edge_deg": args.min_edge_deg,
        "max_edge_deg": args.max_edge_deg,
        "pkl_path": None if args.skip_pkl else str(args.output),
        "npz_path": str(npz_path),
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
