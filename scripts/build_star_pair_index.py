#!/usr/bin/env python3
"""Build an angular pair index for scalable lost-in-space star identification."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import pickle
import struct
from pathlib import Path

import numpy as np


_BIN_MAGIC = b"ASTROIDX"
_BIN_VERSION = 1


def write_pair_index_bin(
    path: Path,
    *,
    star_ids: list[str],
    vectors: np.ndarray,
    magnitudes: np.ndarray,
    bin_keys: np.ndarray,
    bin_offsets: np.ndarray,
    pair_endpoints: np.ndarray,
    bin_arcsec: float,
    bin_size_rad: float,
    min_edge_deg: float,
    max_edge_deg: float,
) -> None:
    """Write the pair index in a flat binary format readable from C++ (no zlib).

    Layout:
        magic[8] = b"ASTROIDX"
        version: uint32
        n_stars: int64, n_bins: int64, n_pairs: int64
        bin_arcsec / bin_size_rad / min_edge_deg / max_edge_deg: float64 (×4)
        4 bytes padding (header total 72 bytes)
        vectors: n_stars × 3 × float64
        magnitudes: n_stars × float64
        bin_keys: n_bins × int32
        bin_offsets: (n_bins + 1) × int64
        pair_endpoints: n_pairs × 2 × int32
        star_ids_blob_size: int64
        star_ids_blob: concatenation of (uint16 length-LE, utf-8 bytes) × n_stars
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    n_stars = len(star_ids)
    n_bins = int(bin_keys.size)
    n_pairs = int(pair_endpoints.shape[0])
    blob = bytearray()
    for star_id in star_ids:
        encoded = star_id.encode("utf-8")
        if len(encoded) > 0xFFFF:
            raise ValueError(f"star id too long for uint16 length prefix: {star_id!r}")
        blob.extend(struct.pack("<H", len(encoded)))
        blob.extend(encoded)
    with path.open("wb") as handle:
        handle.write(_BIN_MAGIC)
        handle.write(struct.pack("<I", _BIN_VERSION))
        handle.write(struct.pack("<qqq", n_stars, n_bins, n_pairs))
        handle.write(struct.pack("<dddd", bin_arcsec, bin_size_rad, min_edge_deg, max_edge_deg))
        handle.write(b"\x00\x00\x00\x00")  # pad to 72-byte header
        np.ascontiguousarray(vectors, dtype=np.float64).tofile(handle)
        np.ascontiguousarray(magnitudes, dtype=np.float64).tofile(handle)
        np.ascontiguousarray(bin_keys, dtype=np.int32).tofile(handle)
        np.ascontiguousarray(bin_offsets, dtype=np.int64).tofile(handle)
        np.ascontiguousarray(pair_endpoints, dtype=np.int32).tofile(handle)
        handle.write(struct.pack("<q", len(blob)))
        handle.write(bytes(blob))


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


_DEFAULT_SKY_CELL_LAT = 4
_DEFAULT_SKY_CELL_LON = 12


def sky_cell_id_array(vectors: np.ndarray, n_lat: int, n_lon: int) -> np.ndarray:
    """Vectorized sky-cell id lookup for unit vectors.

    Partitions the sphere into `n_lat` equal-area latitude bands (uniform in `sin(dec) = z`)
    and `n_lon` equal-longitude slices per band → `n_lat * n_lon` cells. Cell IDs are
    `lat_bin * n_lon + lon_bin` and fit in int16 for any reasonable (n_lat, n_lon).
    """
    z = vectors[:, 2].astype(np.float64, copy=False)
    lat_bin = np.clip(((z + 1.0) * 0.5 * n_lat).astype(np.int64), 0, n_lat - 1)
    phi = np.arctan2(vectors[:, 1], vectors[:, 0])
    lon_bin = (((phi + math.pi) / (2.0 * math.pi)) * n_lon).astype(np.int64) % n_lon
    return (lat_bin * n_lon + lon_bin).astype(np.int16)


def sky_cell_centers(n_lat: int, n_lon: int) -> np.ndarray:
    """Unit-vector centers of the `n_lat * n_lon` sky cells, ordered by id."""
    centers: list[list[float]] = []
    for lat_bin in range(n_lat):
        z_min = -1.0 + lat_bin / n_lat * 2.0
        z_max = -1.0 + (lat_bin + 1) / n_lat * 2.0
        z_center = 0.5 * (z_min + z_max)
        cos_lat = math.sqrt(max(0.0, 1.0 - z_center * z_center))
        for lon_bin in range(n_lon):
            phi_center = -math.pi + (lon_bin + 0.5) / n_lon * 2.0 * math.pi
            centers.append(
                [cos_lat * math.cos(phi_center), cos_lat * math.sin(phi_center), z_center]
            )
    return np.asarray(centers, dtype=np.float64)


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
    parser.add_argument(
        "--write-bin",
        action="store_true",
        help="Also dual-write a flat binary `.bin` next to the `.npz` for zero-dependency C++ "
        "consumption (no zlib required).",
    )
    parser.add_argument(
        "--sky-cell-lat",
        type=int,
        default=_DEFAULT_SKY_CELL_LAT,
        help="Latitude bands for the sky-cell partition stored alongside the catalog vectors.",
    )
    parser.add_argument(
        "--sky-cell-lon",
        type=int,
        default=_DEFAULT_SKY_CELL_LON,
        help="Longitude slices per latitude band. Total cells = lat * lon.",
    )
    args = parser.parse_args()

    stars = load_catalog(args.catalog, args.limit)
    vectors = np.asarray([star[1] for star in stars], dtype=np.float64)
    bin_size_rad = math.radians(args.bin_arcsec / 3600.0)
    min_edge_rad = math.radians(args.min_edge_deg)
    max_edge_rad = math.radians(args.max_edge_deg)
    star_count = len(vectors)

    # Vectorized pair enumeration: for each i, batch the (i, j>i) pair edges in numpy.
    # Intermediate buffers use int32 — catalog indices and bin keys both fit comfortably
    # (max catalog index ~80k vs int32 max 2.1G; max bin key ~max_edge_deg/bin_arcsec arcsec
    # which is in the low thousands). int32 halves intermediate memory vs the previous int64
    # path, which is required to fit large (60k+) builds into typical workstation RAM.
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
        valid_j = (np.flatnonzero(mask) + (i + 1)).astype(np.int32, copy=False)
        valid_edges = edges[mask]
        valid_bins = np.round(valid_edges / bin_size_rad).astype(np.int32)
        pair_lhs_chunks.append(np.full(valid_j.shape, i, dtype=np.int32))
        pair_rhs_chunks.append(valid_j)
        pair_bin_chunks.append(valid_bins)

    if pair_lhs_chunks:
        all_lhs = np.concatenate(pair_lhs_chunks)
        all_rhs = np.concatenate(pair_rhs_chunks)
        all_bins = np.concatenate(pair_bin_chunks)
    else:
        all_lhs = np.empty(0, dtype=np.int32)
        all_rhs = np.empty(0, dtype=np.int32)
        all_bins = np.empty(0, dtype=np.int32)
    pair_lhs_chunks.clear()
    pair_rhs_chunks.clear()
    pair_bin_chunks.clear()

    pair_count = int(all_lhs.size)
    # Group into bin buckets in one numpy-driven pass.
    sort_order = np.argsort(all_bins, kind="stable")
    sorted_bins = all_bins[sort_order]
    sorted_lhs = all_lhs[sort_order]
    sorted_rhs = all_rhs[sort_order]
    del all_lhs, all_rhs, all_bins, sort_order
    unique_bins, group_starts = np.unique(sorted_bins, return_index=True)
    group_ends = np.empty_like(group_starts)
    group_ends[:-1] = group_starts[1:]
    group_ends[-1] = sorted_bins.size
    del sorted_bins

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

    cell_ids_arr = sky_cell_id_array(vectors_arr, args.sky_cell_lat, args.sky_cell_lon)
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
        sky_cell_ids=cell_ids_arr,
        sky_cell_lat=np.int32(args.sky_cell_lat),
        sky_cell_lon=np.int32(args.sky_cell_lon),
    )

    bin_path = args.output.with_suffix(".bin") if args.write_bin else None
    if args.write_bin:
        write_pair_index_bin(
            bin_path,
            star_ids=star_ids,
            vectors=vectors_arr,
            magnitudes=magnitudes_arr,
            bin_keys=bin_keys_arr,
            bin_offsets=bin_offsets_arr,
            pair_endpoints=pair_endpoints_arr,
            bin_arcsec=args.bin_arcsec,
            bin_size_rad=bin_size_rad,
            min_edge_deg=args.min_edge_deg,
            max_edge_deg=args.max_edge_deg,
        )

    metadata = {
        "catalog_path": str(args.catalog),
        "stars": len(stars),
        "pairs": pair_count,
        "bins": int(bin_keys_arr.size),
        "bin_arcsec": args.bin_arcsec,
        "min_edge_deg": args.min_edge_deg,
        "max_edge_deg": args.max_edge_deg,
        "sky_cell_lat": args.sky_cell_lat,
        "sky_cell_lon": args.sky_cell_lon,
        "sky_cells": int(args.sky_cell_lat * args.sky_cell_lon),
        "pkl_path": None if args.skip_pkl else str(args.output),
        "npz_path": str(npz_path),
        "bin_path": str(bin_path) if bin_path is not None else None,
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
