#!/usr/bin/env python3
"""Identify unlabeled stars using a prebuilt angular pair index."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import pickle
import time
from pathlib import Path

import numpy as np
import pandas as pd

from identify_stars_with_index import (
    angular_distance,
    estimate_rotation_camera_inertial,
    load_observations,
    verify_rotation,
)


def edge_bin(edge: float, bin_size_rad: float) -> int:
    return int(round(edge / bin_size_rad))


def load_pair_index(
    index_path: Path,
) -> tuple[list[str], np.ndarray, np.ndarray, dict[int, list[tuple[int, int]]], float, str]:
    """Load a pair index from .npz (preferred) or .pkl.

    Returns (star_ids, catalog_vectors, catalog_magnitudes, index_dict, bin_size_rad, format_used).
    If `index_path` is a .pkl with a sibling .npz, the .npz is loaded.
    """
    suffix = index_path.suffix.lower()
    npz_candidate = index_path if suffix == ".npz" else index_path.with_suffix(".npz")
    if npz_candidate.exists():
        with np.load(npz_candidate, allow_pickle=False) as data:
            star_ids = [str(value) for value in data["star_ids"].tolist()]
            catalog_vectors = np.asarray(data["vectors"], dtype=float)
            catalog_magnitudes = np.asarray(data["magnitudes"], dtype=float)
            bin_size_rad = float(data["bin_size_rad"])
            bin_keys = np.asarray(data["bin_keys"], dtype=np.int64)
            bin_offsets = np.asarray(data["bin_offsets"], dtype=np.int64)
            pair_endpoints = np.ascontiguousarray(data["pair_endpoints"], dtype=np.int64)
        index: dict[int, np.ndarray] = {}
        for slot in range(len(bin_keys)):
            start = int(bin_offsets[slot])
            end = int(bin_offsets[slot + 1])
            index[int(bin_keys[slot])] = pair_endpoints[start:end]
        return star_ids, catalog_vectors, catalog_magnitudes, index, bin_size_rad, "npz"

    with index_path.open("rb") as handle:
        payload = pickle.load(handle)
    star_ids = list(payload["star_ids"])
    catalog_vectors = np.asarray(payload["vectors"], dtype=float)
    magnitudes_list = payload.get("magnitudes", [0.0] * len(star_ids))
    catalog_magnitudes = np.asarray([float(m) for m in magnitudes_list], dtype=float)
    bin_size_rad = float(payload["bin_size_rad"])
    raw_index = payload["index"]
    index = {int(k): list(v) for k, v in raw_index.items()}
    return star_ids, catalog_vectors, catalog_magnitudes, index, bin_size_rad, "pkl"


def neighboring_pair_keys(key: int, radius: int) -> range:
    return range(key - radius, key + radius + 1)


_EMPTY_PAIRS = np.empty((0, 2), dtype=np.int64)


def load_candidate_pairs(index: dict[int, "np.ndarray | list"], key: int, radius: int) -> np.ndarray:
    """Return the union of pair lists across `[key - radius, key + radius]` as an (N, 2) array.

    Accepts both list-of-tuple and ndarray bin values to keep .pkl and .npz indices interchangeable.
    """
    chunks: list[np.ndarray] = []
    for neighbor_key in neighboring_pair_keys(key, radius):
        bucket = index.get(neighbor_key)
        if bucket is None:
            continue
        if isinstance(bucket, np.ndarray):
            if bucket.size:
                chunks.append(bucket)
        elif bucket:
            chunks.append(np.asarray(bucket, dtype=np.int64))
    if not chunks:
        return _EMPTY_PAIRS
    return np.concatenate(chunks, axis=0)


def adjacency(pairs: "np.ndarray | list[tuple[int, int]]") -> dict[int, set[int]]:
    graph: dict[int, set[int]] = {}
    if isinstance(pairs, np.ndarray):
        for lhs, rhs in pairs.tolist():
            graph.setdefault(lhs, set()).add(rhs)
            graph.setdefault(rhs, set()).add(lhs)
    else:
        for lhs, rhs in pairs:
            graph.setdefault(lhs, set()).add(rhs)
            graph.setdefault(rhs, set()).add(lhs)
    return graph


def select_observation_triangles(
    observation_count: int,
    max_observation_triangles: int,
) -> list[tuple[int, int, int]]:
    triangles = list(itertools.combinations(range(observation_count), 3))
    if max_observation_triangles <= 0 or len(triangles) <= max_observation_triangles:
        return triangles
    if max_observation_triangles == 1:
        return [triangles[len(triangles) // 2]]
    last_index = len(triangles) - 1
    selected_indices = {
        round(index * last_index / (max_observation_triangles - 1))
        for index in range(max_observation_triangles)
    }
    return [triangles[index] for index in sorted(selected_indices)]


def candidate_mappings(
    observations: list[np.ndarray],
    catalog_vectors: np.ndarray,
    catalog_magnitudes: np.ndarray,
    index: dict[int, list[tuple[int, int]]],
    bin_size_rad: float,
    neighbor_bins: int,
    tolerance_rad: float,
    magnitude_prior_rad: float,
    obs_indices: tuple[int, int, int],
) -> list[tuple[float, tuple[int, int, int]]]:
    obs_a, obs_b, obs_c = obs_indices
    edge_ab = angular_distance(observations[obs_a], observations[obs_b])
    edge_ac = angular_distance(observations[obs_a], observations[obs_c])
    edge_bc = angular_distance(observations[obs_b], observations[obs_c])

    pairs_ab = load_candidate_pairs(index, edge_bin(edge_ab, bin_size_rad), neighbor_bins)
    pairs_ac = load_candidate_pairs(index, edge_bin(edge_ac, bin_size_rad), neighbor_bins)
    pairs_bc = load_candidate_pairs(index, edge_bin(edge_bc, bin_size_rad), neighbor_bins)
    if pairs_ab.shape[0] == 0 or pairs_ac.shape[0] == 0 or pairs_bc.shape[0] == 0:
        return []

    ab_arr = np.asarray(pairs_ab, dtype=np.int64)
    ac_arr = np.asarray(pairs_ac, dtype=np.int64)
    bc_arr = np.asarray(pairs_bc, dtype=np.int64)

    df_ab = pd.DataFrame(
        np.concatenate([ab_arr, ab_arr[:, [1, 0]]]), columns=["a", "b"]
    ).drop_duplicates()
    df_ac = pd.DataFrame(
        np.concatenate([ac_arr, ac_arr[:, [1, 0]]]), columns=["a", "c"]
    ).drop_duplicates()
    df_bc = pd.DataFrame(
        np.concatenate([bc_arr, bc_arr[:, [1, 0]]]), columns=["b", "c"]
    ).drop_duplicates()

    merged = df_ab.merge(df_ac, on="a", copy=False).merge(df_bc, on=["b", "c"], copy=False)
    if merged.empty:
        return []
    merged = merged[(merged["c"] != merged["a"]) & (merged["c"] != merged["b"])]
    if merged.empty:
        return []

    a_arr = merged["a"].to_numpy(dtype=np.int64, copy=False)
    b_arr = merged["b"].to_numpy(dtype=np.int64, copy=False)
    c_arr = merged["c"].to_numpy(dtype=np.int64, copy=False)

    va = catalog_vectors[a_arr]
    vb = catalog_vectors[b_arr]
    vc = catalog_vectors[c_arr]

    dot_ab = np.einsum("ij,ij->i", va, vb)
    dot_ac = np.einsum("ij,ij->i", va, vc)
    dot_bc = np.einsum("ij,ij->i", vb, vc)
    np.clip(dot_ab, -1.0, 1.0, out=dot_ab)
    np.clip(dot_ac, -1.0, 1.0, out=dot_ac)
    np.clip(dot_bc, -1.0, 1.0, out=dot_bc)

    err_ab = np.abs(edge_ab - np.arccos(dot_ab))
    err_ac = np.abs(edge_ac - np.arccos(dot_ac))
    err_bc = np.abs(edge_bc - np.arccos(dot_bc))

    accept = (err_ab <= tolerance_rad) & (err_ac <= tolerance_rad) & (err_bc <= tolerance_rad)
    if not accept.any():
        return []

    accepted = np.flatnonzero(accept)
    sel_a = a_arr[accepted]
    sel_b = b_arr[accepted]
    sel_c = c_arr[accepted]
    eab = err_ab[accepted]
    eac = err_ac[accepted]
    ebc = err_bc[accepted]

    edge_score = np.sqrt((eab * eab + eac * eac + ebc * ebc) / 3.0)
    magnitude_score = (
        catalog_magnitudes[sel_a] + catalog_magnitudes[sel_b] + catalog_magnitudes[sel_c]
    ) / 3.0
    final_scores = edge_score + magnitude_prior_rad * magnitude_score

    return [
        (float(final_scores[i]), (int(sel_a[i]), int(sel_b[i]), int(sel_c[i])))
        for i in range(accepted.size)
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument("--index", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fx", type=float, required=True)
    parser.add_argument("--fy", type=float, required=True)
    parser.add_argument("--cx", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument("--tolerance-arcsec", type=float, default=300.0)
    parser.add_argument("--neighbor-bins", type=int, default=2)
    parser.add_argument("--verification-tolerance-arcsec", type=float, default=600.0)
    parser.add_argument("--magnitude-prior-arcsec", type=float, default=15.0)
    parser.add_argument("--max-observation-triangles", type=int, default=400)
    parser.add_argument("--max-candidates-per-observation-triangle", type=int, default=8)
    parser.add_argument("--max-verified-hypotheses", type=int, default=400)
    parser.add_argument(
        "--pyramid-size",
        type=int,
        default=0,
        help="Pyramid mode: only build observation triangles from the first N observations. "
        "0 disables pyramid mode and uses all observations.",
    )
    args = parser.parse_args()

    star_ids, catalog_vectors, catalog_magnitudes, index, bin_size_rad, index_format = load_pair_index(args.index)
    tolerance_rad = math.radians(args.tolerance_arcsec / 3600.0)
    verification_tolerance_rad = math.radians(args.verification_tolerance_arcsec / 3600.0)
    magnitude_prior_rad = math.radians(args.magnitude_prior_arcsec / 3600.0)

    observations = load_observations(args.observations, args.fx, args.fy, args.cx, args.cy)
    if args.pyramid_size > 0:
        triangle_pool_size = min(args.pyramid_size, len(observations))
    else:
        triangle_pool_size = len(observations)
    observation_triangles = select_observation_triangles(triangle_pool_size, args.max_observation_triangles)

    triangle_matches = 0
    candidate_hypotheses = 0
    verified_hypotheses = 0
    best_assignments: dict[int, int] = {}
    best_rms_error = math.inf
    best_mean_score = math.inf
    hypotheses: list[tuple[float, tuple[int, int, int], tuple[int, int, int]]] = []

    candidate_generation_seconds = 0.0
    verification_seconds = 0.0

    for obs_indices in observation_triangles:
        gen_start = time.perf_counter()
        candidates = candidate_mappings(
            observations,
            catalog_vectors,
            catalog_magnitudes,
            index,
            bin_size_rad,
            args.neighbor_bins,
            tolerance_rad,
            magnitude_prior_rad,
            obs_indices,
        )
        candidate_hypotheses += len(candidates)
        candidates.sort(key=lambda item: item[0])
        if args.max_candidates_per_observation_triangle > 0:
            candidates = candidates[: args.max_candidates_per_observation_triangle]
        candidate_generation_seconds += time.perf_counter() - gen_start
        for candidate_score, cat_mapping in candidates:
            hypotheses.append((candidate_score, obs_indices, cat_mapping))

    pruning_start = time.perf_counter()
    triangle_matches = len(hypotheses)
    hypotheses.sort(key=lambda item: item[0])
    if args.max_verified_hypotheses > 0:
        hypotheses = hypotheses[: args.max_verified_hypotheses]
    pruning_seconds = time.perf_counter() - pruning_start

    for _candidate_score, obs_indices, cat_mapping in hypotheses:
            verify_start = time.perf_counter()
            rotation_camera_inertial = estimate_rotation_camera_inertial(
                observations,
                catalog_vectors,
                list(zip(obs_indices, cat_mapping, strict=True)),
            )
            candidate_assignments, rms_error, mean_score = verify_rotation(
                rotation_camera_inertial,
                observations,
                catalog_vectors,
                catalog_magnitudes,
                verification_tolerance_rad,
                magnitude_prior_rad,
            )
            verification_seconds += time.perf_counter() - verify_start
            if candidate_assignments:
                verified_hypotheses += 1
            if (
                len(candidate_assignments) > len(best_assignments)
                or (
                    len(candidate_assignments) == len(best_assignments)
                    and (
                        mean_score < best_mean_score
                        or (math.isclose(mean_score, best_mean_score) and rms_error < best_rms_error)
                    )
                )
            ):
                best_assignments = candidate_assignments
                best_rms_error = rms_error
                best_mean_score = mean_score

    assignments = {obs_index: star_ids[cat_index] for obs_index, cat_index in best_assignments.items()}

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["observation_index", "id"])
        for obs_index, star_id in sorted(assignments.items()):
            writer.writerow([obs_index, star_id])

    metadata = {
        "assigned_observations": len(assignments),
        "triangle_matches": triangle_matches,
        "observations": len(observations),
        "observation_triangles": len(observation_triangles),
        "pyramid_size": args.pyramid_size,
        "triangle_pool_size": triangle_pool_size,
        "index_stars": len(star_ids),
        "index_pairs": sum(len(pairs) for pairs in index.values()),
        "candidate_hypotheses": candidate_hypotheses,
        "pruned_hypotheses": triangle_matches,
        "tolerance_arcsec": args.tolerance_arcsec,
        "neighbor_bins": args.neighbor_bins,
        "verification_tolerance_arcsec": args.verification_tolerance_arcsec,
        "magnitude_prior_arcsec": args.magnitude_prior_arcsec,
        "max_candidates_per_observation_triangle": args.max_candidates_per_observation_triangle,
        "max_verified_hypotheses": args.max_verified_hypotheses,
        "verified_hypotheses": verified_hypotheses,
        "best_rms_error_arcsec": math.degrees(best_rms_error) * 3600.0 if math.isfinite(best_rms_error) else None,
        "best_mean_score_arcsec": math.degrees(best_mean_score) * 3600.0 if math.isfinite(best_mean_score) else None,
        "candidate_generation_seconds": candidate_generation_seconds,
        "pruning_seconds": pruning_seconds,
        "verification_seconds": verification_seconds,
        "index_format": index_format,
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
