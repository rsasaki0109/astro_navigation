#!/usr/bin/env python3
"""Identify unlabeled stars using a prebuilt angular pair index."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import pickle
import random
import time
from pathlib import Path

import numpy as np
import pandas as pd

from identify_stars_with_index import (
    angular_distance,
    estimate_rotation_camera_inertial,
    load_observations_with_mag,
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
    parser.add_argument(
        "--calibration-json",
        type=Path,
        default=None,
        help="Optional JSON bundle of camera calibration. Schema matches the truth.json "
        "written by generate_star_tracker_observations_from_catalog.py: top-level keys "
        "'intrinsics' (fx/fy/cx/cy) and optional 'distortion' (k1/k2/p1/p2). When set, the "
        "values populate any unset --fx/--fy/--cx/--cy/--distortion-* flags; explicit flags "
        "still win.",
    )
    parser.add_argument("--fx", type=float, default=None)
    parser.add_argument("--fy", type=float, default=None)
    parser.add_argument("--cx", type=float, default=None)
    parser.add_argument("--cy", type=float, default=None)
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
    parser.add_argument(
        "--pyramid-restarts",
        type=int,
        default=0,
        help="Maximum number of additional pyramid attempts with shuffled observation pools when "
        "the assigned-observation count from the first attempt falls below "
        "--confidence-fraction * total observations. 0 disables retry (current behavior).",
    )
    parser.add_argument(
        "--confidence-fraction",
        type=float,
        default=0.5,
        help="Restart-pyramid threshold: if assigned/observations is below this fraction after an "
        "attempt, retry with a new random subset (up to --pyramid-restarts times). The attempt "
        "with the most assignments wins regardless.",
    )
    parser.add_argument(
        "--pyramid-restart-seed",
        type=int,
        default=0,
        help="Seed for the observation-permutation RNG used between pyramid restarts.",
    )
    parser.add_argument(
        "--distortion-k1",
        type=float,
        default=None,
        help="Brown-Conrady k1 used to iteratively undistort the loaded (u, v) pixels at the "
        "front door. Default None falls back to --calibration-json or 0 (no undistortion).",
    )
    parser.add_argument("--distortion-k2", type=float, default=None)
    parser.add_argument("--distortion-p1", type=float, default=None)
    parser.add_argument("--distortion-p2", type=float, default=None)
    args = parser.parse_args()

    # Reconcile --calibration-json with the individual flags. Explicit CLI flags always win;
    # JSON values fill in anything left as None. After reconciliation fx/fy/cx/cy must be
    # set, distortion defaults to 0.
    calibration_payload: dict = {}
    if args.calibration_json is not None:
        calibration_payload = json.loads(args.calibration_json.read_text(encoding="utf-8"))
    intrinsics_block = calibration_payload.get("intrinsics", {})
    distortion_block = calibration_payload.get("distortion", {})
    for axis in ("fx", "fy", "cx", "cy"):
        if getattr(args, axis) is None:
            value = intrinsics_block.get(axis)
            if value is None:
                raise SystemExit(
                    f"identify_stars_with_pair_index: --{axis} required (or set via --calibration-json)"
                )
            setattr(args, axis, float(value))
    for coef in ("k1", "k2", "p1", "p2"):
        attr = f"distortion_{coef}"
        if getattr(args, attr) is None:
            setattr(args, attr, float(distortion_block.get(coef, 0.0)))

    star_ids, catalog_vectors, catalog_magnitudes, index, bin_size_rad, index_format = load_pair_index(args.index)
    tolerance_rad = math.radians(args.tolerance_arcsec / 3600.0)
    verification_tolerance_rad = math.radians(args.verification_tolerance_arcsec / 3600.0)
    magnitude_prior_rad = math.radians(args.magnitude_prior_arcsec / 3600.0)

    observations, observation_magnitudes = load_observations_with_mag(
        args.observations,
        args.fx,
        args.fy,
        args.cx,
        args.cy,
        k1=args.distortion_k1,
        k2=args.distortion_k2,
        p1=args.distortion_p1,
        p2=args.distortion_p2,
    )

    triangle_matches = 0
    candidate_hypotheses = 0
    verified_hypotheses = 0
    pruning_seconds = 0.0
    candidate_generation_seconds = 0.0
    verification_seconds = 0.0
    observation_triangles_evaluated = 0

    best_assignments: dict[int, int] = {}
    best_rms_error = math.inf
    best_mean_score = math.inf
    best_attempt_index = -1

    permutation = list(range(len(observations)))
    restart_rng = random.Random(args.pyramid_restart_seed)
    confidence_target = args.confidence_fraction * len(observations)
    attempts_taken = 0

    for attempt in range(args.pyramid_restarts + 1):
        attempts_taken = attempt + 1
        if args.pyramid_size > 0:
            pool = permutation[: min(args.pyramid_size, len(permutation))]
        else:
            pool = list(permutation)
        base_triangles = select_observation_triangles(len(pool), args.max_observation_triangles)
        observation_triangles = [(pool[a], pool[b], pool[c]) for (a, b, c) in base_triangles]
        observation_triangles_evaluated += len(observation_triangles)

        hypotheses: list[tuple[float, tuple[int, int, int], tuple[int, int, int]]] = []
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
        triangle_matches += len(hypotheses)
        hypotheses.sort(key=lambda item: item[0])
        if args.max_verified_hypotheses > 0:
            hypotheses = hypotheses[: args.max_verified_hypotheses]
        pruning_seconds += time.perf_counter() - pruning_start

        attempt_assignments: dict[int, int] = {}
        attempt_rms_error = math.inf
        attempt_mean_score = math.inf
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
                observation_magnitudes=observation_magnitudes,
            )
            verification_seconds += time.perf_counter() - verify_start
            if candidate_assignments:
                verified_hypotheses += 1
            if (
                len(candidate_assignments) > len(attempt_assignments)
                or (
                    len(candidate_assignments) == len(attempt_assignments)
                    and (
                        mean_score < attempt_mean_score
                        or (math.isclose(mean_score, attempt_mean_score) and rms_error < attempt_rms_error)
                    )
                )
            ):
                attempt_assignments = candidate_assignments
                attempt_rms_error = rms_error
                attempt_mean_score = mean_score

        if (
            len(attempt_assignments) > len(best_assignments)
            or (
                len(attempt_assignments) == len(best_assignments)
                and (
                    attempt_mean_score < best_mean_score
                    or (math.isclose(attempt_mean_score, best_mean_score) and attempt_rms_error < best_rms_error)
                )
            )
        ):
            best_assignments = attempt_assignments
            best_rms_error = attempt_rms_error
            best_mean_score = attempt_mean_score
            best_attempt_index = attempt

        if len(best_assignments) >= confidence_target:
            break

        restart_rng.shuffle(permutation)

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
        "observation_triangles_evaluated": observation_triangles_evaluated,
        "pyramid_size": args.pyramid_size,
        "pyramid_restarts": args.pyramid_restarts,
        "confidence_fraction": args.confidence_fraction,
        "attempts_taken": attempts_taken,
        "winning_attempt_index": best_attempt_index,
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
