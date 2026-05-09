#!/usr/bin/env python3
"""Identify unlabeled stars using a prebuilt angular triangle index."""

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


def _undistort_normalized(
    x_d: float,
    y_d: float,
    k1: float,
    k2: float,
    p1: float,
    p2: float,
    iterations: int = 8,
) -> tuple[float, float]:
    """Iteratively invert Brown-Conrady distortion in normalized image coordinates.

    Forward model (matches generate_star_tracker_observations_from_catalog.py):
        x_d = x * (1 + k1 r2 + k2 r2^2) + 2 p1 x y + p2 (r2 + 2 x^2)
        y_d = y * (1 + k1 r2 + k2 r2^2) + p1 (r2 + 2 y^2) + 2 p2 x y
    Newton-style fixed-point inversion converges in ~5 iterations for the
    distortion magnitudes we care about (|k1| up to ~0.5).
    """
    x = x_d
    y = y_d
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2
        x_t = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        y_t = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (x_d - x_t) / radial
        y = (y_d - y_t) / radial
    return x, y


def load_observations(
    path: Path,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    k1: float = 0.0,
    k2: float = 0.0,
    p1: float = 0.0,
    p2: float = 0.0,
) -> list[np.ndarray]:
    bearings, _ = load_observations_with_mag(path, fx, fy, cx, cy, k1, k2, p1, p2)
    return bearings


def load_observations_with_mag(
    path: Path,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    k1: float = 0.0,
    k2: float = 0.0,
    p1: float = 0.0,
    p2: float = 0.0,
) -> tuple[list[np.ndarray], list[float] | None]:
    """Load observations and the optional `mag` column.

    Returns (bearings, magnitudes). `magnitudes` is None when the input has no mag
    column, which preserves the historical (id, u, v)-only schema. Distortion is
    handled identically to load_observations.
    """
    bearings: list[np.ndarray] = []
    distortion_active = (k1 != 0.0) or (k2 != 0.0) or (p1 != 0.0) or (p2 != 0.0)
    magnitudes: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        carries_mag = reader.fieldnames is not None and "mag" in reader.fieldnames
        for row in reader:
            u = float(row["u"])
            v = float(row["v"])
            x_d = (u - cx) / fx
            y_d = (v - cy) / fy
            if distortion_active:
                x_n, y_n = _undistort_normalized(x_d, y_d, k1, k2, p1, p2)
            else:
                x_n, y_n = x_d, y_d
            bearings.append(normalize(np.array([x_n, y_n, 1.0])))
            if carries_mag:
                magnitudes.append(float(row["mag"]))
    return bearings, (magnitudes if carries_mag else None)


def angular_distance(lhs: np.ndarray, rhs: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(lhs, rhs), -1.0, 1.0)))


def triangle_edges(vectors: list[np.ndarray], indices: tuple[int, int, int]) -> dict[frozenset[int], float]:
    a, b, c = indices
    return {
        frozenset((a, b)): angular_distance(vectors[a], vectors[b]),
        frozenset((a, c)): angular_distance(vectors[a], vectors[c]),
        frozenset((b, c)): angular_distance(vectors[b], vectors[c]),
    }


def edge_bins(edges: list[float], bin_size_rad: float) -> tuple[int, int, int]:
    return tuple(int(round(edge / bin_size_rad)) for edge in sorted(edges))


def estimate_rotation_camera_inertial(
    observations: list[np.ndarray],
    catalog_vectors: list[np.ndarray],
    pairs: list[tuple[int, int]],
) -> np.ndarray:
    correlation = np.zeros((3, 3), dtype=float)
    for obs_index, cat_index in pairs:
        correlation += np.outer(observations[obs_index], catalog_vectors[cat_index])
    u, _singular_values, vt = np.linalg.svd(correlation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def verify_rotation(
    rotation_camera_inertial: np.ndarray,
    observations: list[np.ndarray],
    catalog_vectors: list[np.ndarray] | np.ndarray,
    catalog_magnitudes: list[float],
    tolerance_rad: float,
    magnitude_prior_rad: float,
    observation_magnitudes: list[float] | None = None,
) -> tuple[dict[int, int], float, float]:
    """Greedy verification under a rotation hypothesis.

    Score per (obs, cat) candidate: `error + magnitude_prior_rad * mag_term`.
    `mag_term` is `|obs_mag - cat_mag|` when observation_magnitudes is provided
    (sharper discrimination because the prior penalizes magnitude mismatch),
    else falls back to the legacy `cat_mag` (prefer-bright-stars prior). The
    fallback path is bit-exact against fixtures that predate the obs-mag axis.
    """
    candidates: list[tuple[float, float, int, int]] = []
    catalog_matrix = np.asarray(catalog_vectors, dtype=float)
    magnitude_vector = np.asarray(catalog_magnitudes, dtype=float)
    predicted_vectors = catalog_matrix @ rotation_camera_inertial.T
    min_dot = math.cos(tolerance_rad)
    use_obs_mag = observation_magnitudes is not None
    for obs_index, observation in enumerate(observations):
        dots = predicted_vectors @ observation
        candidate_indices = np.flatnonzero(dots >= min_dot)
        if candidate_indices.size == 0:
            continue
        errors = np.arccos(np.clip(dots[candidate_indices], -1.0, 1.0))
        if use_obs_mag:
            mag_term = np.abs(magnitude_vector[candidate_indices] - observation_magnitudes[obs_index])
        else:
            mag_term = magnitude_vector[candidate_indices]
        scores = errors + magnitude_prior_rad * mag_term
        for score, error, cat_index in zip(scores, errors, candidate_indices, strict=True):
            candidates.append((float(score), float(error), obs_index, int(cat_index)))

    assignments: dict[int, int] = {}
    used_catalog_indices: set[int] = set()
    errors: list[float] = []
    score_sum = 0.0
    for score, error, obs_index, cat_index in sorted(candidates):
        if obs_index in assignments or cat_index in used_catalog_indices:
            continue
        assignments[obs_index] = cat_index
        used_catalog_indices.add(cat_index)
        errors.append(error)
        score_sum += score

    rms_error = math.sqrt(sum(error * error for error in errors) / len(errors)) if errors else math.inf
    mean_score = score_sum / len(errors) if errors else math.inf
    return assignments, rms_error, mean_score


def neighboring_keys(key: tuple[int, int, int], radius: int) -> list[tuple[int, int, int]]:
    keys: list[tuple[int, int, int]] = []
    for da in range(-radius, radius + 1):
        for db in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                candidate = (key[0] + da, key[1] + db, key[2] + dc)
                if candidate[0] <= candidate[1] <= candidate[2]:
                    keys.append(candidate)
    return keys


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
    parser.add_argument("--neighbor-bins", type=int, default=1)
    parser.add_argument("--verification-tolerance-arcsec", type=float, default=600.0)
    parser.add_argument("--magnitude-prior-arcsec", type=float, default=15.0)
    parser.add_argument("--max-observation-triangles", type=int, default=400)
    args = parser.parse_args()

    with args.index.open("rb") as handle:
        index_payload = pickle.load(handle)
    star_ids: list[str] = index_payload["star_ids"]
    catalog_vectors = np.array(index_payload["vectors"], dtype=float)
    catalog_magnitudes = [float(magnitude) for magnitude in index_payload.get("magnitudes", [0.0] * len(star_ids))]
    index: dict[tuple[int, int, int], list[tuple[int, int, int]]] = index_payload["index"]
    bin_size_rad = float(index_payload["bin_size_rad"])
    tolerance_rad = math.radians(args.tolerance_arcsec / 3600.0)

    observations = load_observations(args.observations, args.fx, args.fy, args.cx, args.cy)
    votes: dict[tuple[int, int], int] = {}
    triangle_matches = 0
    verified_hypotheses = 0
    best_assignments: dict[int, int] = {}
    best_rms_error = math.inf
    best_mean_score = math.inf
    verification_tolerance_rad = math.radians(args.verification_tolerance_arcsec / 3600.0)
    magnitude_prior_rad = math.radians(args.magnitude_prior_arcsec / 3600.0)
    observation_triangles = select_observation_triangles(len(observations), args.max_observation_triangles)
    for obs_indices in observation_triangles:
        obs_edges = triangle_edges(observations, obs_indices)
        obs_key = edge_bins(list(obs_edges.values()), bin_size_rad)
        candidate_triangles: list[tuple[int, int, int]] = []
        for key in neighboring_keys(obs_key, args.neighbor_bins):
            candidate_triangles.extend(index.get(key, []))

        for cat_indices in candidate_triangles:
            for permuted_catalog in itertools.permutations(cat_indices):
                predicted_edges = {
                    frozenset((obs_indices[0], obs_indices[1])): angular_distance(
                        catalog_vectors[permuted_catalog[0]], catalog_vectors[permuted_catalog[1]]
                    ),
                    frozenset((obs_indices[0], obs_indices[2])): angular_distance(
                        catalog_vectors[permuted_catalog[0]], catalog_vectors[permuted_catalog[2]]
                    ),
                    frozenset((obs_indices[1], obs_indices[2])): angular_distance(
                        catalog_vectors[permuted_catalog[1]], catalog_vectors[permuted_catalog[2]]
                    ),
                }
                if all(abs(obs_edges[key] - predicted_edges[key]) <= tolerance_rad for key in obs_edges):
                    triangle_matches += 1
                    for obs_index, cat_index in zip(obs_indices, permuted_catalog, strict=True):
                        votes[(obs_index, cat_index)] = votes.get((obs_index, cat_index), 0) + 1
                    rotation_camera_inertial = estimate_rotation_camera_inertial(
                        observations,
                        catalog_vectors,
                        list(zip(obs_indices, permuted_catalog, strict=True)),
                    )
                    candidate_assignments, rms_error, mean_score = verify_rotation(
                        rotation_camera_inertial,
                        observations,
                        catalog_vectors,
                        catalog_magnitudes,
                        verification_tolerance_rad,
                        magnitude_prior_rad,
                    )
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

    assignments: dict[int, str] = {}
    if best_assignments:
        assignments = {obs_index: star_ids[cat_index] for obs_index, cat_index in best_assignments.items()}
    else:
        used_catalog_indices: set[int] = set()
        ranked_votes = sorted(
            ((vote_count, obs_index, cat_index) for (obs_index, cat_index), vote_count in votes.items()),
            reverse=True,
        )
        for _vote_count, obs_index, cat_index in ranked_votes:
            if obs_index in assignments or cat_index in used_catalog_indices:
                continue
            assignments[obs_index] = star_ids[cat_index]
            used_catalog_indices.add(cat_index)

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
        "index_stars": len(star_ids),
        "tolerance_arcsec": args.tolerance_arcsec,
        "neighbor_bins": args.neighbor_bins,
        "verification_tolerance_arcsec": args.verification_tolerance_arcsec,
        "magnitude_prior_arcsec": args.magnitude_prior_arcsec,
        "verified_hypotheses": verified_hypotheses,
        "best_rms_error_arcsec": math.degrees(best_rms_error) * 3600.0 if math.isfinite(best_rms_error) else None,
        "best_mean_score_arcsec": math.degrees(best_mean_score) * 3600.0 if math.isfinite(best_mean_score) else None,
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
