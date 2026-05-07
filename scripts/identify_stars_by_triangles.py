#!/usr/bin/env python3
"""Identify stars from unlabeled observations using angular triangle patterns."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

import numpy as np


def normalize(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def load_unlabeled_observations(path: Path, fx: float, fy: float, cx: float, cy: float) -> list[np.ndarray]:
    bearings: list[np.ndarray] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            u = float(row["u"])
            v = float(row["v"])
            bearings.append(normalize(np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)))
    return bearings


def load_catalog(path: Path) -> list[tuple[str, np.ndarray]]:
    rows: list[tuple[str, np.ndarray]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((row["id"], normalize(np.array([float(row["x"]), float(row["y"]), float(row["z"])]))))
    return rows


def angular_distance(lhs: np.ndarray, rhs: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(lhs, rhs), -1.0, 1.0)))


def triangle_signature(vectors: list[np.ndarray], indices: tuple[int, int, int]) -> tuple[float, float, float]:
    a, b, c = indices
    edges = [
        angular_distance(vectors[a], vectors[b]),
        angular_distance(vectors[a], vectors[c]),
        angular_distance(vectors[b], vectors[c]),
    ]
    return tuple(sorted(edges))


def triangle_edges(vectors: list[np.ndarray], indices: tuple[int, int, int]) -> dict[frozenset[int], float]:
    a, b, c = indices
    return {
        frozenset((a, b)): angular_distance(vectors[a], vectors[b]),
        frozenset((a, c)): angular_distance(vectors[a], vectors[c]),
        frozenset((b, c)): angular_distance(vectors[b], vectors[c]),
    }


def signature_close(lhs: tuple[float, float, float], rhs: tuple[float, float, float], tolerance_rad: float) -> bool:
    return all(abs(a - b) <= tolerance_rad for a, b in zip(lhs, rhs, strict=True))


def vote_assignments(
    observation_vectors: list[np.ndarray],
    catalog: list[tuple[str, np.ndarray]],
    tolerance_rad: float,
    max_catalog_stars: int,
    max_observation_triangles: int,
) -> tuple[dict[int, str], list[dict[str, object]]]:
    catalog = catalog[:max_catalog_stars]
    catalog_vectors = [entry[1] for entry in catalog]

    catalog_triangles: list[tuple[tuple[float, float, float], tuple[int, int, int]]] = []
    for indices in itertools.combinations(range(len(catalog_vectors)), 3):
        catalog_triangles.append((triangle_signature(catalog_vectors, indices), indices))

    votes: dict[tuple[int, int], int] = {}
    matches: list[dict[str, object]] = []
    observation_triangles = list(itertools.combinations(range(len(observation_vectors)), 3))
    observation_triangles = observation_triangles[:max_observation_triangles]
    for obs_indices in observation_triangles:
        obs_signature = triangle_signature(observation_vectors, obs_indices)
        for cat_signature, cat_indices in catalog_triangles:
            if not signature_close(obs_signature, cat_signature, tolerance_rad):
                continue
            obs_edges = triangle_edges(observation_vectors, obs_indices)
            # Try all vertex permutations because sorted edge signatures lose vertex labels.
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
                if all(
                    abs(obs_edges[key] - predicted_edges[key]) <= tolerance_rad
                    for key in obs_edges
                ):
                    for obs_index, cat_index in zip(obs_indices, permuted_catalog, strict=True):
                        votes[(obs_index, cat_index)] = votes.get((obs_index, cat_index), 0) + 1
                    matches.append(
                        {
                            "observation_indices": list(obs_indices),
                            "catalog_ids": [catalog[index][0] for index in permuted_catalog],
                        }
                    )

    assignments: dict[int, str] = {}
    used_catalog_indices: set[int] = set()
    ranked_votes = sorted(
        ((vote_count, obs_index, cat_index) for (obs_index, cat_index), vote_count in votes.items()),
        reverse=True,
    )
    for _vote_count, obs_index, cat_index in ranked_votes:
        if obs_index in assignments or cat_index in used_catalog_indices:
            continue
        assignments[obs_index] = catalog[cat_index][0]
        used_catalog_indices.add(cat_index)
    return assignments, matches


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observations", type=Path, required=True, help="CSV with columns u,v")
    parser.add_argument("--catalog", type=Path, required=True, help="CSV with columns id,x,y,z")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fx", type=float, required=True)
    parser.add_argument("--fy", type=float, required=True)
    parser.add_argument("--cx", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument("--tolerance-arcsec", type=float, default=120.0)
    parser.add_argument("--max-catalog-stars", type=int, default=80)
    parser.add_argument("--max-observation-triangles", type=int, default=200)
    args = parser.parse_args()

    observations = load_unlabeled_observations(args.observations, args.fx, args.fy, args.cx, args.cy)
    catalog = load_catalog(args.catalog)
    tolerance_rad = math.radians(args.tolerance_arcsec / 3600.0)
    assignments, matches = vote_assignments(
        observations,
        catalog,
        tolerance_rad,
        args.max_catalog_stars,
        args.max_observation_triangles,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["observation_index", "id"])
        for obs_index, star_id in sorted(assignments.items()):
            writer.writerow([obs_index, star_id])

    metadata = {
        "assigned_observations": len(assignments),
        "triangle_matches": len(matches),
        "tolerance_arcsec": args.tolerance_arcsec,
        "max_catalog_stars": args.max_catalog_stars,
        "max_observation_triangles": args.max_observation_triangles,
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
