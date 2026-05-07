#!/usr/bin/env python3
"""Generate synthetic observations by projecting a converted public star catalog."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from generate_star_tracker_case import normalize, quaternion_from_rotation, rotation_from_euler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True, help="converted id,x,y,z star catalog")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stars", type=int, default=30)
    parser.add_argument("--noise-px", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--fx", type=float, default=1000.0)
    parser.add_argument("--fy", type=float, default=1000.0)
    parser.add_argument("--cx", type=float, default=512.0)
    parser.add_argument("--cy", type=float, default=512.0)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    rotation_camera_inertial = rotation_from_euler(args.yaw_deg, args.pitch_deg, args.roll_deg)
    candidates: list[tuple[str, np.ndarray, float, float, float]] = []
    with args.catalog.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            direction_inertial = normalize(
                np.array([float(row["x"]), float(row["y"]), float(row["z"])], dtype=float)
            )
            direction_camera = rotation_camera_inertial @ direction_inertial
            if direction_camera[2] <= 0.0:
                continue
            u = args.fx * direction_camera[0] / direction_camera[2] + args.cx
            v = args.fy * direction_camera[1] / direction_camera[2] + args.cy
            margin = 8.0
            if margin <= u < args.width - margin and margin <= v < args.height - margin:
                mag = float(row.get("mag", "99"))
                candidates.append((row["id"], direction_inertial, mag, u, v))

    if len(candidates) < args.stars:
        raise RuntimeError(f"only {len(candidates)} catalog stars are visible; need {args.stars}")

    candidates.sort(key=lambda item: item[2])
    selected = candidates[: max(args.stars * 3, args.stars)]
    chosen_indices = rng.choice(len(selected), size=args.stars, replace=False)
    chosen = [selected[int(index)] for index in chosen_indices]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    catalog_rows = ["id,x,y,z"]
    observation_rows = ["id,u,v"]
    for star_id, direction, _mag, u, v in chosen:
        catalog_rows.append(f"{star_id},{direction[0]:.12f},{direction[1]:.12f},{direction[2]:.12f}")
        observation_rows.append(
            f"{star_id},{u + rng.normal(0.0, args.noise_px):.6f},{v + rng.normal(0.0, args.noise_px):.6f}"
        )

    (args.output_dir / "catalog.csv").write_text("\n".join(catalog_rows) + "\n", encoding="utf-8")
    (args.output_dir / "observations.csv").write_text(
        "\n".join(observation_rows) + "\n", encoding="utf-8"
    )
    q_xyzw = normalize(quaternion_from_rotation(rotation_camera_inertial))
    truth = {
        "source_catalog": str(args.catalog),
        "q_camera_inertial_xyzw": q_xyzw.tolist(),
        "yaw_deg": args.yaw_deg,
        "pitch_deg": args.pitch_deg,
        "roll_deg": args.roll_deg,
        "stars": args.stars,
        "noise_px": args.noise_px,
        "seed": args.seed,
        "visible_candidates": len(candidates),
        "intrinsics": {"fx": args.fx, "fy": args.fy, "cx": args.cx, "cy": args.cy},
    }
    (args.output_dir / "truth.json").write_text(json.dumps(truth, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output_dir} from {len(candidates)} visible catalog stars")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

