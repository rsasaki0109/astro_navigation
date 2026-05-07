#!/usr/bin/env python3
"""Generate a synthetic identified-star attitude case."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def normalize(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    trace = np.trace(rotation)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[1, 0] - rotation[0, 1]) / s,
                0.25 * s,
            ]
        )
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return np.array(
            [
                0.25 * s,
                (rotation[0, 1] + rotation[1, 0]) / s,
                (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[2, 1] - rotation[1, 2]) / s,
            ]
        )
    if index == 1:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return np.array(
            [
                (rotation[0, 1] + rotation[1, 0]) / s,
                0.25 * s,
                (rotation[1, 2] + rotation[2, 1]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
            ]
        )
    s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return np.array(
        [
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            0.25 * s,
            (rotation[1, 0] - rotation[0, 1]) / s,
        ]
    )


def rotation_from_euler(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    rz = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )
    ry = np.array(
        [[math.cos(pitch), 0.0, math.sin(pitch)], [0.0, 1.0, 0.0], [-math.sin(pitch), 0.0, math.cos(pitch)]]
    )
    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, math.cos(roll), -math.sin(roll)], [0.0, math.sin(roll), math.cos(roll)]]
    )
    return rz @ ry @ rx


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stars", type=int, default=30)
    parser.add_argument("--noise-px", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--fx", type=float, default=1000.0)
    parser.add_argument("--fy", type=float, default=1000.0)
    parser.add_argument("--cx", type=float, default=512.0)
    parser.add_argument("--cy", type=float, default=512.0)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--yaw-deg", type=float, default=12.0)
    parser.add_argument("--pitch-deg", type=float, default=-7.0)
    parser.add_argument("--roll-deg", type=float, default=4.0)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rotation_camera_inertial = rotation_from_euler(args.yaw_deg, args.pitch_deg, args.roll_deg)
    q_xyzw = normalize(quaternion_from_rotation(rotation_camera_inertial))

    catalog_rows = ["id,x,y,z"]
    observation_rows = ["id,u,v"]
    for index in range(args.stars):
        margin = 80.0
        u = rng.uniform(margin, args.width - margin)
        v = rng.uniform(margin, args.height - margin)
        bearing_camera = normalize(np.array([(u - args.cx) / args.fx, (v - args.cy) / args.fy, 1.0]))
        direction_inertial = rotation_camera_inertial.T @ bearing_camera
        noisy_u = u + rng.normal(0.0, args.noise_px)
        noisy_v = v + rng.normal(0.0, args.noise_px)
        star_id = f"star_{index:04d}"
        catalog_rows.append(
            f"{star_id},{direction_inertial[0]:.12f},{direction_inertial[1]:.12f},{direction_inertial[2]:.12f}"
        )
        observation_rows.append(f"{star_id},{noisy_u:.6f},{noisy_v:.6f}")

    (args.output_dir / "catalog.csv").write_text("\n".join(catalog_rows) + "\n", encoding="utf-8")
    (args.output_dir / "observations.csv").write_text("\n".join(observation_rows) + "\n", encoding="utf-8")
    truth = {
        "q_camera_inertial_xyzw": q_xyzw.tolist(),
        "yaw_deg": args.yaw_deg,
        "pitch_deg": args.pitch_deg,
        "roll_deg": args.roll_deg,
        "stars": args.stars,
        "noise_px": args.noise_px,
        "seed": args.seed,
        "intrinsics": {"fx": args.fx, "fy": args.fy, "cx": args.cx, "cy": args.cy},
    }
    (args.output_dir / "truth.json").write_text(json.dumps(truth, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

