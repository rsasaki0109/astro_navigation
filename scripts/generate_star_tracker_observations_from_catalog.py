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
    parser.add_argument(
        "--limiting-magnitude",
        type=float,
        default=None,
        help="Magnitude-aware probabilistic detection. p(detect) = 1 / (1 + exp((mag - "
        "limiting_magnitude) / mag_softness)). Default disabled — falls back to top-3N + uniform pick.",
    )
    parser.add_argument(
        "--mag-softness",
        type=float,
        default=0.5,
        help="Sigmoid width for --limiting-magnitude probabilistic detection.",
    )
    parser.add_argument(
        "--noise-mag-reference",
        type=float,
        default=None,
        help="If set, scale per-star centroid noise as noise_px * 10^(0.4 * (mag - reference)) "
        "to model brighter-stars-have-lower-noise. Default disabled — uniform Gaussian noise.",
    )
    parser.add_argument(
        "--noise-mag-cap-px",
        type=float,
        default=10.0,
        help="Upper cap on per-star centroid noise sigma when --noise-mag-reference is active.",
    )
    parser.add_argument(
        "--apply-proper-motion-years",
        type=float,
        default=0.0,
        help="If non-zero, drift each catalog direction by `pmra_mas_yr * years` in RA and "
        "`pmdec_mas_yr * years` in Dec before projecting to the camera. Tests catalog freshness.",
    )
    parser.add_argument(
        "--distortion-k1",
        type=float,
        default=0.0,
        help="Brown-Conrady radial distortion coefficient k1 applied at projection (forward). "
        "The identifier does not undistort, so this exercises lost-in-space robustness to an "
        "uncalibrated lens.",
    )
    parser.add_argument(
        "--distortion-k2",
        type=float,
        default=0.0,
        help="Brown-Conrady radial distortion coefficient k2 (forward).",
    )
    parser.add_argument(
        "--distortion-p1",
        type=float,
        default=0.0,
        help="Brown-Conrady tangential distortion coefficient p1 (forward).",
    )
    parser.add_argument(
        "--distortion-p2",
        type=float,
        default=0.0,
        help="Brown-Conrady tangential distortion coefficient p2 (forward).",
    )
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    rotation_camera_inertial = rotation_from_euler(args.yaw_deg, args.pitch_deg, args.roll_deg)
    candidates: list[tuple[str, np.ndarray, float, float, float]] = []
    pm_active = args.apply_proper_motion_years != 0.0
    mas_to_rad = math.radians(1.0 / 3600.0 / 1000.0)
    with args.catalog.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            direction_catalog = normalize(
                np.array([float(row["x"]), float(row["y"]), float(row["z"])], dtype=float)
            )
            if pm_active:
                # Drift the catalog direction by N years of proper motion. Convert (ra, dec) deltas
                # in milliarcseconds-per-year * years into a small rotation in the local tangent
                # plane. For our drift magnitudes (~mas to ~arcsec), the linearization is fine.
                try:
                    pmra = float(row.get("pmra_mas_yr", "0.0") or 0.0)
                    pmdec = float(row.get("pmdec_mas_yr", "0.0") or 0.0)
                except ValueError:
                    pmra = 0.0
                    pmdec = 0.0
                ra_rad = math.atan2(direction_catalog[1], direction_catalog[0])
                dec_rad = math.asin(max(-1.0, min(1.0, direction_catalog[2])))
                delta_ra = pmra * args.apply_proper_motion_years * mas_to_rad
                delta_dec = pmdec * args.apply_proper_motion_years * mas_to_rad
                new_ra = ra_rad + delta_ra
                new_dec = max(-math.pi / 2, min(math.pi / 2, dec_rad + delta_dec))
                cd = math.cos(new_dec)
                direction_inertial = normalize(
                    np.array([cd * math.cos(new_ra), cd * math.sin(new_ra), math.sin(new_dec)])
                )
            else:
                direction_inertial = direction_catalog
            direction_camera = rotation_camera_inertial @ direction_inertial
            if direction_camera[2] <= 0.0:
                continue
            u = args.fx * direction_camera[0] / direction_camera[2] + args.cx
            v = args.fy * direction_camera[1] / direction_camera[2] + args.cy
            margin = 8.0
            if margin <= u < args.width - margin and margin <= v < args.height - margin:
                mag = float(row.get("mag", "99"))
                candidates.append((row["id"], direction_catalog, mag, u, v))

    if len(candidates) < args.stars:
        raise RuntimeError(f"only {len(candidates)} catalog stars are visible; need {args.stars}")

    if args.limiting_magnitude is not None:
        magnitudes_arr = np.asarray([candidate[2] for candidate in candidates], dtype=float)
        detection_probs = 1.0 / (1.0 + np.exp((magnitudes_arr - args.limiting_magnitude) / args.mag_softness))
        weights = detection_probs / detection_probs.sum()
        if (detection_probs > 0.0).sum() < args.stars:
            raise RuntimeError(
                f"only {(detection_probs > 0.0).sum()} candidate stars have nonzero detection probability"
                f" at limiting_magnitude={args.limiting_magnitude}, mag_softness={args.mag_softness}; need {args.stars}"
            )
        chosen_indices = rng.choice(len(candidates), size=args.stars, replace=False, p=weights)
        chosen = [candidates[int(index)] for index in chosen_indices]
    else:
        candidates.sort(key=lambda item: item[2])
        selected = candidates[: max(args.stars * 3, args.stars)]
        chosen_indices = rng.choice(len(selected), size=args.stars, replace=False)
        chosen = [selected[int(index)] for index in chosen_indices]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    catalog_rows = ["id,x,y,z"]
    observation_rows = ["id,u,v,mag"]
    distortion_active = (
        args.distortion_k1 != 0.0 or args.distortion_k2 != 0.0
        or args.distortion_p1 != 0.0 or args.distortion_p2 != 0.0
    )
    for star_id, direction, mag, u, v in chosen:
        if args.noise_mag_reference is None:
            sigma_px = args.noise_px
        else:
            sigma_px = min(
                args.noise_px * (10.0 ** (0.4 * (mag - args.noise_mag_reference))),
                args.noise_mag_cap_px,
            )
        if distortion_active:
            # Forward Brown-Conrady: apply distortion to the ideal pixel projection so the
            # identifier sees the lens-distorted measurement. Using the same fx/fy/cx/cy as
            # the projection step is intentional; the lost-in-space identifier does not
            # consume any calibration knowledge.
            x_n = (u - args.cx) / args.fx
            y_n = (v - args.cy) / args.fy
            r2 = x_n * x_n + y_n * y_n
            radial = 1.0 + args.distortion_k1 * r2 + args.distortion_k2 * r2 * r2
            x_t = 2.0 * args.distortion_p1 * x_n * y_n + args.distortion_p2 * (r2 + 2.0 * x_n * x_n)
            y_t = args.distortion_p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * args.distortion_p2 * x_n * y_n
            x_d = x_n * radial + x_t
            y_d = y_n * radial + y_t
            u = args.fx * x_d + args.cx
            v = args.fy * y_d + args.cy
        catalog_rows.append(f"{star_id},{direction[0]:.12f},{direction[1]:.12f},{direction[2]:.12f}")
        observation_rows.append(
            f"{star_id},{u + rng.normal(0.0, sigma_px):.6f},"
            f"{v + rng.normal(0.0, sigma_px):.6f},{mag:.4f}"
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
        "distortion": {
            "k1": args.distortion_k1,
            "k2": args.distortion_k2,
            "p1": args.distortion_p1,
            "p2": args.distortion_p2,
        },
    }
    (args.output_dir / "truth.json").write_text(json.dumps(truth, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output_dir} from {len(candidates)} visible catalog stars")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

