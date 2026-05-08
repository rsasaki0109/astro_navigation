#!/usr/bin/env python3
"""Render a synthetic star tracker exposure from a public catalog.

For each catalog star inside the camera frustum:
- project to (u, v) via the pinhole intrinsics,
- place a Gaussian PSF spot with peak intensity proportional to flux
  (10^(-0.4 * mag)),
- accumulate into a 16-bit floating image,
- add Gaussian read noise scaled by --read-noise-electrons,
- clamp + quantize to 8-bit PNG.

The companion `centroid_stars_from_image.py` consumes the PNG to recover
(u, v) centroids; downstream `identify_stars_with_pair_index.py` then runs
lost-in-space ID end-to-end without ever using the truth labels.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from generate_star_tracker_case import normalize, quaternion_from_rotation, rotation_from_euler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--output-image", type=Path, required=True)
    parser.add_argument("--output-truth", type=Path, required=True, help="CSV of (id, u, v) truth")
    parser.add_argument("--output-meta", type=Path, default=None, help="JSON of attitude/intrinsics")
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
        "--psf-sigma-px",
        type=float,
        default=1.0,
        help="Gaussian PSF standard deviation in pixels.",
    )
    parser.add_argument(
        "--limiting-magnitude",
        type=float,
        default=7.0,
        help="Stars dimmer than this contribute negligibly. Above the limit, peak intensity falls off "
        "as 10^(-0.4 * (mag - limiting)).",
    )
    parser.add_argument(
        "--peak-electrons-at-limit",
        type=float,
        default=80.0,
        help="Peak PSF intensity (in electrons) for a star exactly at --limiting-magnitude. Brighter "
        "stars saturate proportionally faster.",
    )
    parser.add_argument(
        "--read-noise-electrons",
        type=float,
        default=8.0,
        help="Gaussian read noise sigma (electrons) added per pixel.",
    )
    parser.add_argument(
        "--background-electrons",
        type=float,
        default=20.0,
        help="Constant sky background (electrons) added to every pixel before noise.",
    )
    parser.add_argument(
        "--saturation-electrons",
        type=float,
        default=4096.0,
        help="Pixel saturation cap. Stars brighter than this clip at the well depth.",
    )
    parser.add_argument("--seed", type=int, default=11)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    rotation_camera_inertial = rotation_from_euler(args.yaw_deg, args.pitch_deg, args.roll_deg)
    image = np.full((args.height, args.width), args.background_electrons, dtype=np.float64)

    truth_rows: list[tuple[str, float, float, float]] = []
    psf_radius = max(3, int(np.ceil(3.5 * args.psf_sigma_px)))
    sigma_sq = args.psf_sigma_px * args.psf_sigma_px

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
            margin = args.psf_sigma_px * 4.0
            if not (margin <= u < args.width - margin and margin <= v < args.height - margin):
                continue
            try:
                mag = float(row.get("mag", "99"))
            except ValueError:
                mag = 99.0
            peak = args.peak_electrons_at_limit * (10.0 ** (-0.4 * (mag - args.limiting_magnitude)))
            if peak < 0.5:
                continue
            u_int = int(round(u))
            v_int = int(round(v))
            ulo = max(0, u_int - psf_radius)
            uhi = min(args.width, u_int + psf_radius + 1)
            vlo = max(0, v_int - psf_radius)
            vhi = min(args.height, v_int + psf_radius + 1)
            uu, vv = np.meshgrid(np.arange(ulo, uhi), np.arange(vlo, vhi))
            spot = peak * np.exp(-((uu - u) ** 2 + (vv - v) ** 2) / (2.0 * sigma_sq))
            image[vlo:vhi, ulo:uhi] += spot
            truth_rows.append((row["id"], u, v, mag))

    image += rng.normal(0.0, args.read_noise_electrons, image.shape)
    image = np.clip(image, 0.0, args.saturation_electrons)

    # Quantize to 8 bit, scaling so the saturation cap maps to 255.
    quantized = np.clip(image / args.saturation_electrons * 255.0, 0.0, 255.0).astype(np.uint8)

    args.output_image.parent.mkdir(parents=True, exist_ok=True)
    import cv2
    cv2.imwrite(str(args.output_image), quantized)

    args.output_truth.parent.mkdir(parents=True, exist_ok=True)
    with args.output_truth.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "u", "v", "mag"])
        for star_id, u, v, mag in truth_rows:
            writer.writerow([star_id, f"{u:.6f}", f"{v:.6f}", f"{mag:.4f}"])

    if args.output_meta:
        q = normalize(quaternion_from_rotation(rotation_camera_inertial)).tolist()
        meta = {
            "catalog": str(args.catalog),
            "image": str(args.output_image),
            "intrinsics": {"fx": args.fx, "fy": args.fy, "cx": args.cx, "cy": args.cy},
            "size": [args.width, args.height],
            "yaw_deg": args.yaw_deg,
            "pitch_deg": args.pitch_deg,
            "roll_deg": args.roll_deg,
            "q_camera_inertial_xyzw": q,
            "psf_sigma_px": args.psf_sigma_px,
            "limiting_magnitude": args.limiting_magnitude,
            "peak_electrons_at_limit": args.peak_electrons_at_limit,
            "read_noise_electrons": args.read_noise_electrons,
            "background_electrons": args.background_electrons,
            "saturation_electrons": args.saturation_electrons,
            "rendered_stars": len(truth_rows),
            "seed": args.seed,
        }
        args.output_meta.parent.mkdir(parents=True, exist_ok=True)
        args.output_meta.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")

    print(
        f"wrote {args.output_image} ({len(truth_rows)} rendered stars), "
        f"truth at {args.output_truth}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
