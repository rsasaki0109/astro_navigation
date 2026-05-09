#!/usr/bin/env python3
"""Drop star IDs from an observations CSV for lost-in-space experiments."""

from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--truth-output", type=Path, help="optional CSV mapping shuffled observation_index to original id")
    parser.add_argument("--drop-count", type=int, default=0)
    parser.add_argument("--false-count", type=int, default=0)
    parser.add_argument("--width", type=float, default=1024.0)
    parser.add_argument("--height", type=float, default=1024.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--false-near-fraction",
        type=float,
        default=0.0,
        help="Fraction of false detections placed near a real observation rather than uniformly. "
        "0.0 (default) preserves the uniform-random behavior; 1.0 puts every false detection within "
        "--false-near-sigma-px of a randomly chosen real observation.",
    )
    parser.add_argument(
        "--false-near-sigma-px",
        type=float,
        default=20.0,
        help="Gaussian std-dev (in pixels) used to offset near-star false detections.",
    )
    parser.add_argument(
        "--hot-pixel-count",
        type=int,
        default=8,
        help="Number of fixed image-coordinate 'hot pixel' positions sampled when "
        "--hot-pixel-fraction > 0. Hot pixel layout is deterministic given --hot-pixel-seed.",
    )
    parser.add_argument(
        "--hot-pixel-seed",
        type=int,
        default=99,
        help="RNG seed for sampling hot pixel positions; kept separate from --seed so the same "
        "hot-pixel layout can be reused across fixtures.",
    )
    parser.add_argument(
        "--hot-pixel-fraction",
        type=float,
        default=0.0,
        help="Fraction of false detections placed at a hot pixel position rather than uniform or "
        "near-star. Evaluated AFTER --false-near-fraction; the three modes partition the false "
        "detection budget as: hot, then near-real (of the remainder), then uniform random.",
    )
    parser.add_argument(
        "--hot-pixel-sigma-px",
        type=float,
        default=1.0,
        help="Gaussian std-dev (in pixels) applied around each hot pixel position. Default 1.0 "
        "models hot pixels with a tight centroid uncertainty.",
    )
    parser.add_argument(
        "--false-edge-fraction",
        type=float,
        default=0.0,
        help="Fraction of false detections placed in a band within --false-edge-band-px of any "
        "image edge. Models lens-flare / sensor-edge artifacts that cluster around the perimeter. "
        "Evaluated AFTER --hot-pixel-fraction and BEFORE --false-near-fraction in the partition.",
    )
    parser.add_argument(
        "--false-edge-band-px",
        type=float,
        default=24.0,
        help="Band width (pixels) for edge-biased false detections. Each pick chooses a side "
        "uniformly, then samples distance-from-edge ∈ [0, band] and along-edge ∈ [0, side_length].",
    )
    parser.add_argument(
        "--false-mag-mean",
        type=float,
        default=5.0,
        help="Mean of the Gaussian magnitude assigned to false detections when the input CSV "
        "carries a `mag` column. Ignored when the input has no mag column (output drops mag too).",
    )
    parser.add_argument(
        "--false-mag-std",
        type=float,
        default=1.0,
        help="Std-dev of the Gaussian magnitude assigned to false detections.",
    )
    parser.add_argument(
        "--false-mag-hot-mean",
        type=float,
        default=None,
        help="Mean magnitude for hot-pixel false detections specifically (read-noise spikes "
        "typically saturate and look brighter than ambient noise). Default None reuses "
        "--false-mag-mean for all branches (back-compat).",
    )
    parser.add_argument(
        "--false-mag-hot-std",
        type=float,
        default=None,
        help="Std-dev for hot-pixel false detection magnitudes. Default None reuses "
        "--false-mag-std for all branches.",
    )
    args = parser.parse_args()

    rng = random.Random(args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.input.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        carries_mag = reader.fieldnames is not None and "mag" in reader.fieldnames
    if args.drop_count > 0:
        drop = set(rng.sample(range(len(rows)), k=min(args.drop_count, len(rows))))
        rows = [row for index, row in enumerate(rows) if index not in drop]
    real_observations = [(float(row["u"]), float(row["v"])) for row in rows]
    hot_pixel_rng = random.Random(args.hot_pixel_seed)
    hot_pixels = [
        (hot_pixel_rng.uniform(0.0, args.width), hot_pixel_rng.uniform(0.0, args.height))
        for _ in range(max(args.hot_pixel_count, 1))
    ]
    # Cumulative thresholds for the four-mode partition: hot, edge, near, uniform.
    # Each fraction consumes its share of the *remaining* probability mass, so the modes
    # never overlap and the residual always falls back to uniform.
    threshold_hot = args.hot_pixel_fraction
    threshold_edge = threshold_hot + (1.0 - threshold_hot) * args.false_edge_fraction
    threshold_near = threshold_edge + (1.0 - threshold_edge) * args.false_near_fraction

    for _ in range(args.false_count):
        roll = rng.random()
        false_branch = "uniform"
        if args.hot_pixel_fraction > 0.0 and roll < threshold_hot:
            false_branch = "hot"
            hu, hv = rng.choice(hot_pixels)
            u = hu + rng.gauss(0.0, args.hot_pixel_sigma_px)
            v = hv + rng.gauss(0.0, args.hot_pixel_sigma_px)
        elif args.false_edge_fraction > 0.0 and roll < threshold_edge:
            # Pick a side (top/bottom/left/right) uniformly, then sample within the band.
            side = rng.randrange(4)
            depth = rng.uniform(0.0, args.false_edge_band_px)
            if side == 0:  # top
                u = rng.uniform(0.0, args.width)
                v = depth
            elif side == 1:  # bottom
                u = rng.uniform(0.0, args.width)
                v = args.height - depth
            elif side == 2:  # left
                u = depth
                v = rng.uniform(0.0, args.height)
            else:  # right
                u = args.width - depth
                v = rng.uniform(0.0, args.height)
        elif real_observations and args.false_near_fraction > 0.0 and roll < threshold_near:
            anchor_u, anchor_v = rng.choice(real_observations)
            u = anchor_u + rng.gauss(0.0, args.false_near_sigma_px)
            v = anchor_v + rng.gauss(0.0, args.false_near_sigma_px)
        else:
            u = rng.uniform(0.0, args.width)
            v = rng.uniform(0.0, args.height)
        u = min(max(u, 0.0), args.width)
        v = min(max(v, 0.0), args.height)
        false_row = {"u": f"{u:.6f}", "v": f"{v:.6f}"}
        if carries_mag:
            if false_branch == "hot" and args.false_mag_hot_mean is not None:
                mag_mean = args.false_mag_hot_mean
                mag_std = (
                    args.false_mag_hot_std if args.false_mag_hot_std is not None
                    else args.false_mag_std
                )
            else:
                mag_mean = args.false_mag_mean
                mag_std = args.false_mag_std
            false_mag = rng.gauss(mag_mean, mag_std)
            false_row["mag"] = f"{false_mag:.4f}"
        rows.append(false_row)
    rng.shuffle(rows)

    with args.output.open("w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
        if carries_mag:
            writer.writerow(["u", "v", "mag"])
            for row in rows:
                writer.writerow([row["u"], row["v"], row.get("mag", "")])
        else:
            writer.writerow(["u", "v"])
            for row in rows:
                writer.writerow([row["u"], row["v"]])
    if args.truth_output:
        args.truth_output.parent.mkdir(parents=True, exist_ok=True)
        with args.truth_output.open("w", newline="", encoding="utf-8") as out:
            writer = csv.writer(out)
            writer.writerow(["observation_index", "id"])
            for index, row in enumerate(rows):
                writer.writerow([index, row.get("id", "")])
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
