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
    args = parser.parse_args()

    rng = random.Random(args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.input.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if args.drop_count > 0:
        drop = set(rng.sample(range(len(rows)), k=min(args.drop_count, len(rows))))
        rows = [row for index, row in enumerate(rows) if index not in drop]
    real_observations = [(float(row["u"]), float(row["v"])) for row in rows]
    hot_pixel_rng = random.Random(args.hot_pixel_seed)
    hot_pixels = [
        (hot_pixel_rng.uniform(0.0, args.width), hot_pixel_rng.uniform(0.0, args.height))
        for _ in range(max(args.hot_pixel_count, 1))
    ]
    for _ in range(args.false_count):
        roll = rng.random()
        if args.hot_pixel_fraction > 0.0 and roll < args.hot_pixel_fraction:
            hu, hv = rng.choice(hot_pixels)
            u = hu + rng.gauss(0.0, args.hot_pixel_sigma_px)
            v = hv + rng.gauss(0.0, args.hot_pixel_sigma_px)
        elif real_observations and (
            args.false_near_fraction > 0.0
            and roll < args.hot_pixel_fraction + (1.0 - args.hot_pixel_fraction) * args.false_near_fraction
        ):
            anchor_u, anchor_v = rng.choice(real_observations)
            u = anchor_u + rng.gauss(0.0, args.false_near_sigma_px)
            v = anchor_v + rng.gauss(0.0, args.false_near_sigma_px)
        else:
            u = rng.uniform(0.0, args.width)
            v = rng.uniform(0.0, args.height)
        u = min(max(u, 0.0), args.width)
        v = min(max(v, 0.0), args.height)
        rows.append({"u": f"{u:.6f}", "v": f"{v:.6f}"})
    rng.shuffle(rows)

    with args.output.open("w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
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
