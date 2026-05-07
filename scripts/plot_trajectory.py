#!/usr/bin/env python3
"""Plot a TUM or CSV trajectory produced by lunar_visual_odometry."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_positions(path: Path) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    zs: list[float] = []
    if path.suffix == ".csv":
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                xs.append(float(row["tx"]))
                zs.append(float(row["tz"]))
    else:
        with path.open(encoding="utf-8") as handle:
            for line in handle:
                if not line.strip() or line.startswith("#"):
                    continue
                fields = line.split()
                xs.append(float(fields[1]))
                zs.append(float(fields[3]))
    return xs, zs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/trajectory.png"))
    args = parser.parse_args()

    xs, zs = load_positions(args.trajectory)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(7, 5))
    plt.plot(xs, zs, marker="o", linewidth=1)
    plt.xlabel("x [relative scale]")
    plt.ylabel("z [relative scale]")
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(args.output, dpi=160)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

