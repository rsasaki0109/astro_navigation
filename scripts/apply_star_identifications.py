#!/usr/bin/env python3
"""Apply lost-in-space ID assignments to unlabeled observations."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unlabeled", type=Path, required=True)
    parser.add_argument("--assignments", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    observations = list(csv.DictReader(args.unlabeled.open(newline="", encoding="utf-8")))
    assignments = {
        int(row["observation_index"]): row["id"]
        for row in csv.DictReader(args.assignments.open(newline="", encoding="utf-8"))
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "u", "v"])
        for index, row in enumerate(observations):
            if index in assignments:
                writer.writerow([assignments[index], row["u"], row["v"]])
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

