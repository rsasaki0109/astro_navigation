#!/usr/bin/env python3
"""Summarize best exposure choices from a POLAR suite summary.csv."""

from __future__ import annotations

import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path


SEQUENCE_RE = re.compile(r"View1_(?P<traverse>Traverse\d+)_L_(?P<exposure>\d+)ms")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary_csv", type=Path)
    parser.add_argument("--output-md", type=Path)
    args = parser.parse_args()

    rows = list(csv.DictReader(args.summary_csv.open(encoding="utf-8")))
    grouped: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        match = SEQUENCE_RE.match(row["sequence"])
        if not match:
            continue
        row["traverse"] = match.group("traverse")
        row["exposure_ms"] = match.group("exposure")
        grouped[(row["traverse"], row["method"])].append(row)

    lines = [
        "# Exposure Sweep Summary",
        "",
        "| Traverse | Method | Best exposure [ms] | Frames OK | ATE RMSE [m] | RPE trans RMSE [m] |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for key in sorted(grouped):
        candidates = grouped[key]
        best = min(
            candidates,
            key=lambda row: (-int(row["ok_or_initialized"]), float(row["ate_rmse_m"])),
        )
        lines.append(
            "| {traverse} | {method} | {exposure_ms} | {ok_or_initialized}/{frames} | "
            "{ate_rmse_m} | {rpe_translation_rmse_m} |".format(**best)
        )

    text = "\n".join(lines) + "\n"
    print(text)
    if args.output_md:
        args.output_md.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
