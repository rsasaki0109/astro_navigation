#!/usr/bin/env python3
"""Convert RA/Dec star catalogs to star_tracker_attitude unit-vector CSV."""

from __future__ import annotations

import argparse
import csv
import gzip
import math
from pathlib import Path
from typing import TextIO


def open_text(path: Path) -> TextIO:
    if path.suffix == ".gz":
        return gzip.open(path, "rt", newline="", encoding="utf-8")
    return path.open(newline="", encoding="utf-8")


def direction_from_ra_dec(ra: float, dec: float, ra_unit: str) -> tuple[float, float, float, float, float]:
    ra_deg = ra * 15.0 if ra_unit == "hours" else ra
    dec_deg = dec
    ra_rad = math.radians(ra_deg)
    dec_rad = math.radians(dec_deg)
    cos_dec = math.cos(dec_rad)
    return (
        cos_dec * math.cos(ra_rad),
        cos_dec * math.sin(ra_rad),
        math.sin(dec_rad),
        ra_deg,
        dec_deg,
    )


def best_id(row: dict[str, str], id_columns: list[str], fallback_index: int) -> str:
    for column in id_columns:
        value = row.get(column, "").strip()
        if value:
            return value.replace(" ", "_")
    return f"star_{fallback_index:06d}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--format", choices=["hyg", "generic"], default="hyg")
    parser.add_argument("--ra-column", default="ra")
    parser.add_argument("--dec-column", default="dec")
    parser.add_argument("--mag-column", default="mag")
    parser.add_argument("--id-columns", nargs="+", default=["proper", "hip", "hd", "hr", "id"])
    parser.add_argument("--ra-unit", choices=["hours", "degrees"], default="hours")
    parser.add_argument("--max-magnitude", type=float, default=6.5)
    parser.add_argument("--limit", type=int, help="maximum number of output stars after filtering")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with open_text(args.input) as handle, args.output.open("w", newline="", encoding="utf-8") as out:
        reader = csv.DictReader(handle)
        writer = csv.writer(out)
        writer.writerow(["id", "x", "y", "z", "mag", "ra_deg", "dec_deg", "pmra_mas_yr", "pmdec_mas_yr"])
        for index, row in enumerate(reader):
            try:
                ra = float(row[args.ra_column])
                dec = float(row[args.dec_column])
                mag = float(row[args.mag_column]) if row.get(args.mag_column, "") else math.inf
            except (KeyError, ValueError):
                continue
            if not math.isfinite(ra) or not math.isfinite(dec) or mag > args.max_magnitude:
                continue
            if args.format == "hyg" and row.get("proper") == "Sol":
                continue
            try:
                pmra = float(row.get("pmra", "0.0") or 0.0)
            except ValueError:
                pmra = 0.0
            try:
                pmdec = float(row.get("pmdec", "0.0") or 0.0)
            except ValueError:
                pmdec = 0.0
            x, y, z, ra_deg, dec_deg = direction_from_ra_dec(ra, dec, args.ra_unit)
            writer.writerow(
                [
                    best_id(row, args.id_columns, index),
                    f"{x:.12f}",
                    f"{y:.12f}",
                    f"{z:.12f}",
                    f"{mag:.6f}",
                    f"{ra_deg:.9f}",
                    f"{dec_deg:.9f}",
                    f"{pmra:.6f}",
                    f"{pmdec:.6f}",
                ]
            )
            written += 1
            if args.limit and written >= args.limit:
                break
    print(f"wrote {written} stars to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

