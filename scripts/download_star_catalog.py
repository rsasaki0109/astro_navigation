#!/usr/bin/env python3
"""Download public star catalogs for star tracker experiments."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import urllib.request
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class StarCatalogSource:
    name: str
    url: str
    citation: str
    license_note: str


CATALOGS: dict[str, StarCatalogSource] = {
    "hyg-v42": StarCatalogSource(
        name="HYG Database v4.2",
        url="https://codeberg.org/astronexus/hyg/media/branch/main/data/hyg/CURRENT/hyg_v42.csv.gz",
        citation="Astronexus HYG Database v4.2, compiled from Hipparcos, Yale Bright Star, and Gliese catalogs.",
        license_note="HYG data is public-domain style/open data; verify upstream CURRENT/LICENSE before redistribution.",
    )
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--catalog", choices=sorted(CATALOGS))
    parser.add_argument("--output", type=Path, default=Path("datasets/star_catalogs"))
    args = parser.parse_args()

    if args.list:
        for key, item in CATALOGS.items():
            print(f"{key}: {item.name}")
        return 0
    if not args.catalog:
        parser.error("--catalog is required unless --list is used")

    item = CATALOGS[args.catalog]
    root = args.output / args.catalog
    raw = root / "raw" / Path(urllib.request.urlparse(item.url).path).name
    raw.parent.mkdir(parents=True, exist_ok=True)
    if not raw.exists():
        print(f"downloading {item.url}")
        with urllib.request.urlopen(item.url) as response, raw.open("wb") as handle:
            shutil.copyfileobj(response, handle)
    else:
        print(f"using existing {raw}")

    manifest = asdict(item)
    manifest["raw"] = str(raw)
    manifest["sha256"] = sha256(raw)
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"catalog organized under {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

