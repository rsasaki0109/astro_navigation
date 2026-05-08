#!/usr/bin/env python3
"""Download and organize public lunar localization datasets.

The script intentionally keeps the registry explicit. Large datasets require
--confirm-large so accidental multi-GB downloads do not start in CI or shells.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import tarfile
import urllib.parse
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class DatasetItem:
    name: str
    url: str
    size_gb: float
    archive_type: str
    citation: str
    license_note: str


DATASETS: dict[str, DatasetItem] = {
    "polar-traverse-view1": DatasetItem(
        name="NASA POLAR Traverse View 1",
        url="https://ti.arc.nasa.gov/dataset/PolarTrav/downloads/View1.zip",
        size_gb=3.2,
        archive_type="zip",
        citation="Hansen, Wong, and Fong. POLAR Traverse Dataset. NASA Ames, 2023.",
        license_note="Public NASA dataset; verify NASA dataset page terms before redistribution.",
    ),
    "polar-traverse-gt": DatasetItem(
        name="NASA POLAR Traverse Ground Truth",
        url="https://ti.arc.nasa.gov/dataset/PolarTrav/downloads/GroundTruth.zip",
        size_gb=0.53,
        archive_type="zip",
        citation="Hansen, Wong, and Fong. POLAR Traverse Dataset. NASA Ames, 2023.",
        license_note="Public NASA dataset; verify NASA dataset page terms before redistribution.",
    ),
    "polar-stereo-terrain11": DatasetItem(
        name="NASA POLAR Stereo Terrain 11 Fresh Crater",
        url="https://ti.arc.nasa.gov/dataset/IRG_PolarDB/PolarDB_download/Terrain11_FreshCrater.zip",
        size_gb=0.90,
        archive_type="zip",
        citation="Wong et al. POLAR Stereo Dataset. NASA Ames, 2017.",
        license_note="Public NASA dataset; verify NASA dataset page terms before redistribution.",
    ),
    "lunarloc-rss2025-example": DatasetItem(
        name="LunarLoc RSS 2025 release example",
        url="https://github.com/mit-acl/lunarloc-data/archive/refs/heads/main.zip",
        size_gb=0.01,
        archive_type="zip",
        citation="Galliath et al. LunarLoc: Segment-Based Global Localization on the Moon, 2025.",
        license_note="MIT licensed repository; large .lac assets are published on GitHub releases.",
    ),
    "synthetic-lunar-terrain-metadata": DatasetItem(
        name="Synthetic Lunar Terrain Zenodo metadata",
        url="https://zenodo.org/api/records/13218780",
        size_gb=0.001,
        archive_type="json",
        citation="Maertens, Farries, Culton, and Chin. Synthetic Lunar Terrain, 2024.",
        license_note="Use Zenodo record metadata to select files and confirm file licenses.",
    ),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(url: str, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url) as response, output.open("wb") as handle:
        shutil.copyfileobj(response, handle)


def extract(archive: Path, destination: Path, archive_type: str) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    if archive_type == "zip":
        with zipfile.ZipFile(archive) as zf:
            zf.extractall(destination)
    elif archive_type in {"tar", "tar.gz", "tgz"}:
        with tarfile.open(archive) as tf:
            tf.extractall(destination, filter="data")
    elif archive_type == "json":
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(archive, destination / archive.name)
    else:
        raise ValueError(f"unsupported archive type: {archive_type}")


def write_manifest(dataset: DatasetItem, root: Path, archive: Path) -> None:
    manifest = asdict(dataset)
    manifest["archive"] = str(archive)
    manifest["sha256"] = sha256(archive)
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list registered datasets")
    parser.add_argument("--dataset", choices=sorted(DATASETS), help="dataset key to download")
    parser.add_argument("--output", type=Path, default=Path("datasets"), help="dataset root")
    parser.add_argument("--confirm-large", action="store_true", help="allow downloads larger than 1 GB")
    parser.add_argument("--no-extract", action="store_true", help="download only")
    args = parser.parse_args()

    if args.list:
        for key, item in DATASETS.items():
            print(f"{key}: {item.name} ({item.size_gb:g} GB)")
        return 0

    if not args.dataset:
        parser.error("--dataset is required unless --list is used")

    item = DATASETS[args.dataset]
    if item.size_gb > 1.0 and not args.confirm_large:
        print(
            f"{args.dataset} is approximately {item.size_gb:g} GB. "
            "Re-run with --confirm-large to download.",
            file=sys.stderr,
        )
        return 2

    root = args.output / args.dataset
    archive = root / "raw" / Path(urllib.parse.urlparse(item.url).path).name
    if not archive.exists():
        print(f"downloading {item.url}")
        download(item.url, archive)
    else:
        print(f"using existing archive {archive}")

    write_manifest(item, root, archive)
    if not args.no_extract:
        extract(archive, root / "extracted", item.archive_type)
    print(f"dataset organized under {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
