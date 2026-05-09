#!/usr/bin/env python3
"""Render an animated GIF of lost-in-space identification across multiple attitudes.

For each (yaw, pitch, roll) frame the script chains the existing pipeline:

    render_star_image.py  →  centroid_stars_from_image.py  →
        apps/lost_in_space_pair_id  (C++ identifier on the .bin index)

then composites colored circles on top of the rendered exposure:

    green  — correct assignment (matches the closest truth star within --match-px)
    red    — wrong assignment (assigned to a catalog ID that does not match truth)
    blue   — unassigned centroid

The frames are stitched into a looping GIF for the README. Per-frame costs are
dominated by `lost_in_space_pair_id` loading the .bin (~1 s for the 16 k mag8
catalog), so 6-12 frames is the practical sweet spot.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parent.parent


def run(cmd: list[str]) -> None:
    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(f"$ {' '.join(cmd)}\n{result.stdout}\n{result.stderr}\n")
        raise SystemExit(result.returncode)


def load_truth(truth_csv: Path) -> list[tuple[str, float, float]]:
    truth: list[tuple[str, float, float]] = []
    with truth_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth.append((row["id"], float(row["u"]), float(row["v"])))
    return truth


def load_observations(obs_csv: Path) -> list[tuple[float, float]]:
    obs: list[tuple[float, float]] = []
    with obs_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            obs.append((float(row["u"]), float(row["v"])))
    return obs


def load_assignments(assignments_csv: Path) -> dict[int, str]:
    out: dict[int, str] = {}
    with assignments_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            out[int(row["observation_index"])] = row["id"]
    return out


def closest_truth(u: float, v: float, truth: list[tuple[str, float, float]],
                  match_px: float) -> str | None:
    best_id: str | None = None
    best_d2 = match_px * match_px
    for star_id, tu, tv in truth:
        d2 = (u - tu) ** 2 + (v - tv) ** 2
        if d2 <= best_d2:
            best_d2 = d2
            best_id = star_id
    return best_id


def annotate_frame(image: np.ndarray,
                   observations: list[tuple[float, float]],
                   assignments: dict[int, str],
                   truth: list[tuple[str, float, float]],
                   match_px: float,
                   frame_label: str) -> tuple[np.ndarray, dict[str, int]]:
    out = image.copy()
    counts = {"correct": 0, "wrong": 0, "unassigned": 0}
    for obs_index, (u, v) in enumerate(observations):
        truth_id = closest_truth(u, v, truth, match_px)
        assigned = assignments.get(obs_index)
        if assigned is None:
            color = (255, 200, 80)  # blue (BGR)
            counts["unassigned"] += 1
        elif truth_id is not None and assigned == truth_id:
            color = (80, 220, 80)  # green
            counts["correct"] += 1
        else:
            color = (60, 60, 240)  # red
            counts["wrong"] += 1
        cv2.circle(out, (int(round(u)), int(round(v))), radius=12, color=color,
                   thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(out, frame_label, (16, 36), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (240, 240, 240), 2, cv2.LINE_AA)
    cv2.putText(out, frame_label, (16, 36), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (40, 40, 40), 1, cv2.LINE_AA)
    legend = (
        f"correct {counts['correct']}  wrong {counts['wrong']}  "
        f"unassigned {counts['unassigned']}"
    )
    cv2.putText(out, legend, (16, image.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 2, cv2.LINE_AA)
    cv2.putText(out, legend, (16, image.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 40), 1, cv2.LINE_AA)
    return out, counts


def parse_attitudes(spec: str) -> list[tuple[float, float, float]]:
    """Parse --attitudes "y1,p1,r1;y2,p2,r2;..." into a list of triples."""
    attitudes: list[tuple[float, float, float]] = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(x) for x in chunk.split(",")]
        if len(parts) != 3:
            raise SystemExit(f"--attitudes entry must be 'yaw,pitch,roll': {chunk!r}")
        attitudes.append((parts[0], parts[1], parts[2]))
    if not attitudes:
        raise SystemExit("--attitudes produced no entries")
    return attitudes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--index-bin", type=Path, required=True,
                        help="Pair index .bin consumed by apps/lost_in_space_pair_id")
    parser.add_argument("--identifier-bin", type=Path,
                        default=REPO_ROOT / "build" / "apps" / "lost_in_space_pair_id")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--attitudes",
        type=str,
        default="0,0,0;30,15,5;-25,-10,12;60,30,-15;-45,-25,8;15,-30,-20",
        help="Semicolon-separated yaw,pitch,roll triples (degrees).",
    )
    parser.add_argument("--fx", type=float, default=1000.0)
    parser.add_argument("--fy", type=float, default=1000.0)
    parser.add_argument("--cx", type=float, default=512.0)
    parser.add_argument("--cy", type=float, default=512.0)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--workdir", type=Path, default=Path("/tmp/lost_in_space_gif"))
    parser.add_argument("--match-px", type=float, default=4.0,
                        help="Pixel radius used to associate a centroid with a truth star.")
    parser.add_argument("--ms-per-frame", type=int, default=900)
    parser.add_argument("--tolerance-arcsec", type=float, default=120.0)
    parser.add_argument("--neighbor-bins", type=int, default=1)
    parser.add_argument("--verification-tolerance-arcsec", type=float, default=600.0)
    parser.add_argument("--magnitude-prior-arcsec", type=float, default=15.0)
    parser.add_argument("--pyramid-size", type=int, default=6)
    parser.add_argument("--pyramid-restarts", type=int, default=3)
    parser.add_argument("--confidence-fraction", type=float, default=0.5)
    args = parser.parse_args()

    args.workdir.mkdir(parents=True, exist_ok=True)
    attitudes = parse_attitudes(args.attitudes)

    frames_pil: list[Image.Image] = []
    for frame_idx, (yaw, pitch, roll) in enumerate(attitudes):
        frame_dir = args.workdir / f"frame_{frame_idx:02d}"
        frame_dir.mkdir(parents=True, exist_ok=True)
        image_path = frame_dir / "exposure.png"
        truth_path = frame_dir / "truth.csv"
        obs_path = frame_dir / "observations.csv"
        assignments_path = frame_dir / "assignments.csv"

        run([
            sys.executable, str(REPO_ROOT / "scripts" / "render_star_image.py"),
            "--catalog", str(args.catalog),
            "--output-image", str(image_path),
            "--output-truth", str(truth_path),
            "--fx", str(args.fx), "--fy", str(args.fy),
            "--cx", str(args.cx), "--cy", str(args.cy),
            "--width", str(args.width), "--height", str(args.height),
            "--yaw-deg", str(yaw), "--pitch-deg", str(pitch), "--roll-deg", str(roll),
            "--seed", str(args.seed + frame_idx),
        ])
        run([
            sys.executable, str(REPO_ROOT / "scripts" / "centroid_stars_from_image.py"),
            "--input-image", str(image_path),
            "--output-observations", str(obs_path),
        ])
        run([
            str(args.identifier_bin),
            "--observations", str(obs_path),
            "--index", str(args.index_bin),
            "--output", str(assignments_path),
            "--fx", str(args.fx), "--fy", str(args.fy),
            "--cx", str(args.cx), "--cy", str(args.cy),
            "--tolerance-arcsec", str(args.tolerance_arcsec),
            "--neighbor-bins", str(args.neighbor_bins),
            "--verification-tolerance-arcsec", str(args.verification_tolerance_arcsec),
            "--magnitude-prior-arcsec", str(args.magnitude_prior_arcsec),
            "--pyramid-size", str(args.pyramid_size),
            "--pyramid-restarts", str(args.pyramid_restarts),
            "--confidence-fraction", str(args.confidence_fraction),
        ])

        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f"could not read rendered image: {image_path}")
        truth = load_truth(truth_path)
        observations = load_observations(obs_path)
        assignments = load_assignments(assignments_path)
        label = (
            f"frame {frame_idx + 1}/{len(attitudes)}  "
            f"yaw {yaw:+.0f}  pitch {pitch:+.0f}  roll {roll:+.0f}"
        )
        annotated, counts = annotate_frame(
            image, observations, assignments, truth, args.match_px, label)
        print(f"frame {frame_idx + 1}/{len(attitudes)}: "
              f"yaw={yaw:+.1f} pitch={pitch:+.1f} roll={roll:+.1f}  "
              f"correct={counts['correct']} wrong={counts['wrong']} "
              f"unassigned={counts['unassigned']}  "
              f"observations={len(observations)} truth={len(truth)}")
        frames_pil.append(Image.fromarray(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames_pil[0].save(
        args.output,
        save_all=True,
        append_images=frames_pil[1:],
        duration=args.ms_per_frame,
        loop=0,
        optimize=True,
    )
    print(f"wrote {args.output}  ({len(frames_pil)} frames @ {args.ms_per_frame} ms/frame)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
