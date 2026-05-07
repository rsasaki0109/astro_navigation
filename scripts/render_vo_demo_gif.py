#!/usr/bin/env python3
"""Render an animated GIF that shows SIFT features per frame and the growing
trajectory alongside, for a POLAR Traverse-style sequence."""

from __future__ import annotations

import argparse
import csv
import io
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from evaluate_trajectory import (
    apply_similarity,
    load_estimated_positions,
    load_polar_gt_positions,
    umeyama_align,
)


def load_image_paths(images_txt: Path) -> list[Path]:
    return [Path(line.strip()) for line in images_txt.read_text().splitlines() if line.strip()]


def render_features(image_path: Path, n_features: int) -> np.ndarray:
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    sift = cv2.SIFT_create(nfeatures=n_features, contrastThreshold=0.04)
    kps = sift.detect(img, None)
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for kp in kps:
        cv2.circle(
            out,
            (int(kp.pt[0]), int(kp.pt[1])),
            max(3, int(kp.size / 2)),
            (0, 255, 100),
            2,
            lineType=cv2.LINE_AA,
        )
    return out


def crop_top(image: np.ndarray, fraction: float) -> np.ndarray:
    h = image.shape[0]
    return image[int(h * fraction) :, :]


def render_trajectory_panel(
    aligned: np.ndarray,
    gt: np.ndarray,
    frame_idx: int,
    title: str,
    width_px: int,
    height_px: int,
) -> np.ndarray:
    fig = plt.figure(figsize=(width_px / 100.0, height_px / 100.0), dpi=100)
    ax = fig.add_subplot(111)
    ax.plot(gt[:, 0], gt[:, 1], color="black", linewidth=2.0, label="ground truth")
    ax.plot(
        aligned[: frame_idx + 1, 0],
        aligned[: frame_idx + 1, 1],
        marker="o",
        markersize=5,
        linewidth=1.5,
        color="#1f77b4",
        label="SIFT VO",
    )
    ax.scatter(aligned[frame_idx, 0], aligned[frame_idx, 1], s=120, color="red", zorder=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    plt.close(fig)
    buf.seek(0)
    arr = np.array(Image.open(buf).convert("RGB"))
    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True, help="images.txt with one PNG path per line")
    parser.add_argument("--ground-truth", type=Path, required=True, help="POLAR refined_poses.tsv")
    parser.add_argument("--trajectory", type=Path, required=True, help="Estimated trajectory .tum")
    parser.add_argument("--output", type=Path, required=True, help="Output GIF path")
    parser.add_argument("--features", type=int, default=300)
    parser.add_argument("--crop-top-fraction", type=float, default=0.3)
    parser.add_argument("--frame-width-px", type=int, default=540)
    parser.add_argument("--ms-per-frame", type=int, default=600)
    args = parser.parse_args()

    image_paths = load_image_paths(args.images)
    gt = load_polar_gt_positions(args.ground_truth)
    estimated = load_estimated_positions(args.trajectory)
    n = min(len(image_paths), len(estimated), len(gt))
    image_paths = image_paths[:n]
    estimated = estimated[:n]
    gt = gt[:n]
    scale, rotation, translation = umeyama_align(estimated, gt, estimate_scale=True)
    aligned = apply_similarity(estimated, scale, rotation, translation)

    frames: list[Image.Image] = []
    for idx, image_path in enumerate(image_paths):
        feature_image = render_features(image_path, args.features)
        feature_image = crop_top(feature_image, args.crop_top_fraction)
        # resize feature panel
        h, w = feature_image.shape[:2]
        new_w = args.frame_width_px
        new_h = int(h * new_w / w)
        feature_panel = cv2.resize(feature_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        cv2.putText(
            feature_panel,
            f"frame {idx + 1}/{n}  SIFT features",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 100),
            2,
            cv2.LINE_AA,
        )
        traj_panel = render_trajectory_panel(
            aligned,
            gt,
            idx,
            f"VO trajectory @ frame {idx + 1}/{n}",
            new_w,
            new_h,
        )
        composite = np.hstack([feature_panel, traj_panel])
        frames.append(Image.fromarray(cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=args.ms_per_frame,
        loop=0,
        optimize=True,
    )
    print(f"wrote {args.output}  ({n} frames @ {args.ms_per_frame} ms/frame)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
