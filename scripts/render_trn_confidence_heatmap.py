#!/usr/bin/env python3
"""Render a TRN localizability confidence heatmap for the Tycho fixture.

The confidence map is intentionally image-derived and truth-free. It estimates
where terrain-relative navigation should have enough visual texture to lock
position by combining:

- gradient energy: crater rims and ridges
- local texture richness: non-flat patches
- feature density: SIFT keypoints, with Shi-Tomasi fallback
- illumination balance: penalize deep shadow and saturated bright regions
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parent.parent


def normalize01(values: np.ndarray, low_percentile: float = 2.0, high_percentile: float = 98.0) -> np.ndarray:
    lo = float(np.percentile(values, low_percentile))
    hi = float(np.percentile(values, high_percentile))
    if hi <= lo:
        return np.zeros_like(values, dtype=np.float32)
    return np.clip((values.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)


def put_text(
    image: np.ndarray,
    value: str,
    origin: tuple[int, int],
    *,
    scale: float,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    cv2.putText(image, value, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(image, value, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def compute_feature_density(gray: np.ndarray, output_shape: tuple[int, int]) -> tuple[np.ndarray, int, str]:
    detector_name = "SIFT"
    try:
        detector = cv2.SIFT_create(nfeatures=2500, contrastThreshold=0.015, edgeThreshold=12)
        keypoints = detector.detect(gray, None)
    except cv2.error:
        detector_name = "Shi-Tomasi"
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=2500, qualityLevel=0.01, minDistance=4)
        keypoints = []
        if corners is not None:
            keypoints = [cv2.KeyPoint(float(corner[0][0]), float(corner[0][1]), 3.0) for corner in corners]

    impulses = np.zeros(gray.shape, dtype=np.float32)
    for keypoint in keypoints:
        x = int(round(keypoint.pt[0]))
        y = int(round(keypoint.pt[1]))
        if 0 <= x < impulses.shape[1] and 0 <= y < impulses.shape[0]:
            impulses[y, x] += 1.0

    density = cv2.GaussianBlur(impulses, (0, 0), 18.0)
    density = normalize01(density, 1.0, 99.5)
    density = cv2.resize(density, (output_shape[1], output_shape[0]), interpolation=cv2.INTER_CUBIC)
    return np.clip(density, 0.0, 1.0), len(keypoints), detector_name


def build_confidence(gray: np.ndarray, grid_size: int) -> tuple[np.ndarray, dict[str, np.ndarray | int | str]]:
    small = cv2.resize(gray, (grid_size, grid_size), interpolation=cv2.INTER_AREA)
    small_f = small.astype(np.float32) / 255.0

    blurred = cv2.GaussianBlur(small, (0, 0), 1.0)
    grad_x = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3)
    gradient = normalize01(cv2.magnitude(grad_x, grad_y))

    mean = cv2.GaussianBlur(small_f, (0, 0), 3.0)
    mean_sq = cv2.GaussianBlur(small_f * small_f, (0, 0), 3.0)
    texture = normalize01(np.maximum(mean_sq - mean * mean, 0.0))

    feature_density, feature_count, detector_name = compute_feature_density(small, (grid_size, grid_size))

    darkness_penalty = np.clip((0.34 - small_f) / 0.34, 0.0, 1.0)
    saturation_penalty = np.clip((small_f - 0.86) / 0.14, 0.0, 1.0)
    illumination = np.clip(1.0 - 0.75 * darkness_penalty - 0.35 * saturation_penalty, 0.0, 1.0)

    confidence = (
        0.34 * gradient
        + 0.28 * texture
        + 0.30 * feature_density
        + 0.18 * illumination
    )
    confidence = cv2.GaussianBlur(confidence, (0, 0), 1.0)
    confidence = normalize01(confidence, 1.0, 99.0)

    terms: dict[str, np.ndarray | int | str] = {
        "gradient": gradient,
        "texture": texture,
        "feature_density": feature_density,
        "illumination": illumination,
        "feature_count": feature_count,
        "feature_detector": detector_name,
    }
    return confidence.astype(np.float32), terms


def local_to_pixel(x_m: float, y_m: float, px_to_m: float, image_shape: tuple[int, int]) -> tuple[int, int]:
    height, width = image_shape
    x = int(round(x_m / px_to_m))
    y = int(round(y_m / px_to_m))
    return max(0, min(width - 1, x)), max(0, min(height - 1, y))


def draw_bar(image: np.ndarray, origin: tuple[int, int], size: tuple[int, int]) -> None:
    x0, y0 = origin
    width, height = size
    ramp = np.linspace(0.0, 1.0, width, dtype=np.float32)
    bar = np.tile(ramp, (height, 1))
    color = cv2.applyColorMap((bar * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    image[y0 : y0 + height, x0 : x0 + width] = color
    cv2.rectangle(image, (x0, y0), (x0 + width, y0 + height), (230, 235, 230), 1, cv2.LINE_AA)


def render_panel(
    *,
    gray: np.ndarray,
    confidence: np.ndarray,
    terms: dict[str, np.ndarray | int | str],
    rover_pixel: tuple[int, int],
    output_size: tuple[int, int],
) -> np.ndarray:
    width, height = output_size
    map_w = 880
    side_w = width - map_w

    base = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    confidence_full = cv2.resize(confidence, (gray.shape[1], gray.shape[0]), interpolation=cv2.INTER_CUBIC)
    heat = cv2.applyColorMap((confidence_full * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    overlay = cv2.addWeighted(base, 0.52, heat, 0.48, 0.0)
    map_panel = cv2.resize(overlay, (map_w, height), interpolation=cv2.INTER_AREA)

    scale_x = map_w / gray.shape[1]
    scale_y = height / gray.shape[0]

    def scale_point(point: tuple[int, int]) -> tuple[int, int]:
        return int(round(point[0] * scale_x)), int(round(point[1] * scale_y))

    rover_scaled = scale_point(rover_pixel)
    cv2.circle(map_panel, rover_scaled, 18, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 14, (90, 255, 150), -1, cv2.LINE_AA)
    put_text(map_panel, "TRN CONFIDENCE HEATMAP", (24, 42), scale=0.78, color=(245, 246, 242), thickness=2)
    put_text(map_panel, "blue = weak texture / poor light   yellow-red = strong TRN lock potential", (24, 76), scale=0.47, color=(220, 225, 220))
    put_text(map_panel, "TRN LOCK", (rover_scaled[0] + 22, rover_scaled[1] - 14), scale=0.48, color=(90, 255, 150))

    side = np.full((height, side_w, 3), (22, 25, 30), dtype=np.uint8)
    x0 = 28
    put_text(side, "LOCALIZABILITY", (x0, 52), scale=0.82, color=(245, 245, 240), thickness=2)
    put_text(side, "TYCHO TRN", (x0, 90), scale=0.82, color=(245, 245, 240), thickness=2)

    percent_high = float(np.mean(confidence >= 0.72) * 100.0)
    percent_low = float(np.mean(confidence <= 0.32) * 100.0)
    rover_grid = (
        int(round(rover_pixel[0] * confidence.shape[1] / gray.shape[1])),
        int(round(rover_pixel[1] * confidence.shape[0] / gray.shape[0])),
    )
    rover_score = float(confidence[min(confidence.shape[0] - 1, rover_grid[1]), min(confidence.shape[1] - 1, rover_grid[0])])

    rows = [
        ("TRN SCORE", f"{rover_score:0.2f}"),
        ("HIGH CONF", f"{percent_high:5.1f} %"),
        ("LOW CONF", f"{percent_low:5.1f} %"),
        ("FEATURES", str(terms["feature_count"])),
        ("DETECTOR", str(terms["feature_detector"])),
    ]
    y = 150
    for label, value in rows:
        put_text(side, label, (x0, y), scale=0.44, color=(125, 175, 220))
        put_text(side, value, (x0, y + 30), scale=0.58, color=(236, 238, 236))
        y += 67

    draw_bar(side, (x0, height - 104), (side_w - 56, 24))
    put_text(side, "LOW", (x0, height - 56), scale=0.38, color=(200, 205, 210))
    put_text(side, "HIGH", (side_w - 92, height - 56), scale=0.38, color=(240, 220, 130))
    return np.hstack([map_panel, side])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trn-fixture-dir", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_lro_tycho_terminal")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_confidence_heatmap.png")
    parser.add_argument("--grid-size", type=int, default=220)
    args = parser.parse_args()

    summary_path = args.trn_fixture_dir / "summary.json"
    ortho_path = args.trn_fixture_dir / "ortho.png"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    gray = cv2.imread(str(ortho_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise SystemExit(f"failed to read {ortho_path}")

    confidence, terms = build_confidence(gray, args.grid_size)
    px_to_m = float(summary["wac"]["px_to_m"])
    rover_pixel = local_to_pixel(
        float(summary["rover_estimated_xyz_m"][0]),
        float(summary["rover_estimated_xyz_m"][1]),
        px_to_m,
        gray.shape,
    )

    panel = render_panel(
        gray=gray,
        confidence=confidence,
        terms=terms,
        rover_pixel=rover_pixel,
        output_size=(1280, 720),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)).save(args.output)

    rover_grid = (
        int(round(rover_pixel[0] * confidence.shape[1] / gray.shape[1])),
        int(round(rover_pixel[1] * confidence.shape[0] / gray.shape[0])),
    )
    summary_out = {
        "output": str(args.output),
        "source_ortho": str(ortho_path),
        "grid_size": args.grid_size,
        "feature_detector": str(terms["feature_detector"]),
        "feature_count": int(terms["feature_count"]),
        "rover_pixel": list(rover_pixel),
        "rover_confidence": float(confidence[min(confidence.shape[0] - 1, rover_grid[1]), min(confidence.shape[1] - 1, rover_grid[0])]),
        "mean_confidence": float(np.mean(confidence)),
        "high_confidence_fraction": float(np.mean(confidence >= 0.72)),
        "low_confidence_fraction": float(np.mean(confidence <= 0.32)),
        "model": "0.34*gradient + 0.28*texture + 0.30*feature_density + 0.18*illumination_balance",
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary_out, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary_out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
