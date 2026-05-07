#!/usr/bin/env python3
"""Detect star centroids in a PNG via local-max threshold + 2D Gaussian fit.

Reads a single-channel star tracker image (8 or 16 bit), finds bright spots
above an adaptive threshold, refines each peak with a 2D Gaussian fit, and
writes the resulting (u, v) list as `id,u,v` (with `id` empty so downstream
identify_stars_with_pair_index.py treats it as unlabeled).
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import curve_fit
from skimage.feature import peak_local_max


def gaussian_2d(coords: np.ndarray, amp: float, u0: float, v0: float, sigma: float, offset: float) -> np.ndarray:
    u, v = coords
    return amp * np.exp(-((u - u0) ** 2 + (v - v0) ** 2) / (2.0 * sigma * sigma)) + offset


def refine_peak(
    image: np.ndarray, u_int: int, v_int: int, window: int, sigma_init: float
) -> tuple[float, float, float] | None:
    h, w = image.shape
    ulo = max(0, u_int - window)
    uhi = min(w, u_int + window + 1)
    vlo = max(0, v_int - window)
    vhi = min(h, v_int + window + 1)
    patch = image[vlo:vhi, ulo:uhi].astype(np.float64)
    uu, vv = np.meshgrid(np.arange(ulo, uhi), np.arange(vlo, vhi))
    coords = (uu.ravel(), vv.ravel())
    initial = (
        float(patch.max() - patch.min()),
        float(u_int),
        float(v_int),
        float(sigma_init),
        float(patch.min()),
    )
    try:
        popt, _ = curve_fit(gaussian_2d, coords, patch.ravel(), p0=initial, maxfev=300)
    except (RuntimeError, ValueError):
        return None
    amp, u0, v0, sigma, _ = popt
    if amp <= 0.0 or sigma <= 0.0 or sigma > 5.0 * sigma_init:
        return None
    if not (ulo <= u0 < uhi and vlo <= v0 < vhi):
        return None
    return float(u0), float(v0), float(amp)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-image", type=Path, required=True)
    parser.add_argument("--output-observations", type=Path, required=True)
    parser.add_argument(
        "--threshold-sigma",
        type=float,
        default=8.0,
        help="Detection threshold above the background median, in units of background MAD-sigma.",
    )
    parser.add_argument("--peak-min-distance-px", type=int, default=4)
    parser.add_argument("--fit-window-px", type=int, default=4)
    parser.add_argument("--psf-sigma-init-px", type=float, default=1.0)
    parser.add_argument("--max-stars", type=int, default=128)
    args = parser.parse_args()

    image = cv2.imread(str(args.input_image), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"could not read {args.input_image}")
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_f = image.astype(np.float64)

    # Robust background statistics: median + scaled MAD as sigma proxy.
    median = float(np.median(image_f))
    mad = float(np.median(np.abs(image_f - median)))
    sigma = max(mad * 1.4826, 1.0)
    threshold = median + args.threshold_sigma * sigma

    coords = peak_local_max(
        image_f,
        min_distance=args.peak_min_distance_px,
        threshold_abs=threshold,
        num_peaks=args.max_stars * 4,
    )
    centroids: list[tuple[float, float, float]] = []
    for v_int, u_int in coords:
        result = refine_peak(image_f, u_int, v_int, args.fit_window_px, args.psf_sigma_init_px)
        if result is not None:
            centroids.append(result)

    centroids.sort(key=lambda item: -item[2])
    centroids = centroids[: args.max_stars]

    args.output_observations.parent.mkdir(parents=True, exist_ok=True)
    with args.output_observations.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "u", "v"])
        for u, v, _amp in centroids:
            writer.writerow(["", f"{u:.6f}", f"{v:.6f}"])
    print(f"wrote {args.output_observations} ({len(centroids)} centroids; threshold={threshold:.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
