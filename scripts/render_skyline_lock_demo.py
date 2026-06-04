#!/usr/bin/env python3
"""Render the Skyline Lock demo GIF: live horizon localizability over Tycho.

A rover traverses from Tycho's distinctive interior outward across the rim into
flatter terrain. Each frame matches its observed horizon profile against a fixed
candidate grid (scripts/skyline_lock_demo.py) and shows three panels:

  - DEM with the moving truth position, the recovered estimate, and the trail.
  - The live horizon-match score surface (best-yaw NCC) -- a tight single peak
    over the distinctive interior, broadening / aliasing as the rover leaves it.
  - Observed vs predicted horizon at the current estimate.

The headline is the honest one: the lock is sharp where the terrain is
distinctive (high uniqueness_margin) and degrades where it is self-similar or
rotationally symmetric -- localizability, made visible.

Reuses the real LOLA Tycho DEM and the matcher from skyline_lock_demo.py, so the
candidate horizons are predicted ONCE and only the observation moves per frame.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from skyline_lock_demo import (  # noqa: E402
    load_lola_dem, render_horizon, best_yaw_ncc, uniqueness_margin,
    _zero_mean_unit,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="tycho")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path,
                    default=REPO_ROOT / "datasets" / "lro_cache")
    ap.add_argument("--frames", type=int, default=48)
    ap.add_argument("--grid", type=int, default=41)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--truth-yaw-deg", type=float, default=35.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=5.0)
    # Path (fractions of map extent): interior centre -> out past the rim.
    ap.add_argument("--path-start", type=float, nargs=2, default=(0.50, 0.50))
    ap.add_argument("--path-end", type=float, nargs=2, default=(0.80, 0.26))
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "skyline_lock_demo.gif")
    ap.add_argument("--duration-ms", type=int, default=110)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dem, px_to_m = load_lola_dem(args.target, args.half_width_deg,
                                 args.ldem_ppd, args.cache_dir)
    size = dem.shape[0]
    extent_m = size * px_to_m
    r_max = 0.9 * extent_m
    r_min = max(60.0, 2.0 * px_to_m)
    A = args.n_az
    bins_per_deg = A / 360.0

    def horizon_at(xy):
        return render_horizon(dem, px_to_m, xy, args.mast_height_m,
                              n_az=A, r_min_m=r_min, r_max_m=r_max,
                              n_range=args.n_range)

    # Fixed candidate grid + predicted horizons (computed once).
    m = args.grid_margin_frac
    gx = np.linspace(m * extent_m, (1.0 - m) * extent_m, args.grid)
    gy = np.linspace(m * extent_m, (1.0 - m) * extent_m, args.grid)
    cand_xy = [(x, y) for y in gy for x in gx]
    print(f"predicting {len(cand_xy)} candidate horizons ...")
    preds = np.stack([horizon_at(xy) for xy in cand_xy])

    prior_lag = int(round((args.truth_yaw_deg) * bins_per_deg)) % A
    window_bins = max(1, int(math.ceil(3.0 * args.yaw_sigma_deg * bins_per_deg)))
    heading_bins = int(round(args.truth_yaw_deg * bins_per_deg)) % A
    rng = np.random.default_rng(0)
    noise_rad = math.radians(args.noise_arcmin / 60.0)
    grid_step_m = (gx[1] - gx[0]) if args.grid > 1 else extent_m

    # Rover path (truth positions in metres).
    t = np.linspace(0.0, 1.0, args.frames)
    px = (args.path_start[0] + t * (args.path_end[0] - args.path_start[0])) * extent_m
    py = (args.path_start[1] + t * (args.path_end[1] - args.path_start[1])) * extent_m

    az_deg = np.degrees(np.linspace(0.0, 2.0 * math.pi, A, endpoint=False))
    frames: list[Image.Image] = []
    print(f"rendering {args.frames} frames ...")
    for i in range(args.frames):
        truth_xy = (float(px[i]), float(py[i]))
        obs = np.roll(horizon_at(truth_xy), -heading_bins)
        obs = obs + rng.normal(0.0, noise_rad, size=obs.shape)
        ncc, best_lag = best_yaw_ncc(obs, preds, prior_lag=prior_lag,
                                     window_bins=window_bins)
        score_grid = ncc.reshape(args.grid, args.grid)
        j = int(np.argmax(ncc))
        est_xy = cand_xy[j]
        err_m = math.hypot(est_xy[0] - truth_xy[0], est_xy[1] - truth_xy[1])
        margin, _ = uniqueness_margin(ncc, cand_xy, est_xy, radius_m=2.0 * grid_step_m)
        unique = margin >= 0.05
        shift = int(best_lag[j])

        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        verdict = "LOCKED (unique)" if unique else "AMBIGUOUS (aliased)"
        fig.suptitle(
            f"Skyline Lock over {args.target.title()}   "
            f"best NCC={ncc[j]:.3f}   margin={margin:.3f}   "
            f"err={err_m/1000:.1f} km   ->  {verdict}",
            fontsize=13,
            color=("#137333" if unique else "#b8860b"))

        ax = axes[0]
        im = ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(px[:i + 1], py[:i + 1], "-", color="white", lw=1.0, alpha=0.7)
        ax.scatter(*truth_xy, c="white", marker="*", s=220, edgecolors="k", zorder=5, label="truth")
        ax.scatter(*est_xy, c="red", marker="x", s=110, zorder=5, label="estimate")
        ax.set_title("DEM (m) + rover traverse")
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper right", fontsize=8)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

        ax = axes[1]
        im = ax.imshow(score_grid, origin="lower",
                       extent=[gx[0], gx[-1], gy[0], gy[-1]], cmap="viridis",
                       vmin=-0.3, vmax=1.0, aspect="auto")
        ax.scatter(*truth_xy, c="white", marker="*", s=180, edgecolors="k", zorder=5)
        ax.set_title("horizon-match score surface (best-yaw NCC)")
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

        ax = axes[2]
        ax.plot(az_deg, np.degrees(obs), label="observed", lw=1.5)
        ax.plot(az_deg, np.degrees(np.roll(preds[j], -shift)),
                label="predicted@estimate", lw=1.2, alpha=0.8)
        ax.set_ylim(-2, 18)
        ax.set_title("horizon profile")
        ax.set_xlabel("azimuth (deg)"); ax.set_ylabel("elevation (deg)")
        ax.legend(fontsize=8, loc="upper right")

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)
        if (i + 1) % 8 == 0:
            print(f"  {i + 1}/{args.frames}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Hold the last frame a beat so the verdict reads.
    frames[0].save(args.output, save_all=True, append_images=frames[1:] + [frames[-1]] * 8,
                   duration=args.duration_ms, loop=0, optimize=True)
    print(f"wrote {args.output}  ({len(frames)} frames)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
