#!/usr/bin/env python3
"""Render the factor-graph fusion demo GIF: online star + VO + Skyline fusion.

Phase 5 demo. A rover drives radially out of Tycho's distinctive interior into
the self-similar exterior. Each frame re-solves the position factor graph over
the poses seen SO FAR (a causal fixed-lag smoother), fusing VO between-factors,
the star-tracker attitude, and Skyline position fixes whose information is the
real uniqueness_margin. Two panels:

  - DEM with ground truth, VO-only dead reckoning, the fused estimate, the
    current 2-sigma covariance ellipse, and skyline fixes coloured by
    uniqueness (green = unique lock, orange = aliased / down-weighted).
  - Position error vs pose for VO-only vs fused, filling in as the rover drives.

The honest story animates itself: inside the crater the fix is tight and the
fused track hugs ground truth with a small ellipse; out in the symmetric
exterior the margin collapses, the skyline fixes scatter and are ignored, and
the fused estimate coasts on VO with a visibly growing ellipse.

Reuses scripts/factor_graph_fusion_demo.py for the scenario + solver, so the
skyline candidate grid is predicted ONCE and only the graph is re-solved.
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

from skyline_lock_demo import build_dem  # noqa: E402
from factor_graph_fusion_demo import (  # noqa: E402
    make_truth_trajectory, true_yaw_series, simulate_vo, integrate,
    SkylineLocalizer, margin_to_sigma, fuse_positions,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["synth", "lola"], default="lola")
    ap.add_argument("--terrain", choices=["hills", "craters", "flat"], default="craters")
    ap.add_argument("--target", default="tycho")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path, default=REPO_ROOT / "datasets" / "lro_cache")
    ap.add_argument("--size-px", type=int, default=512)
    ap.add_argument("--px-to-m", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--trajectory", choices=["radial", "s_curve", "diag"], default="radial")
    ap.add_argument("--n-poses", type=int, default=60)
    ap.add_argument("--skyline-every", type=int, default=3)
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--grid", type=int, default=41)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=2.0)
    ap.add_argument("--vo-scale-bias", type=float, default=0.04)
    ap.add_argument("--vo-yaw-bias-deg", type=float, default=0.25)
    ap.add_argument("--vo-sigma-frac", type=float, default=0.06)
    ap.add_argument("--sigma-prior-m", type=float, default=20.0)
    ap.add_argument("--sigma-vo-m", type=float, default=120.0)
    ap.add_argument("--sigma-lock-m", type=float, default=400.0)
    ap.add_argument("--margin-ref", type=float, default=0.15)
    ap.add_argument("--margin-floor", type=float, default=0.01)
    ap.add_argument("--sigma-cap-m", type=float, default=40000.0)
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_fusion_demo.gif")
    ap.add_argument("--duration-ms", type=int, default=130)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    dem, px_to_m = build_dem(args)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"

    rng = np.random.default_rng(args.seed)
    truth = make_truth_trajectory(extent_m, args.n_poses, args.trajectory)
    truth_yaw = true_yaw_series(truth)
    star_yaw = truth_yaw + rng.normal(0.0, math.radians(args.yaw_sigma_deg), size=truth_yaw.shape)
    vo_deltas = simulate_vo(truth, star_yaw, rng, scale_bias=args.vo_scale_bias,
                            yaw_bias_rad=math.radians(args.vo_yaw_bias_deg),
                            sigma_frac=args.vo_sigma_frac)
    vo_only = integrate(truth[0], vo_deltas)

    print("precomputing skyline candidate grid ...")
    loc = SkylineLocalizer(dem, px_to_m, grid=args.grid, margin_frac=args.grid_margin_frac,
                           n_az=args.n_az, n_range=args.n_range,
                           mast_height_m=args.mast_height_m, yaw_sigma_deg=args.yaw_sigma_deg)

    # Precompute all skyline fixes once (pose -> (est_xy, sigma, margin, unique)).
    fixes_by_pose = {}
    for k in range(0, args.n_poses, args.skyline_every):
        est_xy, margin, ncc = loc.fix(tuple(truth[k]), truth_yaw[k], star_yaw[k],
                                      rng, args.noise_arcmin)
        sigma = margin_to_sigma(margin, sigma_lock_m=args.sigma_lock_m,
                                margin_ref=args.margin_ref, margin_floor=args.margin_floor,
                                sigma_cap_m=args.sigma_cap_m)
        fixes_by_pose[k] = (est_xy, sigma, margin, margin >= 0.05)
    print(f"precomputed {len(fixes_by_pose)} skyline fixes")

    vo_err_full = np.linalg.norm(vo_only - truth, axis=1)
    frames: list[Image.Image] = []
    fused_err_hist = np.full(args.n_poses, np.nan)
    print(f"rendering {args.n_poses} frames ...")
    for k in range(1, args.n_poses):
        fixes = [(p, fx[0], fx[1]) for p, fx in fixes_by_pose.items() if p <= k]
        est, cov = fuse_positions(k + 1, vo_deltas[:k], truth[0],
                                  args.sigma_prior_m, args.sigma_vo_m, fixes)
        fused_err_hist[k] = float(np.linalg.norm(est[k] - truth[k]))
        cur_margin = None
        for p in range(k, -1, -1):
            if p in fixes_by_pose:
                cur_margin = fixes_by_pose[p]
                break

        fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))
        locked = cur_margin is not None and cur_margin[3]
        verdict = "LOCKED on distinctive terrain" if locked else "dead reckoning (skyline aliased)"
        fig.suptitle(f"Factor-graph fusion over {scene}   pose {k+1}/{args.n_poses}   ->  {verdict}",
                     fontsize=13, color=("#137333" if locked else "#b8860b"))

        ax = axes[0]
        im = ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(truth[:k + 1, 0], truth[:k + 1, 1], "-", color="white", lw=2.2, label="ground truth")
        ax.plot(vo_only[:k + 1, 0], vo_only[:k + 1, 1], "--", color="red", lw=1.8, label="VO only")
        ax.plot(est[:, 0], est[:, 1], "-", color="cyan", lw=1.8, label="fused")
        sx = 2.0 * math.sqrt(max(cov[k, 0, 0], 0.0))
        sy = 2.0 * math.sqrt(max(cov[k, 1, 1], 0.0))
        ax.add_patch(Ellipse((est[k, 0], est[k, 1]), 2 * sx, 2 * sy,
                             fill=False, color="cyan", lw=1.6))
        for p, fx in fixes_by_pose.items():
            if p <= k:
                ax.scatter(*fx[0], c=("#19c819" if fx[3] else "#ff8c00"),
                           marker="P", s=64, edgecolors="k", lw=0.5, zorder=6)
        ax.scatter(*truth[k], c="white", marker="*", s=160, edgecolors="k", zorder=7)
        ax.set_xlim(0, extent_m); ax.set_ylim(0, extent_m)
        ax.set_title("DEM (m): truth / VO-only / fused (+2σ)")
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=8)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

        ax = axes[1]
        ax.plot(np.arange(args.n_poses), vo_err_full, "--", color="red", lw=1.0, alpha=0.35)
        ax.plot(np.arange(k + 1), vo_err_full[:k + 1], "--", color="red", lw=2.0, label="VO only")
        ax.plot(np.arange(args.n_poses), fused_err_hist, "-", color="cyan", lw=2.0, label="fused")
        for p, fx in fixes_by_pose.items():
            if p <= k:
                ax.axvline(p, color=("#19c819" if fx[3] else "#ff8c00"), lw=1.0, alpha=0.45)
        ax.set_xlim(0, args.n_poses - 1)
        ax.set_ylim(0, max(50.0, np.nanmax(vo_err_full) * 1.05))
        ax.set_title("position error vs pose")
        ax.set_xlabel("pose index"); ax.set_ylabel("error (m)")
        ax.legend(fontsize=8, loc="upper left")

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)
        if k % 10 == 0:
            print(f"  {k}/{args.n_poses}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:] + [frames[-1]] * 8,
                   duration=args.duration_ms, loop=0, optimize=True)
    print(f"wrote {args.output}  ({len(frames)} frames)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
