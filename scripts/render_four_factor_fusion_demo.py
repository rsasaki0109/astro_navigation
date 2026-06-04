#!/usr/bin/env python3
"""Render the four-factor fusion GIF: star + VO + Skyline + TRN, online.

Phase 6 demo. A rover drives radially out of Tycho's distinctive interior into
its self-similar exterior. Each frame re-solves the position pose graph over the
poses seen SO FAR (a causal fixed-lag smoother) for two estimators that share
one solver: fused-3 (star + VO + Skyline) and fused-4 (+ TRN). Two panels:

  - Appearance map (the LROC WAC ortho TRN matches against) with ground truth,
    VO-only dead reckoning, fused-3, fused-4, the fused-4 2-sigma ellipse, and
    fixes coloured by uniqueness: skyline P-markers (green lock / orange
    aliased), TRN dots (blue lock / grey starved).
  - Position error vs pose (log) for VO-only, fused-3 and fused-4.

The honest story animates itself: in the crater interior both absolute factors
lock and the fused tracks hug truth; out in the symmetric exterior the skyline
margin collapses (its fixes scatter, fused-3 drifts away with VO) while TRN keeps
matching the textured rim, so fused-4 stays pinned with a small ellipse.

Reuses scripts/four_factor_fusion_demo.py for the scenario, appearance map,
matchers and solver, so all fixes are computed ONCE and only the graph is
re-solved per frame.
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
from four_factor_fusion_demo import build_appearance_map, TrnLocalizer  # noqa: E402


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
    ap.add_argument("--skyline-every", type=int, default=4)
    ap.add_argument("--trn-every", type=int, default=3)
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--grid", type=int, default=41)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=2.0)
    ap.add_argument("--trn-app-px", type=int, default=480)
    ap.add_argument("--trn-upsample", type=int, default=4)
    ap.add_argument("--trn-wac-zoom", type=int, default=6)
    ap.add_argument("--trn-wac-tile-radius", type=int, default=1)
    ap.add_argument("--trn-patch-frac", type=float, default=0.09)
    ap.add_argument("--trn-noise-frac", type=float, default=0.05)
    ap.add_argument("--vo-scale-bias", type=float, default=0.04)
    ap.add_argument("--vo-yaw-bias-deg", type=float, default=0.25)
    ap.add_argument("--vo-sigma-frac", type=float, default=0.06)
    ap.add_argument("--sigma-prior-m", type=float, default=20.0)
    ap.add_argument("--sigma-vo-m", type=float, default=120.0)
    ap.add_argument("--sigma-lock-m", type=float, default=400.0)
    ap.add_argument("--margin-ref", type=float, default=0.15)
    ap.add_argument("--margin-floor", type=float, default=0.01)
    ap.add_argument("--sigma-cap-m", type=float, default=40000.0)
    ap.add_argument("--trn-sigma-lock-m", type=float, default=150.0)
    ap.add_argument("--trn-margin-ref", type=float, default=0.4)
    ap.add_argument("--trn-margin-floor", type=float, default=0.02)
    ap.add_argument("--trn-unique-margin", type=float, default=0.1)
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "four_factor_fusion_demo.gif")
    ap.add_argument("--duration-ms", type=int, default=130)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    SKY_U, SKY_A = "#19c819", "#ff8c00"
    TRN_U, TRN_A = "#1f77ff", "#9aa0a6"

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

    print("building appearance map + skyline grid ...")
    app, app_px_to_m, app_kind = build_appearance_map(args, dem, px_to_m)
    sky = SkylineLocalizer(dem, px_to_m, grid=args.grid, margin_frac=args.grid_margin_frac,
                           n_az=args.n_az, n_range=args.n_range,
                           mast_height_m=args.mast_height_m, yaw_sigma_deg=args.yaw_sigma_deg)
    trn = TrnLocalizer(app, app_px_to_m, patch_m=args.trn_patch_frac * extent_m)

    # Precompute every fix once: pose -> (est_xy, sigma, unique).
    sky_by_pose, trn_by_pose = {}, {}
    for k in range(0, args.n_poses, args.skyline_every):
        est_xy, margin, _ = sky.fix(tuple(truth[k]), truth_yaw[k], star_yaw[k],
                                    rng, args.noise_arcmin)
        sigma = margin_to_sigma(margin, sigma_lock_m=args.sigma_lock_m,
                                margin_ref=args.margin_ref, margin_floor=args.margin_floor,
                                sigma_cap_m=args.sigma_cap_m)
        sky_by_pose[k] = (est_xy, sigma, margin >= 0.05)
    for k in range(0, args.n_poses, args.trn_every):
        est_xy, margin, _ = trn.fix(tuple(truth[k]), rng, args.trn_noise_frac)
        sigma = margin_to_sigma(margin, sigma_lock_m=args.trn_sigma_lock_m,
                                margin_ref=args.trn_margin_ref,
                                margin_floor=args.trn_margin_floor, sigma_cap_m=args.sigma_cap_m)
        trn_by_pose[k] = (est_xy, sigma, margin >= args.trn_unique_margin)
    print(f"  {len(sky_by_pose)} skyline + {len(trn_by_pose)} TRN fixes; appearance {app_kind}")

    vo_err_full = np.linalg.norm(vo_only - truth, axis=1)
    err3_hist = np.full(args.n_poses, np.nan)
    err4_hist = np.full(args.n_poses, np.nan)
    y_top = max(50.0, float(np.nanmax(vo_err_full)) * 1.3)
    frames: list[Image.Image] = []
    print(f"rendering {args.n_poses - 1} frames ...")
    for k in range(1, args.n_poses):
        sky_fixes = [(p, v[0], v[1]) for p, v in sky_by_pose.items() if p <= k]
        trn_fixes = [(p, v[0], v[1]) for p, v in trn_by_pose.items() if p <= k]
        est3, _ = fuse_positions(k + 1, vo_deltas[:k], truth[0],
                                 args.sigma_prior_m, args.sigma_vo_m, sky_fixes)
        est4, cov4 = fuse_positions(k + 1, vo_deltas[:k], truth[0],
                                    args.sigma_prior_m, args.sigma_vo_m, sky_fixes + trn_fixes)
        err3_hist[k] = float(np.linalg.norm(est3[k] - truth[k]))
        err4_hist[k] = float(np.linalg.norm(est4[k] - truth[k]))

        # Verdict from the most recent skyline fix state.
        sky_locked = None
        for p in range(k, -1, -1):
            if p in sky_by_pose:
                sky_locked = sky_by_pose[p][2]
                break
        if sky_locked:
            verdict, col = "Skyline + TRN both locked", "#137333"
        else:
            verdict, col = "Skyline aliased -> TRN holds the lock", "#1f77ff"

        fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.7))
        fig.suptitle(f"Four-factor fusion over {scene}   pose {k+1}/{args.n_poses}   ->  {verdict}",
                     fontsize=13, color=col)

        ax = axes[0]
        ax.imshow(app, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="gray")
        ax.plot(truth[:k + 1, 0], truth[:k + 1, 1], "-", color="white", lw=2.2, label="ground truth")
        ax.plot(vo_only[:k + 1, 0], vo_only[:k + 1, 1], "--", color="red", lw=1.6, label="VO only")
        ax.plot(est3[:, 0], est3[:, 1], "-", color="#ffd24d", lw=1.6, label="fused (no TRN)")
        ax.plot(est4[:, 0], est4[:, 1], "-", color="cyan", lw=2.0, label="fused + TRN")
        sx = 2.0 * math.sqrt(max(cov4[k, 0, 0], 0.0))
        sy = 2.0 * math.sqrt(max(cov4[k, 1, 1], 0.0))
        ax.add_patch(Ellipse((est4[k, 0], est4[k, 1]), 2 * sx, 2 * sy,
                             fill=False, color="cyan", lw=1.6))
        for p, v in sky_by_pose.items():
            if p <= k:
                ax.scatter(*v[0], c=(SKY_U if v[2] else SKY_A), marker="P",
                           s=60, edgecolors="k", lw=0.5, zorder=6)
        for p, v in trn_by_pose.items():
            if p <= k:
                ax.scatter(*v[0], c=(TRN_U if v[2] else TRN_A), marker="o",
                           s=24, edgecolors="k", lw=0.4, zorder=5)
        ax.scatter(*truth[k], c="white", marker="*", s=150, edgecolors="k", zorder=7)
        ax.set_xlim(0, extent_m); ax.set_ylim(0, extent_m)
        ax.set_title(f"appearance map ({app_kind}): truth / VO / fused / fused+TRN")
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=7.5)

        ax = axes[1]
        ax.plot(np.arange(args.n_poses), vo_err_full, "--", color="red", lw=0.8, alpha=0.3)
        ax.plot(np.arange(k + 1), vo_err_full[:k + 1], "--", color="red", lw=2.0, label="VO only")
        ax.plot(np.arange(args.n_poses), err3_hist, "-", color="#e0a800", lw=1.8, label="fused (no TRN)")
        ax.plot(np.arange(args.n_poses), err4_hist, "-", color="cyan", lw=2.4, label="fused + TRN")
        for p, v in sky_by_pose.items():
            if p <= k:
                ax.axvline(p, color=(SKY_U if v[2] else SKY_A), lw=0.9, alpha=0.4)
        ax.set_xlim(0, args.n_poses - 1)
        ax.set_yscale("log"); ax.set_ylim(10.0, y_top)
        ax.set_title("position error vs pose (log)")
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
