#!/usr/bin/env python3
"""Animate Skyline Lock Phase 7: dial the Moon's curvature into the horizon model.

The rover physically observes the TRUE (curved) lunar horizon. We sweep the
*model* used to localize it, from a flat plane (Phase 0-6) to the correct
spherical Moon, and watch where the position fix lands. Two scenes side by side:

  * Tycho (near, kilometre-high rim dominates the skyline): the fix holds at the
    centre the whole sweep -- curvature is essentially free.
  * Apollo 11 mare (skyline leans on faint distant relief): the flat model snaps
    the fix ~16 km onto a phantom mode built from terrain that is physically
    below the lunar horizon; as the curvature is dialled in, those phantom cues
    sink away and the fix snaps home.

Same correction, opposite consequence -- the honest-envelope point of Phase 7.

To keep it fast, each candidate's (azimuth x range) sampled-height cube is built
once with cv2.remap; every frame only re-applies the parabolic drop
alpha * r^2 / (2R) and re-takes the per-azimuth max in numpy.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

from skyline_lock_demo import (  # noqa: E402
    load_lola_dem, render_horizon, best_yaw_ncc, _sample_height,
)
from lro_trn_demo import LUNAR_RADIUS_M  # noqa: E402

R = LUNAR_RADIUS_M
FLAT = "#ff8c00"
CURVED = "#1f77ff"


def build_cube(dem, px_to_m, cand, mast, A, r_min, r_max, n_range):
    """Return (cube (M,A,Rr) sampled heights, cam_z (M,), rng_m (Rr,))."""
    az = np.linspace(0.0, 2.0 * math.pi, A, endpoint=False)
    rng_m = np.linspace(r_min, r_max, n_range)
    dx = np.sin(az)[:, None]
    dy = np.cos(az)[:, None]
    cube = np.empty((len(cand), A, n_range), dtype=np.float32)
    cam_z = np.empty(len(cand), dtype=np.float64)
    for i, (cx, cy) in enumerate(cand):
        xs = cx + rng_m[None, :] * dx
        ys = cy + rng_m[None, :] * dy
        cube[i] = cv2.remap(
            dem, (xs / px_to_m).astype(np.float32), (ys / px_to_m).astype(np.float32),
            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=float("nan"))
        cam_z[i] = _sample_height(dem, px_to_m, cx, cy) + mast
    return cube, cam_z, rng_m


def horizons_at_alpha(cube, cam_z, rng_m, alpha):
    """Per-candidate horizon (M,A) with curvature drop alpha * r^2/(2R)."""
    drop = alpha * (rng_m ** 2) / (2.0 * R)              # (Rr,)
    elev = np.arctan2(cube - drop[None, None, :] - cam_z[:, None, None],
                      rng_m[None, None, :])              # (M,A,Rr)
    elev = np.where(np.isfinite(elev), elev, -np.inf)
    horizon = np.max(elev, axis=2)
    return np.where(np.isfinite(horizon), horizon, 0.0)


class Scene:
    def __init__(self, target, args, kind):
        self.target = target
        self.kind = kind
        dem, px = load_lola_dem(target, args.half_width_deg, args.ldem_ppd, args.cache_dir)
        self.dem, self.px = dem, px
        extent = dem.shape[0] * px
        self.extent = extent
        A = args.n_az
        self.A = A
        self.bpd = A / 360.0
        r_min = max(args.r_min_m, 2.0 * px)
        r_max = 0.9 * extent
        m = args.grid_margin_frac
        gx = np.linspace(m * extent, (1.0 - m) * extent, args.grid)
        gy = np.linspace(m * extent, (1.0 - m) * extent, args.grid)
        self.gx, self.gy = gx, gy
        self.cand = [(x, y) for y in gy for x in gx]
        self.truth = (0.5 * extent, 0.5 * extent)
        self.prior_lag = int(round(args.truth_yaw_deg * self.bpd)) % A
        self.window = max(1, int(math.ceil(3.0 * args.yaw_sigma_deg * self.bpd)))
        # observation = true curved horizon (+ noise)
        hw = render_horizon(dem, px, self.truth, args.mast_height_m, n_az=A,
                            r_min_m=r_min, r_max_m=r_max, n_range=args.n_range,
                            curvature_radius_m=R)
        heading = int(round(args.truth_yaw_deg * self.bpd)) % A
        rng = np.random.default_rng(args.seed + 1)
        self.obs = np.roll(hw, -heading) + rng.normal(
            0.0, math.radians(args.noise_arcmin / 60.0), size=A)
        self.cube, self.cam_z, self.rng_m = build_cube(
            dem, px, self.cand, args.mast_height_m, A, r_min, r_max, args.n_range)
        # Fixed per-scene colour range (from the correct-model surface) so both
        # the sharp crater peak and the broad mare field read, with no flicker.
        ref = self.fix_at(1.0)[0]
        self.vmin = float(np.percentile(ref, 55))
        self.vmax = float(ref.max())

    def fix_at(self, alpha):
        preds = horizons_at_alpha(self.cube, self.cam_z, self.rng_m, alpha)
        ncc, _ = best_yaw_ncc(self.obs, preds, prior_lag=self.prior_lag,
                              window_bins=self.window)
        fi = int(np.argmax(ncc))
        est = self.cand[fi]
        err = math.hypot(est[0] - self.truth[0], est[1] - self.truth[1])
        return ncc.reshape(len(self.gy), len(self.gx)), est, err


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="tycho")
    ap.add_argument("--mare-target", default="apollo11")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path, default=REPO / "datasets" / "lro_cache")
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--r-min-m", type=float, default=60.0)
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--grid", type=int, default=27)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--truth-yaw-deg", type=float, default=35.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--duration-ms", type=int, default=150)
    ap.add_argument("--output", type=Path,
                    default=REPO / "docs" / "figures" / "skyline_lock" / "skyline_curvature_demo.gif")
    ap.add_argument("--mp4", action="store_true", help="also write an .mp4 next to the gif")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    print("building scenes (one-time cube remap) ...")
    scenes = [Scene(args.target, args, "near rim dominates"),
              Scene(args.mare_target, args, "leans on distant terrain")]

    # alpha sweep: flat (0) -> Moon (1), eased so the snap is readable.
    alphas = np.linspace(0.0, 1.0, args.frames) ** 0.7

    frames: list[Image.Image] = []
    print(f"rendering {args.frames} frames ...")
    for i, alpha in enumerate(alphas):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5.6))
        pct = 100.0 * alpha
        r_eff = (R / alpha) if alpha > 1e-6 else float("inf")
        r_txt = "flat plane" if alpha < 1e-6 else f"R = {r_eff/1e6:.2f}e6 m"
        fig.suptitle(
            f"Dialling lunar curvature into the horizon model  --  "
            f"{pct:3.0f}% applied   ({r_txt})",
            fontsize=13)
        for ax, sc in zip(axes, scenes):
            surf, est, err = sc.fix_at(alpha)
            im = ax.imshow(surf, origin="lower",
                           extent=[sc.gx[0], sc.gx[-1], sc.gy[0], sc.gy[-1]],
                           cmap="viridis", aspect="auto", vmin=sc.vmin, vmax=sc.vmax)
            tx, ty = sc.truth
            ax.scatter(tx, ty, c="white", marker="*", s=240, edgecolors="k", zorder=6)
            locked = err <= 1.6 * (sc.gx[1] - sc.gx[0])
            col = CURVED if locked else FLAT
            ax.scatter(est[0], est[1], facecolors="none", edgecolors=col,
                       marker="o", s=200, linewidths=2.6, zorder=6)
            verdict = "fix on truth" if locked else "phantom fix"
            ax.set_title(f"{sc.target} ({sc.kind})\n{verdict} -- error {err/1000:.1f} km",
                         color=col, fontsize=11)
            ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        fig.tight_layout(rect=[0, 0, 1, 0.93])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)
        if (i + 1) % 6 == 0:
            print(f"  {i + 1}/{args.frames}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True,
                   append_images=frames[1:] + [frames[-1]] * 10,
                   duration=args.duration_ms, loop=0, optimize=True)
    print(f"wrote {args.output}  ({len(frames)} frames)")

    if args.mp4:
        mp4 = args.output.with_suffix(".mp4")
        cmd = ["ffmpeg", "-y", "-i", str(args.output),
               "-movflags", "faststart", "-pix_fmt", "yuv420p",
               "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", str(mp4)]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            print(f"wrote {mp4}")
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"mp4 conversion skipped: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
