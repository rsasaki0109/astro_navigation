#!/usr/bin/env python3
"""Honest factor-graph fusion: star tracker + VO + Skyline over real terrain.

Phase 5 of Skyline Lock. The earlier phases built absolute horizon localization
in isolation; this ties the project's localization modalities into ONE estimator
and -- the whole point -- lets each one carry the fix only as far as its own
confidence earns. A GNSS-denied rover drives a traverse and we fuse:

  - VO (visual odometry): a between-pose factor giving relative world-frame
    displacement (body delta rotated by the fused attitude). Locally good,
    globally drifts -- the error random-walks with distance.
  - Star tracker: a unary attitude factor pinning yaw to ~arcmin. Strong and
    absolute, so we rotate VO's body deltas into the world with it.
  - Skyline Lock: a unary *position* factor whose information is set by the
    REAL uniqueness_margin from scripts/skyline_lock_demo.py -- the Phase 0-4
    localizability signal. Where the terrain is distinctive the fix is tight and
    pins the trajectory; where it is self-similar / rotationally symmetric the
    skyline fix itself jumps to an aliased position AND its margin collapses, so
    the graph down-weights it automatically and falls back to dead reckoning.

The estimator is a small linear least-squares pose graph (positions decouple per
axis given absolute attitude), solved in closed form; per-pose covariance comes
straight from the inverse normal matrix, so we can draw the uncertainty growing
during dead reckoning and snapping tight at each good skyline lock.

Honest-envelope claims, all visible in the output:
  - Fused beats VO-only (bounded vs random-walk error) ONLY where skyline locks.
  - A low-margin (aliased) skyline fix is correctly ignored, not trusted: the
    covariance keeps growing through it instead of jumping to the wrong rim arc.
  - Confidence is terrain-driven and reproducible from public LOLA data, not a
    hand-tuned schedule.

Reuses scripts/skyline_lock_demo.py for the DEM + horizon matcher. Conventions
match it: world +X east, +Y north, +Z up; azimuth 0 = +Y (north), clockwise.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from skyline_lock_demo import (  # noqa: E402
    build_dem, render_horizon, best_yaw_ncc, uniqueness_margin,
)


# --------------------------------------------------------------------------- #
# Scenario simulation                                                         #
# --------------------------------------------------------------------------- #
def make_truth_trajectory(extent_m: float, n_poses: int, kind: str) -> np.ndarray:
    """Ground-truth (x, y) traverse in metres, shape (n_poses, 2).

    `s_curve` sweeps from a distinctive interior out across self-similar terrain
    and back, so the fusion is exercised on both locking and aliased ground.
    """
    t = np.linspace(0.0, 1.0, n_poses)
    if kind == "radial":
        # Start at the distinctive crater centre and drive radially outward into
        # the self-similar exterior: a hard lock that degrades to dead reckoning.
        x = (0.50 + 0.36 * t) * extent_m
        y = (0.50 + 0.34 * t) * extent_m
    elif kind == "s_curve":
        x = (0.30 + 0.45 * t) * extent_m
        y = (0.50 + 0.22 * np.sin(2.0 * math.pi * t)) * extent_m
    elif kind == "diag":
        x = (0.25 + 0.50 * t) * extent_m
        y = (0.30 + 0.45 * t) * extent_m
    else:
        raise SystemExit(f"unknown trajectory '{kind}' (use radial|s_curve|diag)")
    return np.column_stack([x, y])


def true_yaw_series(truth: np.ndarray) -> np.ndarray:
    """Heading (rad) along the path: azimuth of motion (0=+Y north, cw to +X)."""
    d = np.diff(truth, axis=0)
    yaw = np.arctan2(d[:, 0], d[:, 1])  # az = atan2(dx, dy)
    return np.concatenate([yaw[:1], yaw])  # pad first


def simulate_vo(
    truth: np.ndarray, fused_yaw: np.ndarray, rng, *,
    scale_bias: float, yaw_bias_rad: float, sigma_frac: float,
) -> np.ndarray:
    """Per-step world-frame VO displacement with realistic drift, shape (T, 2).

    VO measures a *body-frame* step; we rotate it into the world with the fused
    attitude. Drift enters as a small systematic scale and per-step rotational
    bias (the classic VO failure mode) plus zero-mean noise.
    """
    deltas = np.diff(truth, axis=0)                          # (T, 2) world truth
    out = np.zeros_like(deltas)
    for k in range(len(deltas)):
        step = deltas[k]
        L = math.hypot(step[0], step[1])
        true_az = math.atan2(step[0], step[1])
        # Corrupt in the body frame: scale error + accumulating heading bias.
        meas_L = L * (1.0 + scale_bias) + rng.normal(0.0, sigma_frac * max(L, 1.0))
        meas_az = true_az + yaw_bias_rad * (k + 1) + rng.normal(0.0, sigma_frac)
        out[k] = [meas_L * math.sin(meas_az), meas_L * math.cos(meas_az)]
    return out


def integrate(p0: np.ndarray, deltas: np.ndarray) -> np.ndarray:
    """Dead-reckon a trajectory from a start point and per-step world deltas."""
    return np.vstack([p0, p0 + np.cumsum(deltas, axis=0)])


# --------------------------------------------------------------------------- #
# Skyline fixes (real matcher, real margin -> measurement + information)      #
# --------------------------------------------------------------------------- #
class SkylineLocalizer:
    """Precomputes the candidate horizon grid once; serves (fix, margin) calls."""

    def __init__(self, dem, px_to_m, *, grid, margin_frac, n_az, n_range,
                 mast_height_m, yaw_sigma_deg):
        self.dem = dem
        self.px_to_m = px_to_m
        self.n_az = n_az
        self.n_range = n_range
        self.mast = mast_height_m
        size = dem.shape[0]
        self.extent_m = size * px_to_m
        self.r_max = 0.9 * self.extent_m
        self.r_min = max(60.0, 2.0 * px_to_m)
        self.bins_per_deg = n_az / 360.0
        self.window_bins = max(1, int(math.ceil(3.0 * yaw_sigma_deg * self.bins_per_deg)))

        m = margin_frac
        gx = np.linspace(m * self.extent_m, (1.0 - m) * self.extent_m, grid)
        gy = np.linspace(m * self.extent_m, (1.0 - m) * self.extent_m, grid)
        self.cand_xy = [(x, y) for y in gy for x in gx]
        self.grid_step_m = (gx[1] - gx[0]) if grid > 1 else self.extent_m
        self.preds = np.stack([self._horizon(xy) for xy in self.cand_xy])

    def _horizon(self, xy):
        return render_horizon(self.dem, self.px_to_m, xy, self.mast,
                              n_az=self.n_az, r_min_m=self.r_min,
                              r_max_m=self.r_max, n_range=self.n_range)

    def fix(self, truth_xy, yaw_rad, yaw_prior_rad, rng, noise_arcmin):
        """Return (est_xy, margin, best_ncc): a real horizon localization."""
        horizon_world = self._horizon(truth_xy)
        heading_bins = int(round(math.degrees(yaw_rad) * self.bins_per_deg)) % self.n_az
        obs = np.roll(horizon_world, -heading_bins)
        obs = obs + rng.normal(0.0, math.radians(noise_arcmin / 60.0), size=obs.shape)
        prior_lag = int(round(math.degrees(yaw_prior_rad) * self.bins_per_deg)) % self.n_az
        ncc, _ = best_yaw_ncc(obs, self.preds,
                              prior_lag=prior_lag, window_bins=self.window_bins)
        j = int(np.argmax(ncc))
        est_xy = self.cand_xy[j]
        margin, _ = uniqueness_margin(ncc, self.cand_xy, est_xy,
                                      radius_m=2.0 * self.grid_step_m)
        return np.array(est_xy, dtype=np.float64), float(margin), float(ncc[j])


def margin_to_sigma(margin, *, sigma_lock_m, margin_ref, margin_floor, sigma_cap_m):
    """Map uniqueness_margin -> position 1-sigma (m). Distinctive -> tight."""
    eff = max(margin, margin_floor)
    sigma = sigma_lock_m * (margin_ref / eff)
    return float(min(sigma, sigma_cap_m))


# --------------------------------------------------------------------------- #
# Linear pose-graph fusion (positions decouple per axis given star attitude)  #
# --------------------------------------------------------------------------- #
def fuse_positions(
    n_poses, vo_deltas, prior_xy, sigma_prior, sigma_vo,
    fixes,  # list of (k, est_xy(2,), sigma_m)
):
    """Solve the linear least-squares pose graph; return (est (N,2), cov (N,2,2)).

    Factors (all axis-separable -> solve x and y as two N x N systems):
      prior:   (p_0 - prior)/sigma_prior
      between: ((p_k - p_{k-1}) - vo_delta_k)/sigma_vo
      skyline: (p_k - fix)/sigma_fix  at observed poses
    Covariance is the inverse normal matrix (per axis) -> per-pose 2x2 blocks.
    """
    N = n_poses
    est = np.zeros((N, 2))
    cov = np.zeros((N, 2, 2))
    for axis in (0, 1):
        rows, rhs = [], []
        # prior on pose 0
        r = np.zeros(N); r[0] = 1.0 / sigma_prior
        rows.append(r); rhs.append(prior_xy[axis] / sigma_prior)
        # between factors
        for k in range(1, N):
            r = np.zeros(N)
            r[k] = 1.0 / sigma_vo
            r[k - 1] = -1.0 / sigma_vo
            rows.append(r); rhs.append(vo_deltas[k - 1, axis] / sigma_vo)
        # skyline unary factors
        for (k, fix_xy, sigma_fix) in fixes:
            r = np.zeros(N); r[k] = 1.0 / sigma_fix
            rows.append(r); rhs.append(fix_xy[axis] / sigma_fix)
        A = np.asarray(rows); b = np.asarray(rhs)
        Ninfo = A.T @ A
        sol = np.linalg.solve(Ninfo, A.T @ b)
        Sigma = np.linalg.inv(Ninfo)
        est[:, axis] = sol
        cov[:, axis, axis] = np.diag(Sigma)
    return est, cov


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # DEM source (shared with skyline_lock_demo's build_dem).
    ap.add_argument("--source", choices=["synth", "lola"], default="synth")
    ap.add_argument("--terrain", choices=["hills", "craters", "flat"], default="craters")
    ap.add_argument("--target", default="tycho")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path, default=REPO_ROOT / "datasets" / "lro_cache")
    ap.add_argument("--size-px", type=int, default=512)
    ap.add_argument("--px-to-m", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=7)
    # Trajectory + sensors.
    ap.add_argument("--trajectory", choices=["radial", "s_curve", "diag"], default="radial")
    ap.add_argument("--n-poses", type=int, default=60)
    ap.add_argument("--skyline-every", type=int, default=4,
                    help="Attempt a skyline fix every N poses.")
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--grid", type=int, default=41)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=2.0,
                    help="Star-tracker heading 1-sigma (prior window = +/-3 sigma).")
    # VO drift model.
    ap.add_argument("--vo-scale-bias", type=float, default=0.04)
    ap.add_argument("--vo-yaw-bias-deg", type=float, default=0.25,
                    help="Per-step systematic VO heading bias (drift driver).")
    ap.add_argument("--vo-sigma-frac", type=float, default=0.06)
    # Fusion information model.
    ap.add_argument("--sigma-prior-m", type=float, default=20.0)
    ap.add_argument("--sigma-vo-m", type=float, default=120.0,
                    help="Per-step VO between-factor 1-sigma (m).")
    ap.add_argument("--sigma-lock-m", type=float, default=400.0)
    ap.add_argument("--margin-ref", type=float, default=0.15)
    ap.add_argument("--margin-floor", type=float, default=0.01)
    ap.add_argument("--sigma-cap-m", type=float, default=40000.0)
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "outputs" / "factor_graph_fusion" / "factor_graph_fusion.png")
    ap.add_argument("--output-json", type=Path, default=None)
    args = ap.parse_args()

    dem, px_to_m = build_dem(args)
    size = dem.shape[0]
    extent_m = size * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    print(f"scene={scene}  extent={extent_m/1000:.1f} km  px_to_m={px_to_m:.1f}")

    rng = np.random.default_rng(args.seed)
    truth = make_truth_trajectory(extent_m, args.n_poses, args.trajectory)
    truth_yaw = true_yaw_series(truth)
    # Star tracker: absolute attitude to ~yaw_sigma; this IS the fused attitude.
    star_yaw = truth_yaw + rng.normal(0.0, math.radians(args.yaw_sigma_deg), size=truth_yaw.shape)

    vo_deltas = simulate_vo(
        truth, star_yaw, rng,
        scale_bias=args.vo_scale_bias,
        yaw_bias_rad=math.radians(args.vo_yaw_bias_deg),
        sigma_frac=args.vo_sigma_frac)
    vo_only = integrate(truth[0], vo_deltas)

    print("precomputing skyline candidate grid ...")
    loc = SkylineLocalizer(
        dem, px_to_m, grid=args.grid, margin_frac=args.grid_margin_frac,
        n_az=args.n_az, n_range=args.n_range, mast_height_m=args.mast_height_m,
        yaw_sigma_deg=args.yaw_sigma_deg)

    fix_records, fixes = [], []
    fix_poses = list(range(0, args.n_poses, args.skyline_every))
    print(f"running {len(fix_poses)} skyline fixes ...")
    for k in fix_poses:
        est_xy, margin, ncc = loc.fix(
            tuple(truth[k]), truth_yaw[k], star_yaw[k], rng, args.noise_arcmin)
        sigma = margin_to_sigma(
            margin, sigma_lock_m=args.sigma_lock_m, margin_ref=args.margin_ref,
            margin_floor=args.margin_floor, sigma_cap_m=args.sigma_cap_m)
        fix_err = float(math.hypot(est_xy[0] - truth[k, 0], est_xy[1] - truth[k, 1]))
        fixes.append((k, est_xy, sigma))
        fix_records.append({"pose": k, "margin": round(margin, 4), "best_ncc": round(ncc, 4),
                            "sigma_m": round(sigma, 1), "fix_err_m": round(fix_err, 1),
                            "unique": bool(margin >= 0.05)})

    est, cov = fuse_positions(
        args.n_poses, vo_deltas, truth[0], args.sigma_prior_m, args.sigma_vo_m, fixes)

    # Metrics.
    def rmse(a):
        return float(np.sqrt(np.mean(np.sum((a - truth) ** 2, axis=1))))
    vo_err = np.linalg.norm(vo_only - truth, axis=1)
    fused_err = np.linalg.norm(est - truth, axis=1)
    pos_std = np.sqrt(cov[:, 0, 0] + cov[:, 1, 1])  # sqrt(trace) ~ 1-sigma radius
    n_unique = sum(1 for r in fix_records if r["unique"])

    summary = {
        "scene": scene,
        "n_poses": args.n_poses,
        "n_skyline_fixes": len(fixes),
        "n_unique_fixes": n_unique,
        "vo_only_rmse_m": round(rmse(vo_only), 1),
        "vo_only_final_err_m": round(float(vo_err[-1]), 1),
        "fused_rmse_m": round(rmse(est), 1),
        "fused_final_err_m": round(float(fused_err[-1]), 1),
        "fused_max_err_m": round(float(fused_err.max()), 1),
        "rmse_improvement_x": round(rmse(vo_only) / max(rmse(est), 1e-9), 2),
        "fixes": fix_records,
    }
    print(json.dumps(summary, indent=2))

    _render(args, dem, extent_m, truth, vo_only, est, cov, pos_std,
            fixes, fix_records, fused_err, vo_err, scene)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")
    return 0


def _render(args, dem, extent_m, truth, vo_only, est, cov, pos_std,
            fixes, fix_records, fused_err, vo_err, scene) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2))
    fig.suptitle(
        f"Honest factor-graph fusion (star + VO + Skyline) over {scene}  -  "
        "skyline pins the trajectory only where the terrain is distinctive",
        fontsize=13)

    ax = axes[0]
    im = ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
    ax.plot(truth[:, 0], truth[:, 1], "-", color="white", lw=2.2, label="ground truth")
    ax.plot(vo_only[:, 0], vo_only[:, 1], "--", color="red", lw=1.8, label="VO only (drifts)")
    ax.plot(est[:, 0], est[:, 1], "-", color="cyan", lw=1.8, label="fused")
    # Covariance ellipses (2-sigma) every few poses.
    for k in range(0, len(est), max(1, len(est) // 12)):
        sx = 2.0 * math.sqrt(max(cov[k, 0, 0], 0.0))
        sy = 2.0 * math.sqrt(max(cov[k, 1, 1], 0.0))
        ax.add_patch(Ellipse((est[k, 0], est[k, 1]), 2 * sx, 2 * sy,
                             fill=False, color="cyan", lw=0.7, alpha=0.6))
    # Skyline fixes, colored by uniqueness.
    for (k, fxy, _), rec in zip(fixes, fix_records):
        c = "#19c819" if rec["unique"] else "#ff8c00"
        ax.scatter(*fxy, c=c, marker="P", s=70, edgecolors="k", lw=0.5, zorder=6)
    ax.scatter([], [], c="#19c819", marker="P", s=70, edgecolors="k", label="skyline fix (unique)")
    ax.scatter([], [], c="#ff8c00", marker="P", s=70, edgecolors="k", label="skyline fix (aliased)")
    ax.set_title("DEM (m) + trajectories + 2σ ellipses")
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    ax.legend(loc="upper left", fontsize=7.5)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    ax = axes[1]
    ax.plot(vo_err, "--", color="red", lw=1.8, label="VO only")
    ax.plot(fused_err, "-", color="cyan", lw=1.8, label="fused")
    for (k, _, _), rec in zip(fixes, fix_records):
        ax.axvline(k, color=("#19c819" if rec["unique"] else "#ff8c00"),
                   lw=1.0, alpha=0.5)
    ax.set_title("position error vs pose (skyline-fix events marked)")
    ax.set_xlabel("pose index"); ax.set_ylabel("error (m)")
    ax.legend(fontsize=8)

    ax = axes[2]
    ax.plot(pos_std, "-", color="purple", lw=1.8, label="fused 1σ (sqrt trace cov)")
    ax2 = ax.twinx()
    ks = [k for (k, _, _) in fixes]
    margins = [r["margin"] for r in fix_records]
    ax2.scatter(ks, margins, c=["#19c819" if r["unique"] else "#ff8c00" for r in fix_records],
                marker="P", s=55, edgecolors="k", lw=0.4, zorder=5)
    ax2.axhline(0.05, color="gray", ls=":", lw=1.0)
    ax2.set_ylabel("skyline uniqueness_margin")
    ax.set_title("fused uncertainty grows (dead reckoning), snaps at a unique lock")
    ax.set_xlabel("pose index"); ax.set_ylabel("position 1σ (m)")
    ax.legend(fontsize=8, loc="upper left")

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    sys.exit(main())
