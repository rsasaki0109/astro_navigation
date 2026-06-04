#!/usr/bin/env python3
"""Skyline Lock Phase 7 (1): promote yaw to a graph state -- a nonlinear SO(2)
factor graph, solved with a self-contained Gauss-Newton on the circle.

Phase 5 (`factor_graph_fusion_demo.py`) fused star + VO + Skyline as a *linear*
pose graph: it fixed each pose's attitude to the star tracker and rotated VO's
body-frame deltas into the world with it. Positions then decouple per axis and
solve in closed form. That is exactly right -- WHILE the star tracker is healthy.

This phase asks the honest follow-up: what happens across a star-tracker
*blackout* (sun glare, limited sky, a dropped frame)? With attitude fixed, the
estimator can only dead-reckon yaw *forward* from the last good fix, and the VO
heading bias makes that error ramp -- by the far side of a blackout the heading
is several degrees off and every VO step is rotated into the wrong world
direction. Promoting yaw to a graph state turns the estimator into a batch
smoother on SE(2)-ish states (x, y, theta): VO *yaw-increment* factors chain the
gap, so the star fixes that resume *after* the blackout flow backward through
that chain and correct the attitude *during* it. The solve is a plain
Gauss-Newton with an SO(2) retraction (angles add, then wrap) and analytic
Jacobians -- ~120 lines, no GTSAM/Ceres dependency, fully reproducible.

The honest envelope (both visible in the output, two scenes side by side):
  * Mid-traverse blackout (a skyline lock resumes on the far side): the joint
    solve recovers heading to ~1 deg where the fixed-yaw baseline drifts to
    ~9 deg; the dead-reckoned arc snaps back onto truth.
  * End-of-traverse blackout (no fix ever comes back): there is nothing to
    smooth backward from, so the joint solve does NO better than fixed-yaw --
    batch smoothing needs a future anchor. The cliff.

Factors (information = 1/sigma; the skyline sigma comes from the REAL
uniqueness_margin, same as Phase 5):
  prior   : (p_0 - prior)/sig_p ,  wrap(th_0 - star_0)/sig_star
  between : (R(th_{k-1}) d_body_k - (p_k - p_{k-1}))/sig_vo_p
            wrap(th_{k-1} + dyaw_k - th_k)/sig_vo_th
  star    : wrap(th_k - star_k)/sig_star          (healthy poses only)
  skyline : (p_k - fix_k)/sig_fix                 (unique locks only)

Conventions match skyline_lock_demo.py: world +X east, +Y north, +Z up;
azimuth 0 = +Y (north), clockwise to +X; so a body-forward step (0, L) maps to
world (L sin th, L cos th) and R(th) = [[cos, sin], [-sin, cos]].
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

from skyline_lock_demo import build_dem  # noqa: E402
from factor_graph_fusion_demo import (  # noqa: E402
    make_truth_trajectory, true_yaw_series, integrate,
    SkylineLocalizer, margin_to_sigma, fuse_positions,
)


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def Rmat(th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, s], [-s, c]])


def dRmat(th: float) -> np.ndarray:
    """d R(th) / d th."""
    c, s = math.cos(th), math.sin(th)
    return np.array([[-s, c], [-c, -s]])


# --------------------------------------------------------------------------- #
# Sensors                                                                     #
# --------------------------------------------------------------------------- #
def simulate_body_vo(truth, truth_yaw, rng, *, scale_bias, yaw_bias_rad, sigma_frac):
    """Body-frame VO between measurements: (d_body (T,2), dyaw (T,)).

    The rover drives forward, so the true body step is ~(0, L). VO corrupts it
    with a small scale error and zero-mean noise, and reports a yaw increment
    with an accumulating per-step heading bias -- the classic VO drift driver.
    """
    d = np.diff(truth, axis=0)
    T = len(d)
    d_body = np.zeros((T, 2))
    dyaw = np.zeros(T)
    for k in range(T):
        L = math.hypot(d[k, 0], d[k, 1])
        bf = Rmat(truth_yaw[k]).T @ d[k]                      # true body delta
        d_body[k] = bf * (1.0 + scale_bias) + rng.normal(0.0, sigma_frac * max(L, 1.0), 2)
        dyaw[k] = (wrap(truth_yaw[k + 1] - truth_yaw[k])
                   + yaw_bias_rad + rng.normal(0.0, math.radians(0.3)))
    return d_body, dyaw


def deadreckon_yaw(star_yaw, dyaw, blackout):
    """Phase-5 attitude: star where healthy, VO yaw-rate forward-integrated in
    the blackout (the only causal option when attitude is a fixed input)."""
    yaw = star_yaw.copy()
    for k in range(1, len(yaw)):
        if k in blackout:
            yaw[k] = yaw[k - 1] + dyaw[k - 1]
    return yaw


# --------------------------------------------------------------------------- #
# Nonlinear SO(2) factor graph -- self-contained Gauss-Newton                 #
# --------------------------------------------------------------------------- #
class So2PoseGraph:
    """States z = [x_0,y_0,th_0, x_1,...]; 3 per pose. Dense GN (N <~ 80)."""

    def __init__(self, n, *, prior_xy, prior_yaw, d_body, dyaw,
                 star_yaw, healthy, fixes, sig_p, sig_star, sig_vo_p,
                 sig_vo_th, lam0=1e-3):
        self.n = n
        self.prior_xy = np.asarray(prior_xy, float)
        self.prior_yaw = float(prior_yaw)
        self.d_body = d_body
        self.dyaw = dyaw
        self.star_yaw = star_yaw
        self.healthy = set(healthy)
        self.fixes = fixes                  # list of (k, fix_xy(2,), sigma_m)
        self.sig_p, self.sig_star = sig_p, sig_star
        self.sig_vo_p, self.sig_vo_th = sig_vo_p, sig_vo_th
        self.lam0 = lam0

    def residual_jac(self, z):
        n = self.n
        R, rows = [], []

        def push(rvals, jac):
            for rv, jd in zip(rvals, jac):
                R.append(rv)
                row = np.zeros(3 * n)
                for col, val in jd.items():
                    row[col] = val
                rows.append(row)

        # prior on pose 0 (position + attitude)
        sp, ss = self.sig_p, self.sig_star
        push([(z[0] - self.prior_xy[0]) / sp,
              (z[1] - self.prior_xy[1]) / sp,
              float(wrap(z[2] - self.prior_yaw)) / ss],
             [{0: 1 / sp}, {1: 1 / sp}, {2: 1 / ss}])

        # VO between factors (body-frame; nonlinear through R(theta))
        svp, svt = self.sig_vo_p, self.sig_vo_th
        for k in range(1, n):
            a, b = 3 * (k - 1), 3 * k
            tha = z[a + 2]
            wd = Rmat(tha) @ self.d_body[k - 1]
            dwd = dRmat(tha) @ self.d_body[k - 1]
            rp = wd - np.array([z[b] - z[a], z[b + 1] - z[a + 1]])
            ry = float(wrap(tha + self.dyaw[k - 1] - z[b + 2]))
            push([rp[0] / svp, rp[1] / svp, ry / svt],
                 [{a: 1 / svp, b: -1 / svp, a + 2: dwd[0] / svp},
                  {a + 1: 1 / svp, b + 1: -1 / svp, a + 2: dwd[1] / svp},
                  {a + 2: 1 / svt, b + 2: -1 / svt}])

        # star attitude unary (healthy poses only)
        for k in self.healthy:
            push([float(wrap(z[3 * k + 2] - self.star_yaw[k])) / ss],
                 [{3 * k + 2: 1 / ss}])

        # skyline position unary (unique locks only)
        for (k, fxy, sig) in self.fixes:
            push([(z[3 * k] - fxy[0]) / sig, (z[3 * k + 1] - fxy[1]) / sig],
                 [{3 * k: 1 / sig}, {3 * k + 1: 1 / sig}])

        return np.asarray(R), np.asarray(rows)

    @staticmethod
    def retract(z, dz):
        """SO(2) retraction: add, then wrap the per-pose angles."""
        z2 = z + dz
        z2[2::3] = wrap(z2[2::3])
        return z2

    def solve(self, z0, iters=25):
        """Levenberg-damped Gauss-Newton. Returns (z_final, history) where
        history is the list of iterates (for animation)."""
        z = z0.copy()
        lam = self.lam0
        hist = [z.copy()]
        I = np.eye(3 * self.n)
        for _ in range(iters):
            r, J = self.residual_jac(z)
            H = J.T @ J + lam * I
            g = J.T @ r
            dz = np.linalg.solve(H, -g)
            z2 = self.retract(z, dz)
            r2, _ = self.residual_jac(z2)
            if 0.5 * r2 @ r2 < 0.5 * r @ r:
                z = z2
                lam = max(lam * 0.5, 1e-7)
            else:
                lam *= 4.0
            hist.append(z.copy())
        return z, hist


def split(z):
    """z -> (positions (N,2), yaw (N,))."""
    return np.column_stack([z[0::3], z[1::3]]), z[2::3].copy()


# --------------------------------------------------------------------------- #
# Scene                                                                       #
# --------------------------------------------------------------------------- #
def run_scene(dem, px_to_m, args, blackout, *, rng_seed):
    """Run both estimators over one blackout configuration; return a dict."""
    size = dem.shape[0]
    extent_m = size * px_to_m
    rng = np.random.default_rng(rng_seed)

    truth = make_truth_trajectory(extent_m, args.n_poses, args.trajectory)
    truth_yaw = true_yaw_series(truth)
    star_yaw = truth_yaw + rng.normal(0.0, math.radians(args.yaw_sigma_deg),
                                      size=truth_yaw.shape)
    healthy = [k for k in range(args.n_poses) if k not in blackout]

    d_body, dyaw = simulate_body_vo(
        truth, truth_yaw, rng, scale_bias=args.vo_scale_bias,
        yaw_bias_rad=math.radians(args.vo_yaw_bias_deg), sigma_frac=args.vo_sigma_frac)

    loc = SkylineLocalizer(
        dem, px_to_m, grid=args.grid, margin_frac=args.grid_margin_frac,
        n_az=args.n_az, n_range=args.n_range, mast_height_m=args.mast_height_m,
        yaw_sigma_deg=args.yaw_sigma_deg)

    # Skyline needs the star yaw prior to break rotational ambiguity (Phase 1),
    # so it can only lock at healthy poses. We do NOT hard-reject low-margin
    # fixes -- following Phase 5, the margin sets the information (sigma), so an
    # aliased lock self-down-weights instead of being trusted or discarded.
    fixes, fix_records = [], []
    for k in range(0, args.n_poses, args.skyline_every):
        if k in blackout:
            continue
        est_xy, margin, ncc = loc.fix(
            tuple(truth[k]), truth_yaw[k], star_yaw[k], rng, args.noise_arcmin)
        sig = margin_to_sigma(
            margin, sigma_lock_m=args.sigma_lock_m, margin_ref=args.margin_ref,
            margin_floor=args.margin_floor, sigma_cap_m=args.sigma_cap_m)
        fixes.append((k, est_xy, sig))
        fix_records.append({"pose": k, "margin": round(margin, 4),
                            "sigma_m": round(sig, 1),
                            "unique": bool(margin >= args.margin_unique)})

    sig_star = math.radians(args.yaw_sigma_deg)

    # --- Method A: Phase-5 fixed-yaw (linear position smoother) ---
    yaw_fixed = deadreckon_yaw(star_yaw, dyaw, blackout)
    world_delta = np.array([Rmat(yaw_fixed[k]) @ d_body[k]
                            for k in range(args.n_poses - 1)])
    est_fixed, _ = fuse_positions(
        args.n_poses, world_delta, truth[0], args.sigma_prior_m,
        args.sigma_vo_m, fixes)

    # --- Method B: joint nonlinear SO(2) graph ---
    graph = So2PoseGraph(
        args.n_poses, prior_xy=truth[0], prior_yaw=star_yaw[0],
        d_body=d_body, dyaw=dyaw, star_yaw=star_yaw, healthy=healthy,
        fixes=fixes, sig_p=args.sigma_prior_m, sig_star=sig_star,
        sig_vo_p=args.sigma_vo_m, sig_vo_th=math.radians(args.sigma_vo_yaw_deg))
    z0 = np.zeros(3 * args.n_poses)
    z0[0::3] = est_fixed[:, 0]
    z0[1::3] = est_fixed[:, 1]
    z0[2::3] = yaw_fixed
    z_fin, hist = graph.solve(z0, iters=args.gn_iters)
    est_joint, yaw_joint = split(z_fin)

    bo = sorted(blackout)
    yaw_err_fixed = np.degrees(np.abs(wrap(yaw_fixed - truth_yaw)))
    yaw_err_joint = np.degrees(np.abs(wrap(yaw_joint - truth_yaw)))
    pe_fixed = np.linalg.norm(est_fixed - truth, axis=1)
    pe_joint = np.linalg.norm(est_joint - truth, axis=1)

    def rmse(e):
        return float(np.sqrt(np.mean(np.sum((e - truth) ** 2, axis=1))))

    return {
        "blackout": bo,
        "truth": truth, "truth_yaw": truth_yaw,
        "est_fixed": est_fixed, "est_joint": est_joint,
        "yaw_fixed": yaw_fixed, "yaw_joint": yaw_joint,
        "yaw_err_fixed": yaw_err_fixed, "yaw_err_joint": yaw_err_joint,
        "pe_fixed": pe_fixed, "pe_joint": pe_joint,
        "fixes": fixes, "fix_records": fix_records,
        "hist": hist, "graph": graph, "extent_m": extent_m,
        "metrics": {
            "rmse_fixed_m": round(rmse(est_fixed), 1),
            "rmse_joint_m": round(rmse(est_joint), 1),
            "blackout_yaw_err_fixed_deg": round(float(yaw_err_fixed[bo].mean()), 2),
            "blackout_yaw_err_joint_deg": round(float(yaw_err_joint[bo].mean()), 2),
            "blackout_yaw_err_fixed_max_deg": round(float(yaw_err_fixed[bo].max()), 2),
            "blackout_yaw_err_joint_max_deg": round(float(yaw_err_joint[bo].max()), 2),
            "blackout_pos_err_fixed_m": round(float(pe_fixed[bo].mean()), 1),
            "blackout_pos_err_joint_m": round(float(pe_joint[bo].mean()), 1),
            "n_skyline_locks": len(fixes),
        },
    }


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #
def build_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["synth", "lola"], default="synth")
    ap.add_argument("--terrain", choices=["hills", "craters", "flat"], default="craters")
    ap.add_argument("--target", default="tycho")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path, default=REPO_ROOT / "datasets" / "lro_cache")
    ap.add_argument("--size-px", type=int, default=384)
    ap.add_argument("--px-to-m", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--trajectory", choices=["radial", "s_curve", "diag"], default="radial")
    ap.add_argument("--n-poses", type=int, default=48)
    ap.add_argument("--skyline-every", type=int, default=4)
    ap.add_argument("--blackout-start", type=int, default=18)
    ap.add_argument("--blackout-len", type=int, default=16)
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--n-range", type=int, default=140)
    ap.add_argument("--grid", type=int, default=61)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=1.0)
    ap.add_argument("--margin-unique", type=float, default=0.05)
    # VO drift.
    ap.add_argument("--vo-scale-bias", type=float, default=0.03)
    ap.add_argument("--vo-yaw-bias-deg", type=float, default=0.7)
    ap.add_argument("--vo-sigma-frac", type=float, default=0.02)
    # Information model.
    ap.add_argument("--sigma-prior-m", type=float, default=20.0)
    ap.add_argument("--sigma-vo-m", type=float, default=30.0)
    ap.add_argument("--sigma-vo-yaw-deg", type=float, default=2.0)
    ap.add_argument("--sigma-lock-m", type=float, default=400.0)
    ap.add_argument("--margin-ref", type=float, default=0.15)
    ap.add_argument("--margin-floor", type=float, default=0.01)
    ap.add_argument("--sigma-cap-m", type=float, default=40000.0)
    ap.add_argument("--gn-iters", type=int, default=20)
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_so2.png")
    ap.add_argument("--output-json", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_so2.json")
    return ap


def main() -> int:
    args = build_args().parse_args()
    dem, px_to_m = build_dem(args)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    print(f"scene={scene}  extent={extent_m/1000:.1f} km  px_to_m={px_to_m:.1f}")

    bs, bl = args.blackout_start, args.blackout_len
    mid = set(range(bs, bs + bl))
    end = set(range(args.n_poses - bl, args.n_poses))
    print("solving mid-traverse blackout ...")
    mid_r = run_scene(dem, px_to_m, args, mid, rng_seed=args.seed)
    print("solving end-of-traverse blackout (the cliff) ...")
    end_r = run_scene(dem, px_to_m, args, end, rng_seed=args.seed)

    summary = {
        "scene": scene,
        "n_poses": args.n_poses,
        "yaw_sigma_deg": args.yaw_sigma_deg,
        "vo_yaw_bias_deg_per_step": args.vo_yaw_bias_deg,
        "mid_traverse_blackout": {"poses": mid_r["blackout"], **mid_r["metrics"]},
        "end_traverse_blackout": {"poses": end_r["blackout"], **end_r["metrics"]},
    }
    print(json.dumps(summary, indent=2))

    _render(args, dem, scene, mid_r, end_r)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")
    return 0


FIXED = "#ff8c00"
JOINT = "#1f77ff"


def _render(args, dem, scene, mid_r, end_r) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    extent_m = mid_r["extent_m"]
    fig, axes = plt.subplots(2, 3, figsize=(17, 10))
    fig.suptitle(
        f"Skyline Lock Phase 7(1): yaw as a graph state (nonlinear SO(2) "
        f"Gauss-Newton) over {scene}\n"
        "a star-tracker blackout -- batch smoothing recovers heading only when a "
        "fix returns on the far side",
        fontsize=13)

    for row, (r, label) in enumerate([(mid_r, "mid-traverse blackout (a lock resumes after)"),
                                      (end_r, "end-of-traverse blackout (no fix ever returns)")]):
        truth = r["truth"]
        bo = r["blackout"]
        m = r["metrics"]

        # col 0: map
        ax = axes[row][0]
        ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(truth[:, 0], truth[:, 1], "-", color="white", lw=2.4, label="ground truth")
        ax.plot(r["est_fixed"][:, 0], r["est_fixed"][:, 1], "--", color=FIXED, lw=1.9,
                label=f"fixed-yaw (Phase 5)  rmse {m['rmse_fixed_m']:.0f} m")
        ax.plot(r["est_joint"][:, 0], r["est_joint"][:, 1], "-", color=JOINT, lw=1.9,
                label=f"joint SO(2)        rmse {m['rmse_joint_m']:.0f} m")
        # shade the truth poses inside the blackout
        ax.plot(truth[bo, 0], truth[bo, 1], "o", color="k", ms=3.0, alpha=0.5,
                label="blackout poses")
        rec_by_pose = {rc["pose"]: rc for rc in r["fix_records"]}
        for (k, fxy, _) in r["fixes"]:
            c = "#19c819" if rec_by_pose[k]["unique"] else "#ff8c00"
            ax.scatter(*fxy, c=c, marker="P", s=55, edgecolors="k", lw=0.4, zorder=6)
        ax.scatter([], [], c="#19c819", marker="P", s=55, edgecolors="k",
                   label="skyline lock (margin-weighted)")
        ax.set_title(label, fontsize=10.5)
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=7.2)

        # col 1: yaw error
        ax = axes[row][1]
        ax.axvspan(bo[0], bo[-1], color="0.85", label="blackout")
        ax.plot(r["yaw_err_fixed"], "--", color=FIXED, lw=1.9, label="fixed-yaw")
        ax.plot(r["yaw_err_joint"], "-", color=JOINT, lw=1.9, label="joint SO(2)")
        ax.set_title(
            f"heading error  --  blackout mean "
            f"{m['blackout_yaw_err_fixed_deg']:.1f}° → "
            f"{m['blackout_yaw_err_joint_deg']:.1f}°", fontsize=10.5)
        ax.set_xlabel("pose index"); ax.set_ylabel("|yaw error| (deg)")
        ax.legend(fontsize=8)

        # col 2: position error
        ax = axes[row][2]
        ax.axvspan(bo[0], bo[-1], color="0.85", label="blackout")
        ax.plot(r["pe_fixed"], "--", color=FIXED, lw=1.9, label="fixed-yaw")
        ax.plot(r["pe_joint"], "-", color=JOINT, lw=1.9, label="joint SO(2)")
        ax.set_title(
            f"position error  --  blackout mean "
            f"{m['blackout_pos_err_fixed_m']:.0f} → "
            f"{m['blackout_pos_err_joint_m']:.0f} m", fontsize=10.5)
        ax.set_xlabel("pose index"); ax.set_ylabel("position error (m)")
        ax.legend(fontsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    sys.exit(main())
