#!/usr/bin/env python3
"""Skyline Lock Phase 7 (3): a stereo baseline turns the scale gauge into state.

Phase 7 (2) (`factor_graph_so3_demo.py`) ended on an honest cliff: with no
absolute fix anywhere on the traverse, the global metric scale is a pure gauge
freedom. Monocular VO reports translation only up to scale, so the whole map is
similarity-ambiguous and the joint solve leaves the scale state at its initial
1.000 -- it has nothing to pull on. The only thing that recovered scale there was
a pair of absolute Skyline fixes BRACKETING the VO chain, pinning its true length.

This demo adds the other honest way to get scale: a **stereo camera**. A rig with
a known baseline triangulates metric depth, so a stereo-PnP step between two
frames is a *metric* relative translation -- it observes the true body-frame step
length directly, not up to scale. Dropped into the same self-contained SO(3) x
scale graph as a unary factor

    r_stereo(k) = ( s * vo_t[k] - t_stereo[k] ) / sigma_stereo

it makes the scale observable from the VO chain ALONE, with zero absolute fixes.
The gauge freedom is gone.

But there is no free lunch, and the figure shows the price honestly:

  * Perfect baseline -> the recovered scale lands on truth (s 1.000 -> 0.923,
    truth 0.926) from stereo alone, no Skyline lock required.
  * The catch is a CALIBRATION dependency. A mis-calibrated stereo baseline (the
    triangulated depth is off by a constant factor) biases the recovered scale
    almost exactly proportionally -- a +/-3% baseline error pulls s to ~0.96 /
    ~0.89, and since scale multiplies every VO step, that error propagates
    straight into the map. Stereo does not remove the failure mode; it trades an
    *unobservable* gauge freedom for a *calibration-sensitive* one you can at
    least measure and bound.

So the Phase 7 (2) triptych grows a fourth honest answer for the scale DOF:
recoverable from absolute fixes that bracket the chain, OR from a stereo baseline
locally -- and in the second case your map is only as metric as your baseline.

Reuses the SO(3) x scale solver, sensors and scene from factor_graph_so3_demo.py
unchanged (So3StereoGraph subclasses So3ScaleGraph and only appends the stereo
residual), so the Phase 7 (2) demo is byte-for-byte untouched.
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
from factor_graph_so3_demo import (  # noqa: E402
    Surface, make_truth_3d, simulate_sensors, forward_filter, attitude_errs,
    So3ScaleGraph, build_args,
)


# --------------------------------------------------------------------------- #
# Stereo metric-translation sensor + the stereo-augmented graph               #
# --------------------------------------------------------------------------- #
def simulate_stereo(P, R, n_poses, rng, *, baseline_err, sigma_frac, every):
    """Per-step metric body-frame translation from a stereo rig.

    A perfectly-calibrated rig observes the true body step t_body exactly (plus
    triangulation noise). A baseline mis-calibration scales every triangulated
    depth by (1 + baseline_err), so the measured step is (1+baseline_err)*t_body
    -- a constant multiplicative error, which is exactly how a wrong baseline
    shows up. Returns a list of (k, t_meas(3), sigma)."""
    stereo = []
    for k in range(0, n_poses - 1, every):
        t_body = R[k].T @ (P[k + 1] - P[k])
        mag = max(float(np.linalg.norm(t_body)), 1.0)
        t_meas = (1.0 + baseline_err) * t_body + rng.normal(0.0, sigma_frac * mag, 3)
        stereo.append((k, t_meas, max(sigma_frac * mag, 0.05)))
    return stereo


class So3StereoGraph(So3ScaleGraph):
    """So3ScaleGraph + a unary stereo metric-translation factor on each stereo
    step. Adds residuals only (no new states), so the inherited retract / numjac
    / solve work unchanged."""

    def __init__(self, *a, stereo=None, **kw):
        super().__init__(*a, **kw)
        self.stereo = stereo or []

    def residual(self, est):
        r = list(super().residual(est))
        s, R = est["s"], est["R"]  # noqa: F841 (R kept for parity / clarity)
        for (k, t_meas, sig) in self.stereo:
            r += list((s * self.m["vo_t"][k] - t_meas) / sig)
        return np.asarray(r)


# --------------------------------------------------------------------------- #
# Scene: solve the no-fix traverse under several stereo conditions            #
# --------------------------------------------------------------------------- #
def _build_graph(args, n, meas, P0, R0, *, stereo):
    return So3StereoGraph(
        n, meas, P0, R0, [],  # NO absolute fixes -- scale comes from stereo only
        sig_p=args.sigma_prior_m, sig_rot=math.radians(args.sigma_prior_rot_deg),
        sig_vo_p=args.sigma_vo_m, sig_vo_rot=math.radians(args.sigma_vo_rot_deg),
        sig_grav=math.radians(args.sigma_grav_deg),
        sig_star=math.radians(args.yaw_sigma_deg), use_grav=True, stereo=stereo)


def run_stereo_scene(surf, dem, px_to_m, args, *, rng_seed):
    extent_m = dem.shape[0] * px_to_m
    rng = np.random.default_rng(rng_seed)
    P, R, xy, yaw = make_truth_3d(surf, extent_m, args.n_poses, args.trajectory)

    meas = simulate_sensors(
        P, R, args.n_poses, set(), rng, scale_bias=args.vo_scale_bias,
        gyro_bias_deg=[args.gyro_pitch_bias_deg, args.gyro_roll_bias_deg,
                       args.gyro_yaw_bias_deg],
        grav_sigma_deg=args.grav_sigma_deg, star_sigma_deg=args.yaw_sigma_deg,
        vo_sigma_frac=args.vo_sigma_frac)
    s_true = meas["s_true"]

    Pa, Ra = forward_filter(meas, P[0], args.n_poses, [], use_grav=True)
    est0 = dict(P=Pa.copy(), R=Ra.copy(), s=1.0)
    R0 = meas["star"].get(0, Ra[0])

    def rmse(Pest):
        return float(np.sqrt(np.mean(np.sum((Pest - P) ** 2, axis=1))))

    # Three headline conditions, all with NO absolute fix.
    conditions = {
        "gauge_no_stereo": None,
        "stereo_perfect": simulate_stereo(P, R, args.n_poses, np.random.default_rng(rng_seed + 1),
                                          baseline_err=0.0, sigma_frac=args.stereo_sigma_frac,
                                          every=args.stereo_every),
        f"stereo_bias_p{int(args.stereo_baseline_err*100)}":
            simulate_stereo(P, R, args.n_poses, np.random.default_rng(rng_seed + 2),
                            baseline_err=args.stereo_baseline_err,
                            sigma_frac=args.stereo_sigma_frac, every=args.stereo_every),
        f"stereo_bias_m{int(args.stereo_baseline_err*100)}":
            simulate_stereo(P, R, args.n_poses, np.random.default_rng(rng_seed + 3),
                            baseline_err=-args.stereo_baseline_err,
                            sigma_frac=args.stereo_sigma_frac, every=args.stereo_every),
    }
    results = {}
    for name, stereo in conditions.items():
        g = _build_graph(args, args.n_poses, meas, P[0], R0, stereo=stereo)
        est, hist = g.solve(est0, iters=args.gn_iters)
        results[name] = {
            "s_hist": [float(e["s"]) for e in hist],
            "s_final": float(est["s"]),
            "rmse_m": round(rmse(est["P"]), 1),
            "Pest": est["P"],
        }

    # Calibration-sensitivity sweep: recovered scale vs stereo baseline error.
    sweep_err = np.linspace(-args.sweep_max, args.sweep_max, args.sweep_n)
    sweep_s = []
    for i, be in enumerate(sweep_err):
        stereo = simulate_stereo(P, R, args.n_poses, np.random.default_rng(rng_seed + 100 + i),
                                 baseline_err=float(be), sigma_frac=args.stereo_sigma_frac,
                                 every=args.stereo_every)
        g = _build_graph(args, args.n_poses, meas, P[0], R0, stereo=stereo)
        est, _ = g.solve(est0, iters=args.gn_iters)
        sweep_s.append(float(est["s"]))

    return dict(
        truth=P, truth_xy=xy, extent_m=extent_m, s_true=s_true,
        results=results, sweep_err=sweep_err, sweep_s=np.asarray(sweep_s),
        rmse_gauge=results["gauge_no_stereo"]["rmse_m"],
    )


# --------------------------------------------------------------------------- #
# Render                                                                      #
# --------------------------------------------------------------------------- #
GAUGE = "#ff8c00"
PERFECT = "#1f77ff"
BIASP = "#d62728"
BIASM = "#9467bd"
TRUTHC = "#000000"


def _render(args, dem, scene, sc):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    res = sc["results"]
    s_true = sc["s_true"]
    names = list(res.keys())
    perfect_key = "stereo_perfect"
    biasp_key = [n for n in names if n.startswith("stereo_bias_p")][0]
    biasm_key = [n for n in names if n.startswith("stereo_bias_m")][0]
    be = int(args.stereo_baseline_err * 100)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.6))
    fig.suptitle(
        f"Skyline Lock Phase 7(3): a stereo baseline turns the scale gauge into state  ({scene})\n"
        "no absolute fix anywhere -- monocular scale is a gauge freedom; a known stereo baseline "
        "makes it observable, a mis-calibrated one biases it",
        fontsize=12.5)

    # col 0: scale state vs GN iteration for the headline conditions
    ax = axes[0]
    ax.axhline(s_true, color=TRUTHC, ls="--", lw=1.4, label=f"true scale {s_true:.3f}")
    ax.axhline(1.0, color="0.6", ls=":", lw=1.2, label="assumed 1.000")
    ax.plot(res["gauge_no_stereo"]["s_hist"], "-o", color=GAUGE, lw=2.0, ms=3.5,
            label=f"no stereo -> gauge (stays {res['gauge_no_stereo']['s_final']:.3f})")
    ax.plot(res[perfect_key]["s_hist"], "-o", color=PERFECT, lw=2.0, ms=3.5,
            label=f"stereo, true baseline -> {res[perfect_key]['s_final']:.3f}")
    ax.plot(res[biasp_key]["s_hist"], "-o", color=BIASP, lw=1.7, ms=3.0,
            label=f"stereo +{be}% baseline -> {res[biasp_key]['s_final']:.3f}")
    ax.plot(res[biasm_key]["s_hist"], "-o", color=BIASM, lw=1.7, ms=3.0,
            label=f"stereo -{be}% baseline -> {res[biasm_key]['s_final']:.3f}")
    ax.set_title("scale state vs Gauss-Newton iteration", fontsize=10.5)
    ax.set_xlabel("GN iteration"); ax.set_ylabel("global VO scale s")
    ax.legend(fontsize=7.8, loc="best")

    # col 1: calibration-sensitivity curve -- recovered s vs baseline error
    ax = axes[1]
    ax.plot(sc["sweep_err"] * 100, sc["sweep_s"], "-o", color=PERFECT, lw=1.8, ms=3.0,
            label="recovered scale")
    ax.axhline(s_true, color=TRUTHC, ls="--", lw=1.2, label=f"true scale {s_true:.3f}")
    ax.axvline(0.0, color="0.6", ls=":", lw=1.0)
    # ideal proportional line s_true*(1+err) for reference
    ax.plot(sc["sweep_err"] * 100, s_true * (1.0 + sc["sweep_err"]), "--",
            color="#888888", lw=1.0, label="s_true·(1+err) ideal")
    ax.set_title("the price: recovered scale tracks the baseline error", fontsize=10.5)
    ax.set_xlabel("stereo baseline error (%)"); ax.set_ylabel("recovered scale s")
    ax.legend(fontsize=8, loc="best")

    # col 2: what the scale error costs the map (no-fix position RMSE)
    ax = axes[2]
    labels = ["no stereo\n(gauge)", "stereo\ntrue base", f"stereo\n+{be}% base", f"stereo\n-{be}% base"]
    keys = ["gauge_no_stereo", perfect_key, biasp_key, biasm_key]
    colors = [GAUGE, PERFECT, BIASP, BIASM]
    vals = [res[k]["rmse_m"] for k in keys]
    ax.bar(range(len(keys)), vals, color=colors, edgecolor="k", lw=0.5)
    for i, v in enumerate(vals):
        ax.text(i, v, f"{v:.0f} m", ha="center", va="bottom", fontsize=8.5)
    ax.set_xticks(range(len(keys))); ax.set_xticklabels(labels, fontsize=8)
    ax.set_title("no-fix map error: scale drives position", fontsize=10.5)
    ax.set_ylabel("position RMSE (m)")

    fig.tight_layout(rect=[0, 0, 1, 0.9])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


def add_stereo_args(ap):
    ap.add_argument("--stereo-every", type=int, default=3,
                    help="Add a stereo metric-translation factor every Nth VO step.")
    ap.add_argument("--stereo-sigma-frac", type=float, default=0.03,
                    help="Stereo triangulation noise as a fraction of step length.")
    ap.add_argument("--stereo-baseline-err", type=float, default=0.03,
                    help="Headline mis-calibration magnitude (fraction).")
    ap.add_argument("--sweep-max", type=float, default=0.06,
                    help="Calibration sweep half-range (fraction).")
    ap.add_argument("--sweep-n", type=int, default=13)
    return ap


def main() -> int:
    ap = build_args()
    add_stereo_args(ap)
    ap.set_defaults(
        output=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_so3_stereo.png",
        output_json=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "factor_graph_so3_stereo.json")
    args = ap.parse_args()

    dem, px_to_m = build_dem(args)
    surf = Surface(dem, px_to_m)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    print(f"scene={scene}  extent={extent_m/1000:.1f} km  px_to_m={px_to_m:.1f}")
    print("solving no-fix traverse under stereo conditions (gauge / perfect / biased) ...")
    sc = run_stereo_scene(surf, dem, px_to_m, args, rng_seed=args.seed)

    res = sc["results"]
    summary = {
        "scene": scene,
        "n_poses": args.n_poses,
        "vo_scale_bias": args.vo_scale_bias,
        "vo_scale_true": round(sc["s_true"], 4),
        "stereo_every": args.stereo_every,
        "stereo_sigma_frac": args.stereo_sigma_frac,
        "no_fix_gauge_s": round(res["gauge_no_stereo"]["s_final"], 4),
        "stereo_perfect_s": round(res["stereo_perfect"]["s_final"], 4),
        "stereo_perfect_rmse_m": res["stereo_perfect"]["rmse_m"],
        "gauge_rmse_m": res["gauge_no_stereo"]["rmse_m"],
        "stereo_baseline_err": args.stereo_baseline_err,
        "stereo_bias_plus_s": round(
            res[[k for k in res if k.startswith("stereo_bias_p")][0]]["s_final"], 4),
        "stereo_bias_minus_s": round(
            res[[k for k in res if k.startswith("stereo_bias_m")][0]]["s_final"], 4),
        "sweep_baseline_err": [round(float(e), 4) for e in sc["sweep_err"]],
        "sweep_recovered_s": [round(float(s), 4) for s in sc["sweep_s"]],
        "note": "no absolute fix anywhere; scale observability comes from the stereo baseline alone",
    }
    print(json.dumps(summary, indent=2))

    _render(args, dem, scene, sc)
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
