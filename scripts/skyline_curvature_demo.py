#!/usr/bin/env python3
"""Skyline Lock, Phase 7: the lunar-curvature horizon model and when it matters.

Phases 0-6 rendered the predicted skyline with a flat-plane model: the ground at
range r sits on the same datum as the camera, so a feature's elevation angle is
just arctan((h - cam_z) / r). That ignores the Moon's curvature. The real datum
is a sphere of radius R = 1 737.4 km, and the surface drops below the observer's
tangent plane by ~r^2 / (2R) at range r. The Moon has no atmosphere, so -- unlike
a terrestrial viewshed -- there is no refraction term: this is the clean
geometric horizon. From a 2 m mast the bare horizon is only sqrt(2 R h) ~= 2.6 km
away, and a distant feature is visible only if its height clears r^2 / (2R)
(~0.5 km at 40 km, ~7.5 km at 160 km).

This demo asks the honest question: the rover physically observes the TRUE
(curved) horizon -- does it matter whether we localize it with the old flat model
or the correct curvature model?

    obs            = true curved horizon at the truth position (+ sensor noise)
    flat-model fix = match obs against flat-plane predictions   (Phases 0-6)
    curved-model fix = match obs against curvature-correct predictions (Phase 7)

Run over two contrasting LOLA scenes:
  * a distinctive crater (Tycho): the skyline is dominated by the near, kilometre-
    high rim, which clears the lunar horizon easily -- the flat model's bias is
    almost uniform across candidates and cancels in the (zero-mean, unit-norm)
    NCC, so both models lock the centre. Curvature is essentially free here.
  * a mare (Apollo 11): the lock leans on faint distant relief. The flat model
    counts terrain that is physically *below* the lunar horizon -- phantom cues --
    and snaps the position estimate ~16 km onto a false mode. The curvature model
    removes the phantoms and recovers the correct cell (the margin stays small:
    mare is genuinely aliased, but the point estimate is right).

The takeaway matches the project's honest-envelope stance: the correct physics is
nearly free where a near feature dominates, but it is the difference between a
right and a 16 km-wrong fix exactly where you are already marginal -- and it makes
the localizability picture more honest rather than rosier.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO / "scripts"))

from skyline_lock_demo import (  # noqa: E402
    load_lola_dem, render_horizon, best_yaw_ncc, uniqueness_margin,
)
from lro_trn_demo import LUNAR_RADIUS_M  # noqa: E402


def solve_scene(target, args):
    """Localize the true curved horizon with the flat vs curved model.

    Returns a dict with both fixes plus everything the figure needs.
    """
    dem, px_to_m = load_lola_dem(target, args.half_width_deg, args.ldem_ppd,
                                 args.cache_dir)
    size = dem.shape[0]
    extent = size * px_to_m
    A = args.n_az
    n_range = args.n_range
    r_min = max(args.r_min_m, 2.0 * px_to_m)
    r_max = 0.9 * extent
    bins_per_deg = A / 360.0

    truth = (0.5 * extent, 0.5 * extent)
    kw = dict(n_az=A, r_min_m=r_min, r_max_m=r_max, n_range=n_range)

    horizon_flat = render_horizon(dem, px_to_m, truth, args.mast_height_m,
                                  curvature_radius_m=None, **kw)
    horizon_curved = render_horizon(dem, px_to_m, truth, args.mast_height_m,
                                    curvature_radius_m=LUNAR_RADIUS_M, **kw)

    # The rover physically observes the curved horizon.
    heading = int(round(args.truth_yaw_deg * bins_per_deg)) % A
    obs = np.roll(horizon_curved, -heading)
    noise = math.radians(args.noise_arcmin / 60.0)
    obs = obs + np.random.default_rng(args.seed + 1).normal(0.0, noise, size=A)

    m = args.grid_margin_frac
    gx = np.linspace(m * extent, (1.0 - m) * extent, args.grid)
    gy = np.linspace(m * extent, (1.0 - m) * extent, args.grid)
    cand = [(x, y) for y in gy for x in gx]
    grid_step = float(gx[1] - gx[0])
    prior_lag = int(round(args.truth_yaw_deg * bins_per_deg)) % A
    window = max(1, int(math.ceil(3.0 * args.yaw_sigma_deg * bins_per_deg)))

    fixes = {}
    surfaces = {}
    for label, radius in (("flat", None), ("curved", LUNAR_RADIUS_M)):
        preds = np.stack([
            render_horizon(dem, px_to_m, xy, args.mast_height_m,
                           curvature_radius_m=radius, **kw)
            for xy in cand
        ])
        ncc, _ = best_yaw_ncc(obs, preds, prior_lag=prior_lag, window_bins=window)
        flat_idx = int(np.argmax(ncc))
        est = cand[flat_idx]
        err = math.hypot(est[0] - truth[0], est[1] - truth[1])
        margin, second = uniqueness_margin(ncc, cand, est, radius_m=2.0 * grid_step)
        fixes[label] = {
            "est_xy_m": [round(est[0], 1), round(est[1], 1)],
            "error_m": round(err, 1),
            "best_ncc": round(float(ncc[flat_idx]), 4),
            "uniqueness_margin": round(float(margin), 4),
        }
        surfaces[label] = ncc.reshape(args.grid, args.grid)

    mean_drop_deg = float(np.degrees(np.mean(horizon_flat - horizon_curved)))
    return {
        "target": target,
        "extent_m": round(extent, 1),
        "px_to_m": round(px_to_m, 2),
        "relief_m": round(float(dem.max() - dem.min()), 1),
        "grid_step_m": round(grid_step, 1),
        "mean_flat_minus_curved_elev_deg": round(mean_drop_deg, 3),
        "flat_model": fixes["flat"],
        "curved_model": fixes["curved"],
        "_dem": dem, "_px": px_to_m, "_extent": extent, "_truth": truth,
        "_gx": gx, "_gy": gy, "_az_deg": np.arange(A) / bins_per_deg,
        "_horizon_flat": horizon_flat, "_horizon_curved": horizon_curved,
        "_heading": heading, "_surf": surfaces, "_cand": cand,
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="tycho",
                    help="Distinctive-terrain LOLA target (near rim dominates).")
    ap.add_argument("--mare-target", default="apollo11",
                    help="Low-relief LOLA target (skyline leans on distant terrain).")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64])
    ap.add_argument("--half-width-deg", type=float, default=3.0)
    ap.add_argument("--cache-dir", type=Path,
                    default=_REPO / "datasets" / "lro_cache")
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--r-min-m", type=float, default=60.0)
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--grid", type=int, default=31)
    ap.add_argument("--grid-margin-frac", type=float, default=0.12)
    ap.add_argument("--noise-arcmin", type=float, default=2.0)
    ap.add_argument("--truth-yaw-deg", type=float, default=35.0)
    ap.add_argument("--yaw-sigma-deg", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--output", type=Path,
                    default=Path("outputs/skyline_lock/skyline_curvature.png"))
    ap.add_argument("--output-json", type=Path, default=None)
    args = ap.parse_args()

    distinctive = solve_scene(args.target, args)
    mare = solve_scene(args.mare_target, args)

    summary = {
        "title": "Skyline Lock Phase 7: lunar-curvature horizon model",
        "lunar_radius_m": LUNAR_RADIUS_M,
        "mast_height_m": args.mast_height_m,
        "bare_horizon_m": round(math.sqrt(2.0 * LUNAR_RADIUS_M * args.mast_height_m), 1),
        "scenes": {
            "distinctive": {k: v for k, v in distinctive.items() if not k.startswith("_")},
            "mare": {k: v for k, v in mare.items() if not k.startswith("_")},
        },
    }
    print(json.dumps(summary, indent=2))

    _render(args, distinctive, mare, summary)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")
    return 0


def _render(args, distinctive, mare, summary) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    R = LUNAR_RADIUS_M
    FLAT = "#ff8c00"
    CURVED = "#1f77ff"
    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    fig.suptitle(
        "Skyline Lock Phase 7 -- lunar-curvature horizon model  "
        f"(R = {R/1000:.1f} km, 2 m mast -> bare horizon {summary['bare_horizon_m']/1000:.2f} km)",
        fontsize=14)

    for row, sc in enumerate((distinctive, mare)):
        kind = "distinctive crater" if row == 0 else "low-relief mare"
        # ---- col 0: horizon profile, flat vs curved ----
        ax = axes[row, 0]
        az = sc["_az_deg"]
        hf = np.degrees(np.roll(sc["_horizon_flat"], -sc["_heading"]))
        hc = np.degrees(np.roll(sc["_horizon_curved"], -sc["_heading"]))
        ax.plot(az, hf, color=FLAT, lw=1.4, label="flat-plane model")
        ax.plot(az, hc, color=CURVED, lw=1.4, label="curvature-correct")
        ax.fill_between(az, hc, hf, where=(hf >= hc), color=FLAT, alpha=0.18,
                        label="phantom terrain (below lunar horizon)")
        ax.set_title(f"{sc['target']} ({kind}) -- horizon profile\n"
                     f"flat over-reads {sc['mean_flat_minus_curved_elev_deg']:.2f} deg on average")
        ax.set_xlabel("camera azimuth (deg)")
        ax.set_ylabel("elevation (deg)")
        ax.legend(fontsize=7, loc="upper right")

        # ---- col 1: curved score surface + both estimates ----
        ax = axes[row, 1]
        gx, gy = sc["_gx"], sc["_gy"]
        im = ax.imshow(sc["_surf"]["curved"], origin="lower",
                       extent=[gx[0], gx[-1], gy[0], gy[-1]], cmap="viridis",
                       aspect="auto")
        tx, ty = sc["_truth"]
        ax.scatter(tx, ty, c="white", marker="*", s=240, edgecolors="k",
                   zorder=6, label="truth")
        fe = sc["flat_model"]["est_xy_m"]
        ce = sc["curved_model"]["est_xy_m"]
        ax.scatter(fe[0], fe[1], c=FLAT, marker="x", s=130, zorder=6,
                   label=f"flat fix ({sc['flat_model']['error_m']/1000:.1f} km)")
        ax.scatter(ce[0], ce[1], facecolors="none", edgecolors=CURVED, marker="o",
                   s=170, linewidths=2.2, zorder=6,
                   label=f"curved fix ({sc['curved_model']['error_m']/1000:.1f} km)")
        ax.set_title(f"{sc['target']} -- curved-model score surface")
        ax.set_xlabel("x east (m)")
        ax.set_ylabel("y north (m)")
        ax.legend(fontsize=7, loc="upper right")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # ---- col 2 row 0: lunar horizon geometry ----
    ax = axes[0, 2]
    r = np.linspace(0.0, 160e3, 400)
    drop = r ** 2 / (2.0 * R)
    ax.plot(r / 1000, drop / 1000, color="k", lw=1.6)
    ax.fill_between(r / 1000, 0, drop / 1000, color="0.85",
                    label="hidden below horizon")
    bare = math.sqrt(2.0 * R * args.mast_height_m)
    ax.axvline(bare / 1000, color=CURVED, ls="--", lw=1.0)
    ax.annotate(f"bare horizon\n{bare/1000:.1f} km (2 m mast)",
                xy=(bare / 1000, 0), xytext=(12, 0.9),
                fontsize=8, color=CURVED)
    for d_km, txt in ((40, "rim @40 km\nneeds >0.5 km"),
                      (90, "@90 km\n>2.3 km"),
                      (160, "@160 km\n>7.4 km")):
        ax.scatter(d_km, (d_km * 1000) ** 2 / (2 * R) / 1000, color=FLAT, zorder=5)
        ax.annotate(txt, xy=(d_km, (d_km * 1000) ** 2 / (2 * R) / 1000),
                    xytext=(d_km - 38, (d_km * 1000) ** 2 / (2 * R) / 1000 + 0.4),
                    fontsize=7, color="0.25")
    ax.set_title("On the Moon the horizon is close\nfeature at range r visible only above r^2/2R")
    ax.set_xlabel("ground range r (km)")
    ax.set_ylabel("visibility height threshold (km)")
    ax.legend(fontsize=7, loc="upper left")

    # ---- col 2 row 1: summary bars (error flat vs curved) ----
    ax = axes[1, 2]
    labels = [distinctive["target"], mare["target"]]
    flat_err = [distinctive["flat_model"]["error_m"] / 1000,
                mare["flat_model"]["error_m"] / 1000]
    curved_err = [distinctive["curved_model"]["error_m"] / 1000,
                  mare["curved_model"]["error_m"] / 1000]
    x = np.arange(len(labels))
    w = 0.36
    ax.bar(x - w / 2, flat_err, w, color=FLAT, label="flat-model fix")
    ax.bar(x + w / 2, curved_err, w, color=CURVED, label="curved-model fix")
    for xi, (fv, cv) in enumerate(zip(flat_err, curved_err)):
        ax.text(xi - w / 2, fv + 0.2, f"{fv:.1f}", ha="center", fontsize=8, color=FLAT)
        ax.text(xi + w / 2, cv + 0.2, f"{cv:.1f}", ha="center", fontsize=8, color=CURVED)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{l}\n(near rim)" if i == 0 else f"{l}\n(distant terrain)"
                        for i, l in enumerate(labels)])
    ax.set_ylabel("position error (km)")
    ax.set_title("Wrong model is free over a near rim,\n16 km-wrong over mare")
    ax.legend(fontsize=8, loc="upper left")

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    sys.exit(main())
