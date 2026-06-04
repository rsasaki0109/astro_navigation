#!/usr/bin/env python3
"""Converse cliff: the same four-factor stack on two real terrains.

Phase 6 added star + VO + Skyline + TRN in one pose graph and sold it as
"complementary failure modes": the romantic claim that wherever one absolute fix
cliffs, the other holds. This demo runs that SAME stack, unchanged, on two real
LRO scenes and shows the honest, data-grounded version of that story -- which is
*asymmetric*, not the tidy mirror image Phase 6's prose implied:

  - Tycho (highland crater): a kilometre-high distinctive rim gives the horizon
    plenty of relief. Skyline LOCKS across the distinctive interior, then aliases
    out on the rotationally symmetric exterior. TRN locks the whole way on the
    texture-rich ejecta.

  - Apollo 11 (mare): the basalt plain is flat, so the 360-deg horizon is nearly
    featureless and azimuth-symmetric -- Skyline has nothing to lock onto and
    aliases almost everywhere (2/15 unique here vs 7/15 over Tycho). But the mare
    is NOT textureless from above: albedo speckle, small craters and rays give
    the nadir TRN matcher plenty to grip, so TRN still locks 20/20 and carries
    the fix.

So the honest lesson is not "two sensors, two complementary cliffs." It is:
**Skyline is a relief-dependent cue and TRN is a texture-dependent cue, and the
Moon feeds them unequally per terrain.** What makes the fused stack survive both
is not redundancy -- it is that each absolute factor is weighted by its OWN real
uniqueness margin, so the cue the terrain starves is discounted automatically,
with no terrain classifier in the loop. Give the horizon relief and it leads;
take the relief away and the ground texture quietly takes over.

This also corrects an overstated note carried since Phase 6 (four_factor_fusion_
demo.py) that claimed TRN is feature-starved on mare while the horizon pins -- the
real data shows the opposite direction on Apollo 11 at this scale, and that note
has been fixed to match.

Reuses scripts/four_factor_fusion_demo.py (appearance map, TRN matcher) and
scripts/factor_graph_fusion_demo.py (scenario, Skyline matcher, linear solver),
so nothing about the estimator changes between the two scenes -- only the terrain.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from skyline_lock_demo import build_dem  # noqa: E402
from factor_graph_fusion_demo import (  # noqa: E402
    make_truth_trajectory, true_yaw_series, simulate_vo, integrate,
    SkylineLocalizer, margin_to_sigma, fuse_positions,
)
from four_factor_fusion_demo import build_appearance_map, TrnLocalizer  # noqa: E402


# --------------------------------------------------------------------------- #
# One scene's worth of the four-factor solve (the four_factor pipeline,        #
# parameterized by target so we can run it twice and contrast).                #
# --------------------------------------------------------------------------- #
def scene_args(target: str, base) -> SimpleNamespace:
    """Build the four_factor arg bundle for one target, sharing every estimator
    knob with the other scene so the only thing that differs is the terrain."""
    return SimpleNamespace(
        source="lola", terrain="craters", target=target,
        ldem_ppd=base.ldem_ppd, half_width_deg=base.half_width_deg,
        cache_dir=base.cache_dir, size_px=base.size_px, px_to_m=base.px_to_m,
        seed=base.seed, trajectory=base.trajectory, n_poses=base.n_poses,
        skyline_every=base.skyline_every, trn_every=base.trn_every,
        mast_height_m=2.0, n_az=180, n_range=160, grid=base.grid,
        grid_margin_frac=0.12, noise_arcmin=2.0, yaw_sigma_deg=2.0,
        trn_app_px=480, trn_upsample=4, trn_wac_zoom=base.trn_wac_zoom,
        trn_wac_tile_radius=base.trn_wac_tile_radius, trn_patch_frac=0.09,
        trn_noise_frac=0.05, vo_scale_bias=0.04, vo_yaw_bias_deg=0.25,
        vo_sigma_frac=0.06, sigma_prior_m=20.0, sigma_vo_m=120.0,
        sigma_lock_m=400.0, margin_ref=0.15, margin_floor=0.01,
        sigma_cap_m=40000.0, trn_sigma_lock_m=150.0, trn_margin_ref=0.4,
        trn_margin_floor=0.02, trn_unique_margin=0.1,
    )


def solve_scene(a: SimpleNamespace) -> dict:
    dem, px_to_m = build_dem(a)
    extent_m = dem.shape[0] * px_to_m
    rng = np.random.default_rng(a.seed)
    truth = make_truth_trajectory(extent_m, a.n_poses, a.trajectory)
    truth_yaw = true_yaw_series(truth)
    star_yaw = truth_yaw + rng.normal(0.0, math.radians(a.yaw_sigma_deg), size=truth_yaw.shape)
    vo_deltas = simulate_vo(truth, star_yaw, rng, scale_bias=a.vo_scale_bias,
                            yaw_bias_rad=math.radians(a.vo_yaw_bias_deg),
                            sigma_frac=a.vo_sigma_frac)
    vo_only = integrate(truth[0], vo_deltas)

    app, app_px_to_m, app_kind = build_appearance_map(a, dem, px_to_m)
    sky = SkylineLocalizer(dem, px_to_m, grid=a.grid, margin_frac=a.grid_margin_frac,
                           n_az=a.n_az, n_range=a.n_range,
                           mast_height_m=a.mast_height_m, yaw_sigma_deg=a.yaw_sigma_deg)
    trn = TrnLocalizer(app, app_px_to_m, patch_m=a.trn_patch_frac * extent_m)

    sky_fixes, sky_rec = [], []
    for k in range(0, a.n_poses, a.skyline_every):
        est_xy, margin, _ = sky.fix(tuple(truth[k]), truth_yaw[k], star_yaw[k], rng, a.noise_arcmin)
        sigma = margin_to_sigma(margin, sigma_lock_m=a.sigma_lock_m, margin_ref=a.margin_ref,
                                margin_floor=a.margin_floor, sigma_cap_m=a.sigma_cap_m)
        sky_fixes.append((k, est_xy, sigma))
        sky_rec.append({"pose": k, "margin": margin, "sigma": sigma,
                        "unique": bool(margin >= 0.05),
                        "err": float(math.hypot(*(est_xy - truth[k])))})
    trn_fixes, trn_rec = [], []
    for k in range(0, a.n_poses, a.trn_every):
        est_xy, margin, _ = trn.fix(tuple(truth[k]), rng, a.trn_noise_frac)
        sigma = margin_to_sigma(margin, sigma_lock_m=a.trn_sigma_lock_m, margin_ref=a.trn_margin_ref,
                                margin_floor=a.trn_margin_floor, sigma_cap_m=a.sigma_cap_m)
        trn_fixes.append((k, est_xy, sigma))
        trn_rec.append({"pose": k, "margin": margin, "sigma": sigma,
                        "unique": bool(margin >= a.trn_unique_margin),
                        "err": float(math.hypot(*(est_xy - truth[k])))})

    est3, _ = fuse_positions(a.n_poses, vo_deltas, truth[0], a.sigma_prior_m, a.sigma_vo_m, sky_fixes)
    est4, cov4 = fuse_positions(a.n_poses, vo_deltas, truth[0], a.sigma_prior_m, a.sigma_vo_m,
                                sky_fixes + trn_fixes)

    def rmse(p):
        return float(np.sqrt(np.mean(np.sum((p - truth) ** 2, axis=1))))
    return {
        "target": a.target, "dem": dem, "app": app, "extent_m": extent_m, "app_kind": app_kind,
        "truth": truth, "vo_only": vo_only, "est3": est3, "est4": est4, "cov4": cov4,
        "sky_fixes": sky_fixes, "sky_rec": sky_rec, "trn_fixes": trn_fixes, "trn_rec": trn_rec,
        "vo_err": np.linalg.norm(vo_only - truth, axis=1),
        "err3": np.linalg.norm(est3 - truth, axis=1),
        "err4": np.linalg.norm(est4 - truth, axis=1),
        "metrics": {
            "vo_only_rmse_m": round(rmse(vo_only), 1),
            "fused3_rmse_m": round(rmse(est3), 1),
            "fused4_rmse_m": round(rmse(est4), 1),
            "n_skyline_unique": sum(r["unique"] for r in sky_rec),
            "n_skyline_fixes": len(sky_rec),
            "n_trn_unique": sum(r["unique"] for r in trn_rec),
            "n_trn_fixes": len(trn_rec),
            "skyline_margin_median": round(float(np.median([r["margin"] for r in sky_rec])), 4),
            "trn_margin_median": round(float(np.median([r["margin"] for r in trn_rec])), 4),
        },
    }


# --------------------------------------------------------------------------- #
# Render: two rows (highland vs mare), three columns                          #
# --------------------------------------------------------------------------- #
SKY_U, SKY_A = "#19c819", "#ff8c00"
TRN_U, TRN_A = "#1f77ff", "#9aa0a6"


def _render_row(axes, r, label, trn_unique_margin):
    import math as _m
    from matplotlib.patches import Ellipse
    extent_m = r["extent_m"]

    ax = axes[0]
    ax.imshow(r["app"], origin="lower", extent=[0, extent_m, 0, extent_m], cmap="gray")
    ax.plot(r["truth"][:, 0], r["truth"][:, 1], "-", color="white", lw=2.0, label="ground truth")
    ax.plot(r["vo_only"][:, 0], r["vo_only"][:, 1], "--", color="red", lw=1.4, label="VO only")
    ax.plot(r["est3"][:, 0], r["est3"][:, 1], "-", color="#ffd24d", lw=1.5, label="fused (no TRN)")
    ax.plot(r["est4"][:, 0], r["est4"][:, 1], "-", color="cyan", lw=1.9, label="fused + TRN")
    for k in range(0, len(r["est4"]), max(1, len(r["est4"]) // 12)):
        sx = 2.0 * _m.sqrt(max(r["cov4"][k, 0, 0], 0.0))
        sy = 2.0 * _m.sqrt(max(r["cov4"][k, 1, 1], 0.0))
        ax.add_patch(Ellipse((r["est4"][k, 0], r["est4"][k, 1]), 2 * sx, 2 * sy,
                             fill=False, color="cyan", lw=0.6, alpha=0.55))
    for (k, fxy, _), rec in zip(r["sky_fixes"], r["sky_rec"]):
        ax.scatter(*fxy, c=(SKY_U if rec["unique"] else SKY_A), marker="P", s=58,
                   edgecolors="k", lw=0.5, zorder=6)
    for (k, fxy, _), rec in zip(r["trn_fixes"], r["trn_rec"]):
        ax.scatter(*fxy, c=(TRN_U if rec["unique"] else TRN_A), marker="o", s=22,
                   edgecolors="k", lw=0.4, zorder=5)
    ax.set_xlim(0, extent_m); ax.set_ylim(0, extent_m)
    nsu, nsf = r["metrics"]["n_skyline_unique"], r["metrics"]["n_skyline_fixes"]
    ax.set_title(f"{label}: appearance + tracks\nskyline locks {nsu}/{nsf}, TRN locks "
                 f"{r['metrics']['n_trn_unique']}/{r['metrics']['n_trn_fixes']}", fontsize=9.5)
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    ax.legend(loc="upper left", fontsize=6.6)

    ax = axes[1]
    ax.plot(r["vo_err"], "--", color="red", lw=1.6, label="VO only")
    ax.plot(r["err3"], "-", color="#e0a800", lw=1.6, label="fused (no TRN)")
    ax.plot(r["err4"], "-", color="cyan", lw=2.1, label="fused + TRN")
    for (k, _, _), rec in zip(r["sky_fixes"], r["sky_rec"]):
        ax.axvline(k, color=(SKY_U if rec["unique"] else SKY_A), lw=0.8, alpha=0.3)
    ax.set_yscale("log")
    ax.set_title(f"{label}: position error vs pose\nfused+TRN RMSE "
                 f"{r['metrics']['fused4_rmse_m']:.0f} m  vs skyline-only "
                 f"{r['metrics']['fused3_rmse_m']:.0f} m", fontsize=9.5)
    ax.set_xlabel("pose index"); ax.set_ylabel("error (m)")
    ax.legend(fontsize=7.5)

    ax = axes[2]
    sk_k = [rec["pose"] for rec in r["sky_rec"]]; sk_m = [rec["margin"] for rec in r["sky_rec"]]
    tr_k = [rec["pose"] for rec in r["trn_rec"]]; tr_m = [rec["margin"] for rec in r["trn_rec"]]
    ax.plot(sk_k, sk_m, "-P", color="#137333", lw=1.4, ms=6, label="Skyline margin (horizon relief)")
    ax.plot(tr_k, tr_m, "-o", color="#1f77ff", lw=1.4, ms=4, label="TRN margin (ground texture)")
    ax.axhline(0.05, color="#137333", ls=":", lw=1.0, alpha=0.7)
    ax.axhline(trn_unique_margin, color="#1f77ff", ls=":", lw=1.0, alpha=0.7)
    ax.set_ylim(bottom=0.0)
    ax.set_title(f"{label}: uniqueness margins\n(dotted = lock threshold)", fontsize=9.5)
    ax.set_xlabel("pose index"); ax.set_ylabel("uniqueness margin")
    ax.legend(fontsize=7.5, loc="upper right")


def render(highland, mare, out_png, trn_unique_margin):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(
        "Converse cliff: one four-factor stack, two real terrains  -  "
        "the horizon needs relief, the ground needs texture, the Moon feeds them unequally",
        fontsize=13.5)
    _render_row(axes[0], highland, f"Tycho highland ({highland['app_kind']})", trn_unique_margin)
    _render_row(axes[1], mare, f"Apollo 11 mare ({mare['app_kind']})", trn_unique_margin)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120)
    print(f"wrote {out_png}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--highland-target", default="tycho")
    ap.add_argument("--mare-target", default="apollo11")
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
    ap.add_argument("--grid", type=int, default=41)
    ap.add_argument("--trn-wac-zoom", type=int, default=6)
    ap.add_argument("--trn-wac-tile-radius", type=int, default=1)
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "converse_cliff.png")
    ap.add_argument("--output-json", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "converse_cliff.json")
    args = ap.parse_args()

    print(f"solving highland scene ({args.highland_target}) ...")
    highland = solve_scene(scene_args(args.highland_target, args))
    print(f"solving mare scene ({args.mare_target}) ...")
    mare = solve_scene(scene_args(args.mare_target, args))

    summary = {
        "highland": {"target": args.highland_target, **highland["metrics"]},
        "mare": {"target": args.mare_target, **mare["metrics"]},
    }
    print(json.dumps(summary, indent=2))

    render(highland, mare, args.output, 0.1)
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
