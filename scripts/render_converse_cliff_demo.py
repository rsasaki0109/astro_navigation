#!/usr/bin/env python3
"""Animate the converse cliff: two real terrains, one four-factor stack.

Top row Tycho highland, bottom row Apollo 11 mare. Each row replays the same
radial traverse as a causal fixed-lag smoother (re-solve the position graph over
poses seen so far), with the appearance map, the growing tracks, the fixes
appearing coloured by uniqueness, and the running skyline-vs-TRN margin trace.

Watch the asymmetry animate: over Tycho the Skyline margin spikes while the rover
crosses the distinctive interior (green locks), then collapses out on the
symmetric exterior; over the mare it never lifts off the floor. TRN's margin sits
high on both, so the fused+TRN track hugs truth in both rows while skyline-only
peels away with VO on the mare from the very start.

Reuses scripts/converse_cliff_demo.py (scene solve) and the four_factor/
factor_graph_fusion building blocks it imports.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from factor_graph_fusion_demo import fuse_positions  # noqa: E402
from factor_graph_fusion_demo import simulate_vo, true_yaw_series, make_truth_trajectory, integrate  # noqa: E402
from converse_cliff_demo import (  # noqa: E402
    scene_args, solve_scene, SKY_U, SKY_A, TRN_U, TRN_A,
)


def _causal_tracks(r, n_poses, sigma_prior_m, sigma_vo_m):
    """Recover the per-pose VO deltas used in the scene and replay a causal
    fixed-lag solve so the animation reveals the fused tracks honestly (each
    frame only sees fixes up to that pose), not a pre-baked batch answer."""
    # The scene stored truth/vo_only; reconstruct vo_deltas as consecutive diffs
    # of the integrated VO-only track (integrate is a cumulative sum of deltas).
    vo_deltas = np.diff(r["vo_only"], axis=0)
    sky_by_pose = {k: (fxy, sig, rec["unique"])
                   for (k, fxy, sig), rec in zip(r["sky_fixes"], r["sky_rec"])}
    trn_by_pose = {k: (fxy, sig, rec["unique"])
                   for (k, fxy, sig), rec in zip(r["trn_fixes"], r["trn_rec"])}
    return vo_deltas, sky_by_pose, trn_by_pose


def _draw_row(axes, r, label, k, state, y_top):
    import math as _m
    from matplotlib.patches import Ellipse
    extent_m = r["extent_m"]
    truth = r["truth"]
    vo_deltas, sky_by_pose, trn_by_pose = state

    sky_fixes = [(p, v[0], v[1]) for p, v in sky_by_pose.items() if p <= k]
    trn_fixes = [(p, v[0], v[1]) for p, v in trn_by_pose.items() if p <= k]
    est3, _ = fuse_positions(k + 1, vo_deltas[:k], truth[0], 20.0, 120.0, sky_fixes)
    est4, cov4 = fuse_positions(k + 1, vo_deltas[:k], truth[0], 20.0, 120.0, sky_fixes + trn_fixes)

    sky_locked = None
    for p in range(k, -1, -1):
        if p in sky_by_pose:
            sky_locked = sky_by_pose[p][2]
            break
    verdict = "horizon locked" if sky_locked else "horizon aliased -> TRN carries"

    ax = axes[0]
    ax.imshow(r["app"], origin="lower", extent=[0, extent_m, 0, extent_m], cmap="gray")
    ax.plot(truth[:k + 1, 0], truth[:k + 1, 1], "-", color="white", lw=2.0, label="ground truth")
    ax.plot(r["vo_only"][:k + 1, 0], r["vo_only"][:k + 1, 1], "--", color="red", lw=1.4, label="VO only")
    ax.plot(est3[:, 0], est3[:, 1], "-", color="#ffd24d", lw=1.5, label="fused (no TRN)")
    ax.plot(est4[:, 0], est4[:, 1], "-", color="cyan", lw=1.9, label="fused + TRN")
    sx = 2.0 * _m.sqrt(max(cov4[k, 0, 0], 0.0)); sy = 2.0 * _m.sqrt(max(cov4[k, 1, 1], 0.0))
    ax.add_patch(Ellipse((est4[k, 0], est4[k, 1]), 2 * sx, 2 * sy, fill=False, color="cyan", lw=1.4))
    for p, v in sky_by_pose.items():
        if p <= k:
            ax.scatter(*v[0], c=(SKY_U if v[2] else SKY_A), marker="P", s=56, edgecolors="k", lw=0.5, zorder=6)
    for p, v in trn_by_pose.items():
        if p <= k:
            ax.scatter(*v[0], c=(TRN_U if v[2] else TRN_A), marker="o", s=20, edgecolors="k", lw=0.4, zorder=5)
    ax.scatter(*truth[k], c="white", marker="*", s=120, edgecolors="k", zorder=7)
    ax.set_xlim(0, extent_m); ax.set_ylim(0, extent_m)
    ax.set_title(f"{label}: {verdict}", fontsize=10.5,
                 color=("#137333" if sky_locked else "#1f77ff"))
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    ax.legend(loc="upper left", fontsize=6.6)

    ax = axes[1]
    full_vo = r["vo_err"]
    ax.plot(np.arange(len(full_vo)), full_vo, "--", color="red", lw=0.7, alpha=0.3)
    ax.plot(np.arange(k + 1), full_vo[:k + 1], "--", color="red", lw=1.8, label="VO only")
    e3 = np.linalg.norm(est3 - truth[:k + 1], axis=1)
    e4 = np.linalg.norm(est4 - truth[:k + 1], axis=1)
    ax.plot(np.arange(k + 1), e3, "-", color="#e0a800", lw=1.6, label="fused (no TRN)")
    ax.plot(np.arange(k + 1), e4, "-", color="cyan", lw=2.1, label="fused + TRN")
    ax.set_xlim(0, len(full_vo) - 1); ax.set_yscale("log"); ax.set_ylim(8.0, y_top)
    ax.set_title(f"{label}: position error vs pose (log)", fontsize=10.0)
    ax.set_xlabel("pose index"); ax.set_ylabel("error (m)")
    ax.legend(fontsize=7.5, loc="upper left")

    ax = axes[2]
    sk = [(rec["pose"], rec["margin"], rec["unique"]) for rec in r["sky_rec"] if rec["pose"] <= k]
    tr = [(rec["pose"], rec["margin"]) for rec in r["trn_rec"] if rec["pose"] <= k]
    if sk:
        ax.plot([s[0] for s in sk], [s[1] for s in sk], "-P", color="#137333", lw=1.4, ms=6,
                label="Skyline margin (relief)")
    if tr:
        ax.plot([t[0] for t in tr], [t[1] for t in tr], "-o", color="#1f77ff", lw=1.4, ms=4,
                label="TRN margin (texture)")
    ax.axhline(0.05, color="#137333", ls=":", lw=1.0, alpha=0.7)
    ax.axhline(0.1, color="#1f77ff", ls=":", lw=1.0, alpha=0.7)
    ax.set_xlim(0, len(r["truth"]) - 1); ax.set_ylim(0.0, 0.8)
    ax.set_title(f"{label}: uniqueness margins", fontsize=10.0)
    ax.set_xlabel("pose index"); ax.set_ylabel("margin")
    ax.legend(fontsize=7.5, loc="upper right")


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
    ap.add_argument("--frame-every", type=int, default=2, help="Render every Nth pose (smaller GIF).")
    ap.add_argument("--duration-ms", type=int, default=160)
    ap.add_argument("--mp4", action="store_true")
    ap.add_argument("--output", type=Path,
                    default=REPO_ROOT / "docs" / "figures" / "skyline_lock" / "converse_cliff_demo.gif")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    print(f"solving highland scene ({args.highland_target}) ...")
    highland = solve_scene(scene_args(args.highland_target, args))
    print(f"solving mare scene ({args.mare_target}) ...")
    mare = solve_scene(scene_args(args.mare_target, args))

    h_state = _causal_tracks(highland, args.n_poses, 20.0, 120.0)
    m_state = _causal_tracks(mare, args.n_poses, 20.0, 120.0)
    h_top = max(50.0, float(highland["vo_err"].max()) * 1.3)
    m_top = max(50.0, float(mare["vo_err"].max()) * 1.3)

    frames: list[Image.Image] = []
    ks = list(range(1, args.n_poses, args.frame_every)) + [args.n_poses - 1]
    print(f"rendering {len(ks)} frames ...")
    for k in ks:
        fig, axes = plt.subplots(2, 3, figsize=(17, 9.4))
        fig.suptitle(
            f"Converse cliff -- one four-factor stack on two real terrains   pose {k+1}/{args.n_poses}\n"
            "the horizon needs relief (Tycho has it, mare does not); the ground texture carries both",
            fontsize=12.5)
        _draw_row(axes[0], highland, "Tycho highland", k, h_state, h_top)
        _draw_row(axes[1], mare, "Apollo 11 mare", k, m_state, m_top)
        fig.tight_layout(rect=[0, 0, 1, 0.94])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:] + [frames[-1]] * 8,
                   duration=args.duration_ms, loop=0, optimize=True)
    print(f"wrote {args.output}  ({len(frames)} frames)")

    if args.mp4:
        mp4 = args.output.with_suffix(".mp4")
        cmd = ["ffmpeg", "-y", "-i", str(args.output), "-movflags", "faststart",
               "-pix_fmt", "yuv420p", "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", str(mp4)]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            print(f"wrote {mp4}")
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            print(f"mp4 conversion skipped: {exc}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
