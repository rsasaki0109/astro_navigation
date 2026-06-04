#!/usr/bin/env python3
"""Animate Skyline Lock Phase 7(1): watch the SO(2) factor graph think.

We freeze a mid-traverse star-tracker blackout and replay the Gauss-Newton
iterations. The optimizer starts from the Phase-5 fixed-yaw estimate -- a
trajectory that bulges away from truth across the blackout because the dead-
reckoned heading drifted several degrees. Iteration by iteration the resumed
star fixes (and skyline locks) on the far side flow backward through the VO
yaw-increment chain, the per-pose heading corrects, and the dead-reckoned arc
snaps onto the ground truth. The right panel shows |yaw error| collapsing inside
the blackout band as it happens.

Reuses the solver and scene from factor_graph_so2_demo.py.
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

from skyline_lock_demo import build_dem  # noqa: E402
from factor_graph_so2_demo import (  # noqa: E402
    build_args, run_scene, split, wrap,
)

FIXED = "#ff8c00"
JOINT = "#1f77ff"


def main() -> int:
    # Inherit the demo's scene/solver args, then add animation knobs and
    # repoint --output at the gif.
    ap = build_args()
    ap.add_argument("--duration-ms", type=int, default=200)
    ap.add_argument("--mp4", action="store_true")
    ap.set_defaults(output=REPO_ROOT / "docs" / "figures" / "skyline_lock"
                    / "factor_graph_so2_demo.gif")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dem, px_to_m = build_dem(args)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    bs, bl = args.blackout_start, args.blackout_len
    blackout = set(range(bs, bs + bl))
    print(f"scene={scene}  solving mid-traverse blackout for the iterate history ...")
    r = run_scene(dem, px_to_m, args, blackout, rng_seed=args.seed)

    truth = r["truth"]; truth_yaw = r["truth_yaw"]
    graph = r["graph"]; hist = r["hist"]; bo = r["blackout"]
    fixes = r["fixes"]
    costs = [0.5 * float(graph.residual_jac(z)[0] @ graph.residual_jac(z)[0]) for z in hist]
    yaw_max = max(np.degrees(np.abs(wrap(split(z)[1] - truth_yaw))).max() for z in hist)

    frames: list[Image.Image] = []
    n = len(hist)
    print(f"rendering {n} iterate frames ...")
    for i, z in enumerate(hist):
        pos, yaw = split(z)
        yaw_err = np.degrees(np.abs(wrap(yaw - truth_yaw)))
        fig, axes = plt.subplots(1, 2, figsize=(13, 5.8))
        fig.suptitle(
            f"SO(2) factor graph -- Gauss-Newton iteration {i:2d}/{n - 1}   "
            f"(cost {costs[i]:.1f})    star-tracker blackout poses {bo[0]}-{bo[-1]}",
            fontsize=12.5)

        ax = axes[0]
        ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(truth[:, 0], truth[:, 1], "-", color="white", lw=2.4, label="ground truth")
        ax.plot(pos[:, 0], pos[:, 1], "-", color=JOINT, lw=2.0,
                label="current estimate")
        ax.plot(truth[bo, 0], truth[bo, 1], "o", color="k", ms=3.0, alpha=0.5,
                label="blackout poses")
        for (k, fxy, _) in fixes:
            ax.scatter(*fxy, c="#19c819", marker="P", s=50, edgecolors="k", lw=0.4, zorder=6)
        ax.scatter([], [], c="#19c819", marker="P", s=50, edgecolors="k", label="skyline lock")
        ax.set_title("trajectory: fixed-yaw bulge → snaps onto truth", fontsize=10.5)
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=8)

        ax = axes[1]
        ax.axvspan(bo[0], bo[-1], color="0.85", label="blackout")
        ax.plot(yaw_err, "-", color=JOINT, lw=2.0)
        ax.scatter(np.arange(len(yaw_err)), yaw_err, c=JOINT, s=10, zorder=5)
        ax.set_ylim(-0.4, yaw_max * 1.05)
        ax.set_title("heading error per pose — collapses inside the blackout",
                     fontsize=10.5)
        ax.set_xlabel("pose index"); ax.set_ylabel("|yaw error| (deg)")
        ax.legend(fontsize=8, loc="upper right")

        fig.tight_layout(rect=[0, 0, 1, 0.93])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Hold the first (the problem) and last (the fix) so the eye catches both.
    seq = [frames[0]] * 6 + frames + [frames[-1]] * 12
    seq[0].save(args.output, save_all=True, append_images=seq[1:],
                duration=args.duration_ms, loop=0, optimize=True)
    print(f"wrote {args.output}  ({len(seq)} frames)")

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
