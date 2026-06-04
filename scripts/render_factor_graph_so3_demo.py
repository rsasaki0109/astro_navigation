#!/usr/bin/env python3
"""Animate Skyline Lock Phase 7(2): watch the SO(3)+scale factor graph think.

We freeze a mid-traverse star-tracker blackout and replay the Gauss-Newton
iterations. The optimizer starts from the causal forward-filter estimate -- a
trajectory that bulges away from truth across the blackout (dead-reckoned yaw
drifted several degrees AND the VO scale is wrong) and a scale guess of 1.0.
Iteration by iteration the Skyline locks that resume on the far side flow
backward through the VO yaw-increment chain, the metric scale slides toward
truth, and the arc snaps onto the ground track. The right panel shows the
attitude triptych: yaw collapsing inside the blackout band while roll and pitch
sit near zero the whole time -- held by gravity, never the hard part.

Reuses the solver and scene from factor_graph_so3_demo.py.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from skyline_lock_demo import build_dem  # noqa: E402
from factor_graph_so3_demo import (  # noqa: E402
    build_args, run_scene, Surface, attitude_errs,
)

FILTER = "#ff8c00"
JOINT = "#1f77ff"
PITCHC = "#2ca02c"


def main() -> int:
    ap = build_args()
    ap.add_argument("--duration-ms", type=int, default=220)
    ap.add_argument("--mp4", action="store_true")
    ap.set_defaults(output=REPO_ROOT / "docs" / "figures" / "skyline_lock"
                    / "factor_graph_so3_demo.gif")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dem, px_to_m = build_dem(args)
    surf = Surface(dem, px_to_m)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    bs, bl = args.blackout_start, args.blackout_len
    blackout = set(range(bs, bs + bl))
    print(f"scene={scene}  solving mid-traverse blackout for the iterate history ...")
    r = run_scene(surf, dem, px_to_m, args, blackout, rng_seed=args.seed)

    truth = r["truth"]; truth_R = None
    graph = r["graph"]; hist = r["hist"]; bo = r["blackout"]
    fixes = r["fixes"]
    s_true = r["metrics"]["vo_scale_true"]

    # rebuild truth attitude for per-iterate attitude error
    _, R_truth, _, _ = _truth_attitude(surf, extent_m, args)

    costs = [0.5 * float(graph.residual(e) @ graph.residual(e)) for e in hist]
    yaw_max = 0.0
    for e in hist:
        _, yw = attitude_errs(e["R"], R_truth)
        yaw_max = max(yaw_max, float(yw.max()))

    scales = [float(e["s"]) for e in hist]
    frames: list[Image.Image] = []
    n = len(hist)
    print(f"rendering {n} iterate frames ...")
    for i, e in enumerate(hist):
        pos = e["P"]
        tilt, yaw = attitude_errs(e["R"], R_truth)
        fig, axes = plt.subplots(1, 3, figsize=(18, 5.6),
                                 gridspec_kw={"width_ratios": [1.25, 1.0, 0.85]})
        fig.suptitle(
            f"SO(3)+scale factor graph -- Gauss-Newton iteration {i:2d}/{n - 1}   "
            f"(cost {costs[i]:.1f})    star-tracker blackout poses {bo[0]}-{bo[-1]}",
            fontsize=12.5)

        ax = axes[0]
        ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(truth[:, 0], truth[:, 1], "-", color="white", lw=2.4, label="ground truth")
        ax.plot(pos[:, 0], pos[:, 1], "-", color=JOINT, lw=2.0, label="current estimate")
        ax.plot(truth[bo, 0], truth[bo, 1], "o", color="k", ms=3.0, alpha=0.5,
                label="blackout poses")
        for (k, fxy, _) in fixes:
            ax.scatter(fxy[0], fxy[1], c="#19c819", marker="P", s=50,
                       edgecolors="k", lw=0.4, zorder=6)
        ax.scatter([], [], c="#19c819", marker="P", s=50, edgecolors="k", label="skyline lock")
        ax.set_title("trajectory: forward-filter bulge → snaps onto truth", fontsize=10.5)
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=8)

        ax = axes[1]
        ax.axvspan(bo[0], bo[-1], color="0.85", label="blackout")
        ax.plot(yaw, "-", color=JOINT, lw=2.1, label="heading / yaw (gravity-unobservable)")
        ax.plot(tilt, "-", color=PITCHC, lw=1.6, label="tilt / roll+pitch (gravity-held)")
        ax.set_ylim(-0.3, yaw_max * 1.05)
        ax.set_title("attitude error per pose — only heading collapses; tilt already flat",
                     fontsize=9.6)
        ax.set_xlabel("pose index"); ax.set_ylabel("|attitude error| (deg)")
        ax.legend(fontsize=8, loc="upper right")

        ax = axes[2]
        ax.axhline(s_true, color="k", ls="--", lw=1.4, label=f"true scale {s_true:.3f}")
        ax.axhline(1.0, color=FILTER, ls=":", lw=1.4, label="assumed scale 1.000")
        ax.plot(range(i + 1), scales[:i + 1], "-o", color=JOINT, lw=2.0, ms=3.5,
                label="estimated scale")
        ax.set_xlim(-0.5, n - 0.5)
        ax.set_ylim(min(s_true, min(scales)) - 0.015, 1.0 + 0.015)
        ax.set_title(f"metric scale state → {scales[i]:.3f}", fontsize=9.6)
        ax.set_xlabel("GN iteration"); ax.set_ylabel("global VO scale s")
        ax.legend(fontsize=8, loc="lower right")

        fig.tight_layout(rect=[0, 0, 1, 0.93])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)

    args.output.parent.mkdir(parents=True, exist_ok=True)
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
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            print(f"mp4 conversion skipped: {exc}")
    return 0


def _truth_attitude(surf, extent_m, args):
    """Rebuild the ground-truth poses to score per-iterate attitude error."""
    from factor_graph_so3_demo import make_truth_3d
    return make_truth_3d(surf, extent_m, args.n_poses, args.trajectory)


if __name__ == "__main__":
    sys.exit(main())
