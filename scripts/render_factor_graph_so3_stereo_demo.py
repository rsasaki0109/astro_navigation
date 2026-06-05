#!/usr/bin/env python3
"""Animate Phase 7(3): watch a stereo baseline pull the scale gauge into place.

No absolute fix anywhere on the traverse. The left panel replays the Gauss-Newton
iterations of the stereo-augmented SO(3) x scale solve: the dead-reckoned map
starts overshooting truth (scale assumed 1.000, ~8% long) and shrinks onto the
ground track as the stereo metric-translation factor pulls the global scale down
toward truth. The right panel tracks the scale state per iteration for three
conditions at once -- no stereo (a flat gauge freedom that never moves), a true
baseline (converges onto truth), and a mis-calibrated baseline (converges, but to
a biased value). The map only becomes metric because the baseline is trusted.

Reuses scripts/factor_graph_so3_stereo_demo.py for the sensors, scene and solver.
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
    Surface, make_truth_3d, simulate_sensors, forward_filter,
)
from factor_graph_so3_stereo_demo import (  # noqa: E402
    simulate_stereo, _build_graph, add_stereo_args, build_args,
    GAUGE, PERFECT, BIASP, BIASM, TRUTHC,
)


def main() -> int:
    ap = build_args()
    add_stereo_args(ap)
    ap.add_argument("--duration-ms", type=int, default=220)
    ap.add_argument("--mp4", action="store_true")
    ap.set_defaults(output=REPO_ROOT / "docs" / "figures" / "skyline_lock"
                    / "factor_graph_so3_stereo_demo.gif")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dem, px_to_m = build_dem(args)
    surf = Surface(dem, px_to_m)
    extent_m = dem.shape[0] * px_to_m
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    print(f"scene={scene}  solving stereo conditions for the iterate history ...")

    rng = np.random.default_rng(args.seed)
    P, R, xy, yaw = make_truth_3d(surf, extent_m, args.n_poses, args.trajectory)
    meas = simulate_sensors(
        P, R, args.n_poses, set(), rng, scale_bias=args.vo_scale_bias,
        gyro_bias_deg=[args.gyro_pitch_bias_deg, args.gyro_roll_bias_deg, args.gyro_yaw_bias_deg],
        grav_sigma_deg=args.grav_sigma_deg, star_sigma_deg=args.yaw_sigma_deg,
        vo_sigma_frac=args.vo_sigma_frac)
    s_true = meas["s_true"]
    Pa, Ra = forward_filter(meas, P[0], args.n_poses, [], use_grav=True)
    est0 = dict(P=Pa.copy(), R=Ra.copy(), s=1.0)
    R0 = meas["star"].get(0, Ra[0])
    be = int(args.stereo_baseline_err * 100)

    stereo_perfect = simulate_stereo(P, R, args.n_poses, np.random.default_rng(args.seed + 1),
                                     baseline_err=0.0, sigma_frac=args.stereo_sigma_frac,
                                     every=args.stereo_every)
    stereo_bias = simulate_stereo(P, R, args.n_poses, np.random.default_rng(args.seed + 2),
                                  baseline_err=args.stereo_baseline_err,
                                  sigma_frac=args.stereo_sigma_frac, every=args.stereo_every)

    def solve(stereo):
        g = _build_graph(args, args.n_poses, meas, P[0], R0, stereo=stereo)
        return g.solve(est0, iters=args.gn_iters)

    _, hist_gauge = solve(None)
    est_p, hist_perfect = solve(stereo_perfect)
    _, hist_bias = solve(stereo_bias)

    s_gauge = [float(e["s"]) for e in hist_gauge]
    s_perfect = [float(e["s"]) for e in hist_perfect]
    s_bias = [float(e["s"]) for e in hist_bias]
    n = len(hist_perfect)

    frames: list[Image.Image] = []
    print(f"rendering {n} iterate frames ...")
    for i in range(n):
        e = hist_perfect[i]
        fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.8),
                                 gridspec_kw={"width_ratios": [1.15, 1.0]})
        fig.suptitle(
            f"Phase 7(3): stereo baseline -> metric scale, no fixes   "
            f"GN iteration {i:2d}/{n - 1}   (scale {s_perfect[i]:.3f})",
            fontsize=12.5)

        ax = axes[0]
        ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
        ax.plot(P[:, 0], P[:, 1], "-", color="white", lw=2.4, label="ground truth")
        ax.plot(e["P"][:, 0], e["P"][:, 1], "-", color=PERFECT, lw=2.0,
                label="stereo estimate (true baseline)")
        ax.scatter(P[0, 0], P[0, 1], c="#19c819", marker="o", s=55, edgecolors="k",
                   zorder=6, label="known start (only anchor)")
        ax.set_xlim(0, extent_m); ax.set_ylim(0, extent_m)
        ax.set_title("no-fix map: dead-reckoned overshoot -> metric as scale drops",
                     fontsize=10.0)
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
        ax.legend(loc="upper left", fontsize=7.8)

        ax = axes[1]
        ax.axhline(s_true, color=TRUTHC, ls="--", lw=1.4, label=f"true scale {s_true:.3f}")
        ax.axhline(1.0, color="0.6", ls=":", lw=1.2, label="assumed 1.000")
        ax.plot(range(i + 1), s_gauge[:i + 1], "-o", color=GAUGE, lw=1.8, ms=3.0,
                label="no stereo -> gauge freedom")
        ax.plot(range(i + 1), s_perfect[:i + 1], "-o", color=PERFECT, lw=2.2, ms=3.5,
                label="stereo, true baseline")
        ax.plot(range(i + 1), s_bias[:i + 1], "-o", color=BIASP, lw=1.8, ms=3.0,
                label=f"stereo +{be}% baseline")
        ax.set_xlim(-0.5, n - 0.5)
        lo = min(s_true, min(s_perfect), min(s_bias)) - 0.02
        ax.set_ylim(lo, 1.0 + 0.02)
        ax.set_title("scale state per iteration: stereo breaks the gauge, baseline sets the bias",
                     fontsize=9.6)
        ax.set_xlabel("GN iteration"); ax.set_ylabel("global VO scale s")
        ax.legend(fontsize=8, loc="best")

        fig.tight_layout(rect=[0, 0, 1, 0.92])
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
