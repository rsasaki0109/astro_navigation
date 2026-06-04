#!/usr/bin/env python3
"""Skyline Lock: celestial-horizon TRN proof-of-concept (synthetic + real LOLA).

A GNSS-denied lunar rover with a known-ish attitude (from the star tracker) looks
at the black sky / terrain boundary. On the Moon the horizon is a sharp, high-SNR
absolute cue: the elevation-angle-vs-azimuth profile of distant ridges and crater
rims is a fingerprint of *where you are standing*. This script asks the only
question that matters before any heavier plumbing is built:

    Given a DEM, does matching an observed horizon profile against horizons
    predicted across a candidate-position grid produce a usable localization
    peak -- and where does it degenerate?

Pipeline:
  1. Build a DEM. `--source synth` makes a procedural lunar DEM
     (`--terrain hills|craters|flat`, no download); `--source lola` crops the
     real LOLA LDEM around `--target` (e.g. tycho), reusing lro_trn_demo.py.
  2. Pick a truth rover position + heading. Render its 360-deg horizon profile
     by ray-marching the DEM outward along each azimuth -- the NEW geometry core
     (the existing TRN renderer marches *downward* for a nadir view; Skyline
     marches *outward* to find the first sky-occluding elevation).
  3. Hide the truth. Over a grid of candidate (x, y) positions, render each
     predicted horizon and score it against the observation with a zero-mean
     normalized cross-correlation. Heading is recovered as the circular lag;
     without `--yaw-prior-deg` the lag is fully marginalized (admits symmetric
     aliases), with it the lag search is windowed to +/- 3 sigma (the star
     tracker pins heading and rejects gross mismatches).
  4. Emit a Top-K ranking, best NCC (match quality), a uniqueness_margin
     (best minus the strongest competing spatial mode = localizability), a
     score-surface heatmap, and an observed-vs-predicted horizon overlay.

The honest-envelope point: skyline matching localizes sub-cell on terrain with
relief and recovers heading to ~1 deg, but uniqueness depends on terrain
*distinctiveness*, not on the yaw prior. Self-similar synth terrain and the
rotational symmetry of a circular crater rim both alias (low margin) even when
the global peak is correct; a distinctive feature seen from its centre locks
unambiguously (high margin). We report a Top-K posterior + margin rather than
asserting a single fix -- matching the project's "show the cliffs" stance.

Conventions match scripts/synthetic_trn_demo.py: world +X east, +Y north,
+Z up; the heightmap is indexed by world coords via `col = x / px_to_m`,
`row = y / px_to_m`. Azimuth 0 points along +Y (north) and increases clockwise
toward +X (east): (dx, dy) = (sin az, cos az).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# Synthetic terrain                                                           #
# --------------------------------------------------------------------------- #
def synth_dem(
    size_px: int, px_to_m: float, seed: int, terrain: str
) -> np.ndarray:
    """Return a (size_px, size_px) float32 heightmap in metres.

    `terrain`:
      - hills:   long-wavelength ridges + a few big massifs -> strong skyline.
      - craters: rim-and-bowl crater field -> moderate, localized skyline cues.
      - flat:    near-zero relief -> deliberately degenerate (mare) baseline.
    """
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:size_px, 0:size_px].astype(np.float64)
    h = np.zeros((size_px, size_px), dtype=np.float64)

    if terrain == "flat":
        # Sub-metre roughness only: there is essentially no horizon to lock to.
        h += rng.normal(0.0, 0.5, size=h.shape)
        return h.astype(np.float32)

    if terrain in ("hills", "craters"):
        if terrain == "hills":
            # Sum of a handful of random low-frequency sinusoids: smooth ridges
            # whose crests rise tens to ~hundreds of metres across the map.
            n_waves = 6
            for _ in range(n_waves):
                kx = rng.uniform(0.6, 3.0) * (2.0 * math.pi / size_px)
                ky = rng.uniform(0.6, 3.0) * (2.0 * math.pi / size_px)
                phase = rng.uniform(0.0, 2.0 * math.pi)
                amp = rng.uniform(40.0, 160.0)
                h += amp * np.sin(kx * xx + ky * yy + phase)
            # A couple of dominant massifs to anchor a few azimuths strongly.
            for _ in range(3):
                cx = rng.uniform(0.15, 0.85) * size_px
                cy = rng.uniform(0.15, 0.85) * size_px
                sig = rng.uniform(0.08, 0.16) * size_px
                amp = rng.uniform(250.0, 500.0)
                r2 = (xx - cx) ** 2 + (yy - cy) ** 2
                h += amp * np.exp(-r2 / (2.0 * sig * sig))

        # Crater rims/bowls (the workhorse skyline texture for `craters`, and a
        # bit of extra structure for `hills`).
        n_craters = 14 if terrain == "craters" else 5
        for _ in range(n_craters):
            cx = rng.uniform(0.08, 0.92) * size_px
            cy = rng.uniform(0.08, 0.92) * size_px
            radius = rng.uniform(0.05, 0.14) * size_px
            r = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
            sigma_bowl = max(2.0, radius * 0.5)
            sigma_rim = max(1.5, radius * 0.16)
            bowl = np.exp(-(r ** 2) / (2.0 * sigma_bowl * sigma_bowl))
            rim = np.exp(-((r - radius) ** 2) / (2.0 * sigma_rim * sigma_rim))
            h += -180.0 * bowl + 220.0 * rim

        h += rng.normal(0.0, 1.0, size=h.shape)
        return h.astype(np.float32)

    raise SystemExit(f"unknown terrain '{terrain}' (use hills|craters|flat)")


def load_lola_dem(target: str, half_deg: float, ppd: int, cache_dir):
    """Crop the real LOLA LDEM around a landmark into an isotropic square DEM.

    Reuses scripts/lro_trn_demo.py's LDEM fetcher/loader. The raw crop is
    lat/lon-gridded, so longitude pixels are compressed by cos(lat) in metres;
    we resample the lon axis to make the grid isotropic (single metres/pixel),
    then centre-crop to a square so the localizer can treat it like a synth DEM.

    NOTE (honest envelope): the horizon renderer uses a flat-plane model and
    ignores lunar curvature. Over a ~180 km box the curvature drop (~v=d^2/2R)
    reaches several hundred metres -- comparable to relief -- so absolute
    horizon elevations are biased. Relative matching across candidates still
    holds (same renderer everywhere); curvature-correct ranging is a Phase-3+ fix.
    """
    import sys as _sys
    repo_root = Path(__file__).resolve().parent.parent
    _sys.path.insert(0, str(repo_root / "scripts"))
    from lro_trn_demo import TARGETS, fetch_ldem, LUNAR_RADIUS_M  # noqa: E402

    if target not in TARGETS:
        raise SystemExit(f"unknown target '{target}'. choices: {sorted(TARGETS)}")
    lat, lon = TARGETS[target]
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    ldem = fetch_ldem(ppd, cache_dir)            # (180ppd, 360ppd), row0=+90 lat
    n_lat, n_lon = ldem.shape

    r_c = int(round((90.0 - lat) * ppd))
    c_c = int(round((lon % 360.0) * ppd))
    half = int(round(half_deg * ppd))
    r0, r1 = max(0, r_c - half), min(n_lat, r_c + half)
    cols = [(c) % n_lon for c in range(c_c - half, c_c + half)]
    patch = ldem[r0:r1][:, cols].astype(np.float32)

    deg_per_px = 1.0 / ppd
    px_to_m_lat = math.radians(deg_per_px) * LUNAR_RADIUS_M
    px_to_m_lon = px_to_m_lat * math.cos(math.radians(lat))
    h, w = patch.shape
    new_w = max(1, int(round(w * px_to_m_lon / px_to_m_lat)))
    iso = cv2.resize(patch, (new_w, h), interpolation=cv2.INTER_LINEAR)

    s = min(iso.shape)
    r_off = (iso.shape[0] - s) // 2
    c_off = (iso.shape[1] - s) // 2
    square = iso[r_off:r_off + s, c_off:c_off + s].copy()
    return square, px_to_m_lat


def build_dem(args) -> tuple[np.ndarray, float]:
    """Return (square float32 heightmap in metres, px_to_m)."""
    if args.source == "lola":
        return load_lola_dem(args.target, args.half_width_deg,
                             args.ldem_ppd, args.cache_dir)
    return synth_dem(args.size_px, args.px_to_m, args.seed, args.terrain), args.px_to_m


# --------------------------------------------------------------------------- #
# Horizon renderer -- the new geometry core                                   #
# --------------------------------------------------------------------------- #
def render_horizon(
    heightmap_m: np.ndarray,
    px_to_m: float,
    cam_xy_m: tuple[float, float],
    mast_height_m: float,
    *,
    n_az: int,
    r_min_m: float,
    r_max_m: float,
    n_range: int,
) -> np.ndarray:
    """Return horizon elevation angle (radians) per azimuth bin, shape (n_az,).

    For each azimuth, march outward in ground range and take the maximum
    elevation angle arctan2(h(r) - cam_z, r) -- the skyline is the upper
    envelope of the terrain seen from the camera. Off-map samples are ignored;
    an azimuth with no valid sample is reported as 0 (level horizon).
    """
    cam_x, cam_y = cam_xy_m
    cam_z = _sample_height(heightmap_m, px_to_m, cam_x, cam_y) + mast_height_m

    az = np.linspace(0.0, 2.0 * math.pi, n_az, endpoint=False)  # (A,)
    rng_m = np.linspace(r_min_m, r_max_m, n_range)              # (R,)
    # Outgoing unit directions: az measured from +Y (north), clockwise to +X.
    dx = np.sin(az)[:, None]  # (A, 1)
    dy = np.cos(az)[:, None]
    xs = cam_x + rng_m[None, :] * dx  # (A, R) world metres
    ys = cam_y + rng_m[None, :] * dy

    map_x = (xs / px_to_m).astype(np.float32)
    map_y = (ys / px_to_m).astype(np.float32)
    h = cv2.remap(
        heightmap_m, map_x, map_y, cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=float("nan"),
    ).astype(np.float64)  # (A, R)

    elev = np.arctan2(h - cam_z, rng_m[None, :])  # (A, R)
    elev = np.where(np.isfinite(elev), elev, -np.inf)
    horizon = np.max(elev, axis=1)               # (A,)
    horizon = np.where(np.isfinite(horizon), horizon, 0.0)
    return horizon


def _sample_height(heightmap_m: np.ndarray, px_to_m: float, x: float, y: float) -> float:
    h, w = heightmap_m.shape
    cx = min(max(x / px_to_m, 0.0), w - 1.0)
    cy = min(max(y / px_to_m, 0.0), h - 1.0)
    val = cv2.remap(
        heightmap_m,
        np.array([[cx]], dtype=np.float32), np.array([[cy]], dtype=np.float32),
        cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE,
    )
    return float(val[0, 0])


# --------------------------------------------------------------------------- #
# Yaw-marginalized profile scoring                                            #
# --------------------------------------------------------------------------- #
def _zero_mean_unit(p: np.ndarray) -> np.ndarray:
    p = p - p.mean(axis=-1, keepdims=True)
    norm = np.linalg.norm(p, axis=-1, keepdims=True)
    norm = np.where(norm < 1e-12, 1.0, norm)
    return p / norm


def best_yaw_ncc(
    obs: np.ndarray,
    preds: np.ndarray,
    *,
    prior_lag: int | None = None,
    window_bins: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Best circular-cross-correlation (over yaw) of obs vs each pred.

    obs: (A,). preds: (M, A). Returns (scores (M,), best_lag_bins (M,)).
    Yaw is the circular shift maximizing zero-mean normalized correlation, so an
    absolute elevation bias (height uncertainty) and the heading both fall out.

    Without a prior the search spans all A lags (the Phase-0 yaw-free case, which
    admits symmetric false peaks). With a star-tracker yaw prior, the lag search
    is restricted to a +/- window_bins window around prior_lag (a near-uniform
    prior inside the window) -- this is what breaks the symmetric ambiguity.
    """
    o = _zero_mean_unit(obs.astype(np.float64))          # (A,)
    p = _zero_mean_unit(preds.astype(np.float64))        # (M, A)
    A = o.shape[0]
    O = np.fft.rfft(o)                                   # (A//2+1,)
    P = np.fft.rfft(p, axis=1)                           # (M, A//2+1)
    # corr[m, s] = circular cross-correlation of obs and pred m at lag s.
    corr = np.fft.irfft(np.conj(O)[None, :] * P, n=A, axis=1)  # (M, A)

    if prior_lag is not None and window_bins is not None:
        lags = np.arange(A)
        # Circular distance of each lag from the prior, in bins.
        d = np.minimum((lags - prior_lag) % A, (prior_lag - lags) % A)
        allowed = d <= window_bins
        corr = np.where(allowed[None, :], corr, -np.inf)

    best_lag = corr.argmax(axis=1)
    scores = corr[np.arange(corr.shape[0]), best_lag]
    return scores, best_lag


def uniqueness_margin(
    ncc: np.ndarray, cand_xy: list, est_xy: tuple[float, float], radius_m: float
) -> tuple[float, float]:
    """Return (margin, second_mode_ncc).

    second_mode_ncc is the highest score among candidates farther than radius_m
    from the estimate -- i.e. the best *competing* spatial mode. margin =
    best_ncc - second_mode_ncc. A small margin means the skyline does not pin a
    unique position (symmetric / aliased horizons); a large margin means the
    lock is spatially unambiguous. This is the honest localizability signal
    (vs an absolute peak-to-sidelobe, which a flat noise field can game).
    """
    best = float(ncc.max())
    ex, ey = est_xy
    far = np.array([
        ncc[i] for i, (x, y) in enumerate(cand_xy)
        if math.hypot(x - ex, y - ey) > radius_m
    ])
    if far.size == 0:
        return 0.0, best
    second = float(far.max())
    return best - second, second


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["synth", "lola"], default="synth",
                    help="synth = procedural DEM; lola = real LOLA LDEM patch "
                    "around --target (Phase 2 real-terrain validation).")
    ap.add_argument("--terrain", choices=["hills", "craters", "flat"], default="hills")
    ap.add_argument("--target", default="tycho",
                    help="LOLA target landmark (used when --source lola).")
    ap.add_argument("--ldem-ppd", type=int, default=16, choices=[4, 16, 64],
                    help="LOLA LDEM resolution: 4~7.6km/px (~2MB), 16~1.9km/px "
                    "(~33MB, default), 64~470m/px (~530MB).")
    ap.add_argument("--half-width-deg", type=float, default=3.0,
                    help="Half-extent of the LOLA crop in degrees (default 3 -> "
                    "~180 km box, captures Tycho's full rim).")
    ap.add_argument("--cache-dir", type=Path,
                    default=Path(__file__).resolve().parent.parent / "datasets" / "lro_cache")
    ap.add_argument("--size-px", type=int, default=512)
    ap.add_argument("--px-to-m", type=float, default=30.0,
                    help="DEM ground sample distance in metres/pixel (synth only).")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--mast-height-m", type=float, default=2.0)
    ap.add_argument("--truth-frac", type=float, nargs=2, default=(0.5, 0.5),
                    help="Truth rover position as a fraction of map extent (fx fy).")
    ap.add_argument("--n-az", type=int, default=180)
    ap.add_argument("--r-min-m", type=float, default=60.0)
    ap.add_argument("--r-max-m", type=float, default=None,
                    help="Max horizon search range (default: ~0.9 * map width).")
    ap.add_argument("--n-range", type=int, default=160)
    ap.add_argument("--grid", type=int, default=41,
                    help="Candidate grid is grid x grid over the central map region.")
    ap.add_argument("--grid-margin-frac", type=float, default=0.12,
                    help="Keep candidates this far from the map edge.")
    ap.add_argument("--noise-arcmin", type=float, default=2.0,
                    help="Gaussian noise added to the observed horizon (arcmin).")
    ap.add_argument("--truth-yaw-deg", type=float, default=35.0,
                    help="The rover's true heading; rolls the observed horizon "
                    "into camera-relative azimuth.")
    ap.add_argument("--yaw-prior-deg", type=float, default=None,
                    help="Star-tracker heading estimate (typically truth_yaw +/- "
                    "tracker error). If set, the yaw search is restricted to a "
                    "window around it (Phase 1). Omit for the yaw-free Phase-0 "
                    "marginalization.")
    ap.add_argument("--yaw-sigma-deg", type=float, default=5.0,
                    help="Star-tracker heading 1-sigma; the lag window is +/- 3 sigma.")
    ap.add_argument("--topk", type=int, default=5)
    ap.add_argument("--output", type=Path, default=Path("outputs/skyline_lock/skyline_lock.png"))
    ap.add_argument("--output-json", type=Path, default=None)
    args = ap.parse_args()

    dem, px_to_m = build_dem(args)
    size = dem.shape[0]
    extent_m = size * px_to_m
    r_max = args.r_max_m if args.r_max_m is not None else 0.9 * extent_m

    A = args.n_az
    bins_per_deg = A / 360.0
    # Start the horizon march at least ~2 px out so the first sample clears the
    # camera's own cell (matters for coarse LOLA grids where 1 px is ~km).
    r_min = max(args.r_min_m, 2.0 * px_to_m)

    # Truth position and its observed horizon (with sensor noise). The rover
    # heading rolls the world-azimuth horizon into camera-relative azimuth:
    # obs[c] = H_world[(c + heading) mod A]  ==  roll(H_world, -heading_bins).
    truth_xy = (args.truth_frac[0] * extent_m, args.truth_frac[1] * extent_m)
    horizon_world = render_horizon(dem, px_to_m, truth_xy, args.mast_height_m,
                                   n_az=A, r_min_m=r_min, r_max_m=r_max,
                                   n_range=args.n_range)
    heading_bins = int(round(args.truth_yaw_deg * bins_per_deg)) % A
    obs = np.roll(horizon_world, -heading_bins)
    noise_rad = math.radians(args.noise_arcmin / 60.0)
    rng = np.random.default_rng(args.seed + 1)
    obs_noisy = obs + rng.normal(0.0, noise_rad, size=obs.shape)

    # Candidate grid over the central region.
    m = args.grid_margin_frac
    gx = np.linspace(m * extent_m, (1.0 - m) * extent_m, args.grid)
    gy = np.linspace(m * extent_m, (1.0 - m) * extent_m, args.grid)
    cand_xy = [(x, y) for y in gy for x in gx]  # row-major: row=y, col=x

    preds = np.stack([
        render_horizon(dem, px_to_m, (x, y), args.mast_height_m,
                       n_az=args.n_az, r_min_m=r_min, r_max_m=r_max,
                       n_range=args.n_range)
        for (x, y) in cand_xy
    ])  # (M, A)

    if args.yaw_prior_deg is not None:
        prior_lag = int(round(args.yaw_prior_deg * bins_per_deg)) % A
        window_bins = max(1, int(math.ceil(3.0 * args.yaw_sigma_deg * bins_per_deg)))
    else:
        prior_lag, window_bins = None, None
    ncc, best_lag = best_yaw_ncc(obs_noisy, preds,
                                 prior_lag=prior_lag, window_bins=window_bins)
    score_grid = ncc.reshape(args.grid, args.grid)  # (row=y, col=x)

    flat_idx = int(np.argmax(ncc))
    peak_r, peak_c = divmod(flat_idx, args.grid)
    est_xy = cand_xy[flat_idx]
    err_m = math.hypot(est_xy[0] - truth_xy[0], est_xy[1] - truth_xy[1])
    # Recovered heading from the winning candidate's best lag.
    est_yaw_deg = (best_lag[flat_idx] / bins_per_deg) % 360.0
    yaw_err_deg = abs((est_yaw_deg - args.truth_yaw_deg + 180.0) % 360.0 - 180.0)
    grid_step_m = (gx[1] - gx[0]) if args.grid > 1 else extent_m
    margin, second_ncc = uniqueness_margin(ncc, cand_xy, est_xy,
                                           radius_m=2.0 * grid_step_m)

    order = np.argsort(-ncc)[: args.topk]
    topk = [{
        "rank": i + 1,
        "x_m": round(cand_xy[j][0], 2),
        "y_m": round(cand_xy[j][1], 2),
        "ncc": round(float(ncc[j]), 4),
        "err_m": round(math.hypot(cand_xy[j][0] - truth_xy[0],
                                  cand_xy[j][1] - truth_xy[1]), 2),
    } for i, j in enumerate(order)]

    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    summary = {
        "scene": scene,
        "yaw_mode": "star-prior" if prior_lag is not None else "yaw-free",
        "extent_m": round(extent_m, 1),
        "px_to_m": px_to_m,
        "grid": args.grid,
        "grid_step_m": round(float(grid_step_m), 2),
        "n_az": args.n_az,
        "r_max_m": round(r_max, 1),
        "noise_arcmin": args.noise_arcmin,
        "truth_yaw_deg": args.truth_yaw_deg,
        "yaw_prior_deg": args.yaw_prior_deg,
        "est_yaw_deg": round(est_yaw_deg, 2),
        "yaw_error_deg": round(yaw_err_deg, 2),
        "truth_xy_m": [round(truth_xy[0], 2), round(truth_xy[1], 2)],
        "est_xy_m": [round(est_xy[0], 2), round(est_xy[1], 2)],
        "position_error_m": round(err_m, 2),
        "best_ncc": round(float(ncc[flat_idx]), 4),
        "second_mode_ncc": round(second_ncc, 4),
        "uniqueness_margin": round(margin, 4),
        # Two-part honest gate: the horizon must match well in absolute terms
        # (best_ncc) AND the recovered position must be grid-consistent. The
        # margin is reported separately so a strong-but-ambiguous lock (good
        # NCC, small margin -> symmetric horizon) is visible rather than hidden.
        "matched": bool(ncc[flat_idx] >= 0.9),
        "localized": bool(ncc[flat_idx] >= 0.9 and err_m <= 1.5 * grid_step_m),
        "unique": bool(margin >= 0.05),
        "topk": topk,
    }
    print(json.dumps(summary, indent=2))

    _render_figure(args, dem, px_to_m, extent_m, gx, gy, score_grid,
                   truth_xy, est_xy, obs_noisy, preds[flat_idx],
                   int(best_lag[flat_idx]), order, cand_xy)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.output_json}")

    return 0


def _render_figure(args, dem, px_to_m, extent_m, gx, gy, score_grid,
                   truth_xy, est_xy, obs, pred_best, shift, order, cand_xy) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    yaw_mode = "star-prior" if args.yaw_prior_deg is not None else "yaw-free"
    scene = f"lola:{args.target}" if args.source == "lola" else f"synth:{args.terrain}"
    fig.suptitle(
        f"Skyline Lock  -  {scene}  yaw={yaw_mode}  "
        f"err={math.hypot(est_xy[0]-truth_xy[0], est_xy[1]-truth_xy[1]):.0f} m  "
        f"best NCC={score_grid.max():.3f}",
        fontsize=13)

    # Panel 1: DEM with truth / estimate / top-K.
    ax = axes[0]
    im = ax.imshow(dem, origin="lower", extent=[0, extent_m, 0, extent_m], cmap="terrain")
    ax.scatter(*truth_xy, c="white", marker="*", s=240, edgecolors="k",
               label="truth", zorder=5)
    ax.scatter(*est_xy, c="red", marker="x", s=120, label="estimate", zorder=5)
    for j in order[1:]:
        ax.scatter(*cand_xy[j], c="orange", marker="o", s=24, zorder=4)
    ax.set_title("DEM (m) + truth/estimate")
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    ax.legend(loc="upper right", fontsize=8)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # Panel 2: score surface.
    ax = axes[1]
    im = ax.imshow(score_grid, origin="lower",
                   extent=[gx[0], gx[-1], gy[0], gy[-1]], cmap="viridis",
                   aspect="auto")
    ax.scatter(*truth_xy, c="white", marker="*", s=200, edgecolors="k", zorder=5)
    ax.set_title("horizon-match score surface (best-yaw NCC)")
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # Panel 3: observed vs best-candidate predicted horizon (yaw-aligned).
    ax = axes[2]
    A = obs.shape[0]
    az_deg = np.degrees(np.linspace(0.0, 2.0 * math.pi, A, endpoint=False))
    # corr lag convention: obs[n] ~= pred[n + shift], so overlay roll(pred, -shift).
    ax.plot(az_deg, np.degrees(obs), label="observed", lw=1.5)
    ax.plot(az_deg, np.degrees(np.roll(pred_best, -shift)),
            label=f"predicted (yaw {shift*360//A} deg)", lw=1.2, alpha=0.8)
    ax.set_title("horizon profile: observed vs predicted@estimate")
    ax.set_xlabel("azimuth (deg)"); ax.set_ylabel("elevation (deg)")
    ax.legend(fontsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    sys.exit(main())
