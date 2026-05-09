#!/usr/bin/env python3
"""Composite "lunar landing mission" demo GIF: star tracker + TRN side-by-side.

Six mission moments simulate a descent from orbit (400 km altitude) to
terminal landing approach (30 km), with both localisation modules running
independently per frame:

  LEFT panel  — star tracker. The spacecraft attitude rotates through six
                recognisable asterisms (Orion, Big Dipper, Cygnus+Lyra,
                Cassiopeia, Leo, Scorpius). Each frame runs the full
                star ID pipeline (render_star_image -> centroid_stars ->
                apps/lost_in_space_pair_id) and overlays the recovered
                identifications with constellation lines + bright-star
                labels.

  RIGHT panel — TRN. The descent camera looks down at Tycho at progressively
                lower altitude (400, 200, 100, 50, 30, 30 km). Each frame
                runs the LRO/LOLA bench (scripts/lro_trn_demo.py) and
                overlays the recovered position vs truth.

The composited frame includes a telemetry strip with attitude readout (deg)
+ position readout (m, with truth-vs-recovered error) + altitude bar. The
output is a single GIF that tells the localisation story end-to-end.

Inputs assumed cached:
  - HYG mag<=6.5 catalog converted to id,x,y,z,mag,ra_deg,dec_deg
  - Pair-index .bin for the star ID side
  - LROC WAC mosaic tiles (Trek WMTS, fetched on demand) and LDEM_64.img
    (PDS Geosciences, 530 MB one-time download)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from lro_trn_demo import (  # noqa: E402
    fetch_ldem,
    fetch_wac_mosaic,
    crop_ldem_to_wac,
    LUNAR_RADIUS_M,
    TARGETS,
)
from render_constellation_demo_gif import (  # noqa: E402
    NAMED_STARS,
    CONSTELLATION_LINES,
    closest_truth,
    load_assignments,
    load_observations,
    load_truth,
    text_with_shadow,
)
from synthetic_trn_demo import (  # noqa: E402
    match_features,
    recover_pose_pnp,
    render_rover_view,
    sample_height_bilinear,
    world_camera_rotation,
)


# Six mission moments. Each row is:
#   (label, attitude_yaw_pitch_roll_deg, trn_altitude_m, trn_zoom, trn_ldem_ppd)
# Star-tracker attitudes are the same as the constellation demo. TRN follows a
# descent profile from 400 km orbital (cycle 3, z=5+LDEM_4) down to 30 km
# terminal (cycle 4, z=8+LDEM_64). The (zoom, ppd) bumps are picked per
# altitude so the rover-view sample distance stays within ~2x of the WAC ortho:
#   400 km  z=5  ldem 4    179 m err   (orbital, very coarse DEM)
#   200 km  z=5  ldem 4     98 m err
#   100 km  z=7  ldem 4    101 m err   (z=7 ortho catches up to lower altitude)
#    50 km  z=7  ldem 4    334 m err   (transition zone — parallax growing)
#    30 km  z=8  ldem 64    32 m err   (terminal — finest data)
#    30 km  z=8  ldem 64    32 m err
MOMENTS: list[tuple[str, str, tuple[float, float, float], float, int, int]] = [
    ("Orbital insertion",  "Orion",         ( 0.0,  -5.9,   91.2),  400_000,  5,  4),
    ("Descent burn",       "Big Dipper",    ( 0.0,  32.9,   -2.5),  200_000,  5,  4),
    ("Approach phase",     "Cygnus + Lyra", ( 0.0, -18.3,  -45.2),  100_000,  7,  4),
    ("Powered descent",    "Cassiopeia",    ( 0.0, -32.9,    6.6),   50_000,  7,  4),
    ("Final approach",     "Leo",           ( 0.0,  58.5,   49.5),   30_000,  8, 64),
    ("Touchdown burn",     "Scorpius",      ( 0.0,  20.1, -118.3),   30_000,  8, 64),
]

TRN_TARGET = "tycho"  # one site for narrative continuity


def run(cmd: list[str]) -> None:
    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(f"$ {' '.join(cmd)}\n{result.stdout}\n{result.stderr}\n")
        raise SystemExit(result.returncode)


def render_star_tracker_panel(
    *,
    attitude_label: str,
    yaw: float, pitch: float, roll: float,
    catalog: Path, index_bin: Path, identifier_bin: Path,
    workdir: Path, frame_seed: int,
    panel_size: int,
) -> tuple[np.ndarray, dict]:
    """Run the full star-ID pipeline for one attitude and return a square panel
    with constellation lines + star labels overlaid, plus the assignment counts.
    """
    workdir.mkdir(parents=True, exist_ok=True)
    image_path = workdir / "exposure.png"
    truth_path = workdir / "truth.csv"
    obs_path = workdir / "observations.csv"
    assignments_path = workdir / "assignments.csv"

    # Render at native 1024 (matches render_star_image defaults)
    fx = fy = 900.0
    cx = cy = 512.0
    width = height = 1024
    run([
        sys.executable, str(REPO_ROOT / "scripts" / "render_star_image.py"),
        "--catalog", str(catalog),
        "--output-image", str(image_path),
        "--output-truth", str(truth_path),
        "--fx", str(fx), "--fy", str(fy),
        "--cx", str(cx), "--cy", str(cy),
        "--width", str(width), "--height", str(height),
        "--yaw-deg", str(yaw), "--pitch-deg", str(pitch), "--roll-deg", str(roll),
        "--seed", str(frame_seed),
    ])
    run([
        sys.executable, str(REPO_ROOT / "scripts" / "centroid_stars_from_image.py"),
        "--input-image", str(image_path),
        "--output-observations", str(obs_path),
    ])
    run([
        str(identifier_bin),
        "--observations", str(obs_path),
        "--index", str(index_bin),
        "--output", str(assignments_path),
        "--fx", str(fx), "--fy", str(fy),
        "--cx", str(cx), "--cy", str(cy),
        "--tolerance-arcsec", "120",
        "--neighbor-bins", "1",
        "--verification-tolerance-arcsec", "600",
        "--magnitude-prior-arcsec", "15",
        "--pyramid-size", "6",
        "--pyramid-restarts", "3",
        "--confidence-fraction", "0.5",
    ])

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    truth = load_truth(truth_path)
    observations = load_observations(obs_path)
    assignments = load_assignments(assignments_path)

    truth_lookup_by_id = {sid: (u, v) for sid, u, v in truth}
    correct_id_pixel: dict[str, tuple[float, float]] = {}
    counts = {"correct": 0, "wrong": 0, "unassigned": 0}
    match_px = 4.0
    for obs_index, (u, v) in enumerate(observations):
        truth_id = closest_truth(u, v, truth, match_px)
        assigned = assignments.get(obs_index)
        if assigned is None:
            counts["unassigned"] += 1
            continue
        if truth_id is not None and assigned == truth_id:
            counts["correct"] += 1
            correct_id_pixel[assigned] = (u, v)
        else:
            counts["wrong"] += 1

    # Constellation lines (cyan) between correctly identified stars in the
    # target constellation.
    out = image.copy()
    for con_tag, edges in CONSTELLATION_LINES.items():
        for star_a, star_b in edges:
            pa = correct_id_pixel.get(star_a)
            pb = correct_id_pixel.get(star_b)
            if pa is None or pb is None:
                continue
            cv2.line(out,
                     (int(round(pa[0])), int(round(pa[1]))),
                     (int(round(pb[0])), int(round(pb[1]))),
                     color=(255, 220, 80), thickness=1, lineType=cv2.LINE_AA)

    # Centroid rings: green = correct, red = wrong, faint blue = unassigned.
    for obs_index, (u, v) in enumerate(observations):
        truth_id = closest_truth(u, v, truth, match_px)
        assigned = assignments.get(obs_index)
        if assigned is None:
            color = (220, 180, 80); radius = 7
        elif truth_id is not None and assigned == truth_id:
            color = (90, 220, 120); radius = 8
        else:
            color = (60, 60, 240); radius = 8
        cv2.circle(out, (int(round(u)), int(round(v))), radius=radius,
                   color=color, thickness=2, lineType=cv2.LINE_AA)

    # Bright-star labels (gold).
    for star_id, (u, v) in correct_id_pixel.items():
        entry = NAMED_STARS.get(star_id)
        if entry is None:
            continue
        name, _ = entry
        text_with_shadow(out, name,
                         (int(round(u)) + 12, int(round(v)) - 7),
                         scale=0.55, color=(80, 220, 255), thickness=1)

    text_with_shadow(out, f"STAR TRACKER  -  {attitude_label}",
                     (16, 36), scale=0.95, color=(240, 240, 240), thickness=2)

    # Resize to panel size (with antialias)
    panel = cv2.resize(out, (panel_size, panel_size), interpolation=cv2.INTER_AREA)
    return panel, counts


def render_trn_panel(
    *,
    target: str, altitude_m: float, zoom: int, ldem_ppd: int, tile_radius: int,
    cache_dir: Path, panel_size: tuple[int, int],
) -> tuple[np.ndarray, dict]:
    """Run the LRO/LOLA TRN pipeline for one descent altitude, return a panel."""
    target_lat, target_lon = TARGETS[target]
    ortho, lat_min, lat_max, lon_min, lon_max = fetch_wac_mosaic(
        target_lat, target_lon, zoom=zoom, tile_radius=tile_radius, cache_dir=cache_dir
    )
    ldem = fetch_ldem(ldem_ppd, cache_dir)
    dem_c = crop_ldem_to_wac(ldem, ldem_ppd, lat_min, lat_max, lon_min, lon_max)
    heightmap_full = cv2.resize(dem_c, (ortho.shape[1], ortho.shape[0]),
                                interpolation=cv2.INTER_LINEAR)
    heightmap_m = heightmap_full - heightmap_full.mean()

    deg_per_px_lat = (lat_max - lat_min) / ortho.shape[0]
    deg_per_px_lon = (lon_max - lon_min) / ortho.shape[1]
    lat_origin = (lat_min + lat_max) * 0.5
    px_to_m_lat = math.radians(deg_per_px_lat) * LUNAR_RADIUS_M
    px_to_m_lon = math.radians(deg_per_px_lon) * LUNAR_RADIUS_M * math.cos(math.radians(lat_origin))
    px_to_m = (px_to_m_lat + px_to_m_lon) * 0.5

    rover_x_m = 0.5 * ortho.shape[1] * px_to_m
    rover_y_m = 0.5 * ortho.shape[0] * px_to_m
    truth_xyz = (rover_x_m, rover_y_m, altitude_m)
    R_wc = world_camera_rotation(0, -90, 0)
    rover = render_rover_view(
        ortho, heightmap_m,
        camera_xyz_m=truth_xyz, R_world_camera=R_wc,
        fx=520.0, fy=520.0, cx=320.0, cy=240.0,
        rover_width=640, rover_height=480, px_to_m=px_to_m,
    )

    pts_o, pts_r, _, _, _ = match_features(ortho, rover, 0.85, 2000)
    pnp = None
    if pts_o.shape[0] >= 6:
        pnp = recover_pose_pnp(
            pts_o, pts_r,
            heightmap_m=heightmap_m, px_to_m=px_to_m,
            fx=520.0, fy=520.0, cx=320.0, cy=240.0,
            reproj_error_px=5.0, ransac_iters=200_000,
        )

    if pnp is not None:
        estimated_xyz = pnp["rover_world_xyz_m"]
        position_error_m = math.sqrt(sum(
            (estimated_xyz[i] - truth_xyz[i]) ** 2 for i in range(3)
        ))
    else:
        estimated_xyz = None
        position_error_m = float("nan")

    rover_color = cv2.cvtColor(rover, cv2.COLOR_GRAY2BGR)
    text_with_shadow(rover_color,
                     f"TRN VIEW  -  {target.upper()}  -  alt {altitude_m/1000:.0f} km",
                     (12, 28), scale=0.7, color=(240, 240, 240), thickness=2)

    panel = cv2.resize(rover_color, panel_size, interpolation=cv2.INTER_AREA)
    return panel, {
        "match_count": int(pts_o.shape[0]),
        "estimated_xyz_m": estimated_xyz,
        "truth_xyz_m": list(truth_xyz),
        "position_error_m": position_error_m,
        "altitude_m": altitude_m,
        "px_to_m": px_to_m,
        "pnp_inliers": pnp["pnp_inliers"] if pnp else 0,
    }


def composite_frame(
    *,
    star_panel: np.ndarray, trn_panel: np.ndarray,
    attitude_label: str, yaw: float, pitch: float, roll: float,
    star_counts: dict, trn_data: dict,
    moment_label: str, moment_index: int, total_moments: int,
    frame_size: tuple[int, int],
) -> np.ndarray:
    """Lay out: STAR_TRACKER (left, square) | TRN (top-right) + HUD (bottom-right)."""
    width, height = frame_size
    frame = np.full((height, width, 3), 16, dtype=np.uint8)  # near-black

    star_h = height
    star_w = star_h
    if star_panel.shape != (star_h, star_w, 3):
        star_panel = cv2.resize(star_panel, (star_w, star_h), interpolation=cv2.INTER_AREA)
    frame[:, :star_w] = star_panel

    right_w = width - star_w - 8  # 8 px gutter
    trn_h = int(right_w * 0.75)  # 4:3
    trn_x0 = star_w + 8
    if trn_panel.shape[:2] != (trn_h, right_w):
        trn_panel = cv2.resize(trn_panel, (right_w, trn_h), interpolation=cv2.INTER_AREA)
    frame[:trn_h, trn_x0:trn_x0 + right_w] = trn_panel

    # HUD strip below TRN panel
    hud_y0 = trn_h + 12
    hud_y1 = height - 12
    hud_x0 = trn_x0 + 8
    cv2.rectangle(frame, (trn_x0, hud_y0 - 6), (trn_x0 + right_w, hud_y1),
                  color=(36, 36, 36), thickness=-1)

    # Header line: mission moment
    text_with_shadow(
        frame,
        f"MISSION  {moment_index + 1} / {total_moments}  -  {moment_label}",
        (hud_x0, hud_y0 + 22),
        scale=0.7, color=(240, 240, 240), thickness=2,
    )

    # Attitude readout
    text_with_shadow(
        frame, "ATTITUDE",
        (hud_x0, hud_y0 + 60),
        scale=0.55, color=(150, 200, 240), thickness=1,
    )
    att_str = f"yaw {yaw:+6.1f}  pitch {pitch:+6.1f}  roll {roll:+6.1f}  deg"
    text_with_shadow(
        frame, att_str,
        (hud_x0, hud_y0 + 86),
        scale=0.55, color=(240, 240, 240), thickness=1,
    )

    # Star tracker scoreboard
    sc = star_counts
    text_with_shadow(
        frame,
        f"stars: correct {sc['correct']}  wrong {sc['wrong']}  unassigned {sc['unassigned']}",
        (hud_x0, hud_y0 + 112),
        scale=0.5, color=(180, 220, 180), thickness=1,
    )

    # Position readout
    text_with_shadow(
        frame, "POSITION",
        (hud_x0, hud_y0 + 152),
        scale=0.55, color=(150, 200, 240), thickness=1,
    )
    truth = trn_data["truth_xyz_m"]
    estim = trn_data["estimated_xyz_m"]
    if estim is not None:
        text_with_shadow(
            frame,
            f"truth   ({truth[0]/1000:6.1f}, {truth[1]/1000:6.1f}, {truth[2]/1000:6.1f})  km",
            (hud_x0, hud_y0 + 178),
            scale=0.5, color=(200, 200, 200), thickness=1,
        )
        text_with_shadow(
            frame,
            f"recover ({estim[0]/1000:6.1f}, {estim[1]/1000:6.1f}, {estim[2]/1000:6.1f})  km",
            (hud_x0, hud_y0 + 200),
            scale=0.5, color=(200, 200, 200), thickness=1,
        )
        err_color = (90, 220, 120) if trn_data["position_error_m"] < 500 else (40, 200, 220)
        text_with_shadow(
            frame,
            f"error   {trn_data['position_error_m']:6.1f} m   "
            f"(matches {trn_data['match_count']}, inliers {trn_data['pnp_inliers']})",
            (hud_x0, hud_y0 + 222),
            scale=0.55, color=err_color, thickness=1,
        )
    else:
        text_with_shadow(
            frame, "PnP failed",
            (hud_x0, hud_y0 + 178),
            scale=0.55, color=(60, 60, 240), thickness=1,
        )

    # Altitude bar (visualises descent)
    bar_x0 = hud_x0
    bar_y0 = hud_y0 + 250
    bar_w = right_w - 16
    bar_h = 14
    cv2.rectangle(frame, (bar_x0, bar_y0), (bar_x0 + bar_w, bar_y0 + bar_h),
                  color=(60, 60, 60), thickness=1)
    alt_max = 400_000
    fill_frac = min(1.0, trn_data["altitude_m"] / alt_max)
    cv2.rectangle(frame,
                  (bar_x0 + 1, bar_y0 + 1),
                  (bar_x0 + 1 + int((bar_w - 2) * fill_frac), bar_y0 + bar_h - 1),
                  color=(80, 160, 220), thickness=-1)
    text_with_shadow(
        frame,
        f"altitude  {trn_data['altitude_m']/1000:5.0f} km",
        (bar_x0, bar_y0 + bar_h + 22),
        scale=0.5, color=(180, 200, 220), thickness=1,
    )

    return frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path,
                        default=REPO_ROOT / "datasets" / "star_catalogs" / "hyg-v42" /
                        "converted" / "hyg_v42_bright_mag6p5_unit.csv")
    parser.add_argument("--index-bin", type=Path, required=True,
                        help="Pair-index .bin (build with build_star_pair_index.py --write-bin)")
    parser.add_argument("--identifier-bin", type=Path,
                        default=REPO_ROOT / "build" / "apps" / "lost_in_space_pair_id")
    parser.add_argument("--cache-dir", type=Path,
                        default=REPO_ROOT / "datasets" / "lro_cache")
    parser.add_argument("--workdir", type=Path, default=Path("/tmp/mission_demo_gif"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-width", type=int, default=1280)
    parser.add_argument("--frame-height", type=int, default=720)
    parser.add_argument("--ms-per-frame", type=int, default=2200)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    args.workdir.mkdir(parents=True, exist_ok=True)
    args.cache_dir.mkdir(parents=True, exist_ok=True)

    star_panel_size = args.frame_height
    right_w = args.frame_width - star_panel_size - 8
    trn_h = int(right_w * 0.75)
    trn_panel_size = (right_w, trn_h)

    frames_pil: list[Image.Image] = []
    summary_rows: list[str] = []
    for idx, (moment, attitude_label, (yaw, pitch, roll), altitude_m, zoom, ppd) in enumerate(MOMENTS):
        frame_dir = args.workdir / f"frame_{idx:02d}_{moment.replace(' ', '_').lower()}"
        frame_dir.mkdir(parents=True, exist_ok=True)

        print(f"\n[{idx + 1}/{len(MOMENTS)}] {moment}: attitude {attitude_label}, "
              f"TRN alt {altitude_m/1000:.0f} km z={zoom} ppd={ppd}")
        star_panel, star_counts = render_star_tracker_panel(
            attitude_label=attitude_label,
            yaw=yaw, pitch=pitch, roll=roll,
            catalog=args.catalog, index_bin=args.index_bin,
            identifier_bin=args.identifier_bin,
            workdir=frame_dir / "star",
            frame_seed=args.seed + idx,
            panel_size=star_panel_size,
        )
        trn_panel, trn_data = render_trn_panel(
            target=TRN_TARGET, altitude_m=altitude_m,
            zoom=zoom, ldem_ppd=ppd, tile_radius=2 if zoom >= 7 else 1,
            cache_dir=args.cache_dir, panel_size=trn_panel_size,
        )
        composite = composite_frame(
            star_panel=star_panel, trn_panel=trn_panel,
            attitude_label=attitude_label, yaw=yaw, pitch=pitch, roll=roll,
            star_counts=star_counts, trn_data=trn_data,
            moment_label=moment, moment_index=idx, total_moments=len(MOMENTS),
            frame_size=(args.frame_width, args.frame_height),
        )
        msg = (f"  star: correct={star_counts['correct']} wrong={star_counts['wrong']} "
               f"unassigned={star_counts['unassigned']}; "
               f"trn: matches={trn_data['match_count']} err={trn_data['position_error_m']:.0f} m")
        print(msg)
        summary_rows.append(msg)
        frames_pil.append(Image.fromarray(cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames_pil[0].save(
        args.output,
        save_all=True,
        append_images=frames_pil[1:],
        duration=args.ms_per_frame,
        loop=0,
        optimize=True,
    )
    print(f"\nwrote {args.output}  ({len(frames_pil)} frames @ {args.ms_per_frame} ms/frame)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
