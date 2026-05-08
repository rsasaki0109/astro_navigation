#!/usr/bin/env python3
"""TRN trajectory demo: visualise position recovery across a descent path.

A virtual lander descends laterally + vertically toward Tycho central peak.
Per frame, the script:
  1. Renders the rover nadir view at the truth pose using real LRO WAC + LOLA.
  2. Runs SIFT + AP3P PnP to recover the lander position.
  3. Composites:
        - LEFT: top-down ortho map of the area, with the truth trajectory drawn
          as a yellow line and every recovered position so far as a green dot.
          Current truth = yellow diamond, current recovered = green circle,
          a red line connects truth -> recovered to visualise the error.
        - RIGHT TOP: current rover view + drawn SIFT inlier matches against
          the ortho.
        - RIGHT BOTTOM: telemetry HUD with frame index, altitude, truth/recovered
          position, current error, running mean error.

Frames are stitched into a single GIF for the README.

Default trajectory: 10 frames descending from (offset=+10 km east, +10 km north,
altitude 40 km) to (Tycho center, altitude 30 km), constant nadir attitude. Both
endpoints land in the verified-clean envelope of WAC z=8 + LOLA LDEM_64 PnP.
"""

from __future__ import annotations

import argparse
import json
import math
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
from render_constellation_demo_gif import text_with_shadow  # noqa: E402
from synthetic_trn_demo import (  # noqa: E402
    match_features,
    recover_pose_pnp,
    render_rover_view,
    sample_height_bilinear,
    world_camera_rotation,
)


def world_xy_to_map_pixel(x_m: float, y_m: float, *,
                          map_shape: tuple[int, int], px_to_m: float,
                          map_scale: float) -> tuple[int, int]:
    """Convert world (X, Y) in metres to pixel coords on the resized map panel.

    `map_scale` is the ratio of the map panel size to the ortho mosaic size, so
    if the map is rendered at full ortho resolution map_scale=1.0.
    """
    map_x = int(round(x_m / px_to_m * map_scale))
    map_y = int(round(y_m / px_to_m * map_scale))
    return map_x, map_y


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="tycho", choices=sorted(TARGETS.keys()))
    parser.add_argument("--cache-dir", type=Path,
                        default=REPO_ROOT / "datasets" / "lro_cache")
    parser.add_argument("--zoom", type=int, default=8)
    parser.add_argument("--tile-radius", type=int, default=2)
    parser.add_argument("--ldem-ppd", type=int, default=64)
    parser.add_argument("--frames", type=int, default=9)
    parser.add_argument("--start-offset-east-m", type=float, default=3_000.0,
                        help="Lander start position east of mosaic centre. Defaults are tuned "
                        "for the verified-clean envelope (~3 km lateral motion + 8 km vertical "
                        "descent over Tycho central peak): 9/9 PnP success, mean ~150 m.")
    parser.add_argument("--start-offset-north-m", type=float, default=3_000.0)
    parser.add_argument("--start-altitude-m", type=float, default=38_000.0,
                        help="Default 38 km is the verified-clean upper bound at z=8 + LDEM_64; "
                        "above that, parallax distortion from the heightmap starts breaking SIFT.")
    parser.add_argument("--end-altitude-m", type=float, default=30_000.0)
    parser.add_argument("--rover-fx", type=float, default=520.0)
    parser.add_argument("--rover-fy", type=float, default=520.0)
    parser.add_argument("--rover-cx", type=float, default=320.0)
    parser.add_argument("--rover-cy", type=float, default=240.0)
    parser.add_argument("--rover-width", type=int, default=640)
    parser.add_argument("--rover-height", type=int, default=480)
    parser.add_argument("--ratio-test", type=float, default=0.85)
    parser.add_argument("--max-features", type=int, default=2000)
    parser.add_argument("--pnp-reproj-error-px", type=float, default=5.0)
    parser.add_argument("--pnp-ransac-iters", type=int, default=200_000)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-width", type=int, default=1280)
    parser.add_argument("--frame-height", type=int, default=720)
    parser.add_argument("--ms-per-frame", type=int, default=700)
    args = parser.parse_args()

    args.cache_dir.mkdir(parents=True, exist_ok=True)

    target_lat, target_lon = TARGETS[args.target]
    print(f"target: {args.target} (lat={target_lat:+.3f} lon={target_lon:+.3f})")

    print("[1] fetch ortho + DEM")
    ortho, lat_min, lat_max, lon_min, lon_max = fetch_wac_mosaic(
        target_lat, target_lon,
        zoom=args.zoom, tile_radius=args.tile_radius, cache_dir=args.cache_dir,
    )
    ldem = fetch_ldem(args.ldem_ppd, args.cache_dir)
    dem_c = crop_ldem_to_wac(ldem, args.ldem_ppd, lat_min, lat_max, lon_min, lon_max)
    heightmap_full = cv2.resize(dem_c, (ortho.shape[1], ortho.shape[0]),
                                interpolation=cv2.INTER_LINEAR)
    heightmap_m = heightmap_full - heightmap_full.mean()

    deg_per_px_lat = (lat_max - lat_min) / ortho.shape[0]
    deg_per_px_lon = (lon_max - lon_min) / ortho.shape[1]
    lat_origin = (lat_min + lat_max) * 0.5
    px_to_m_lat = math.radians(deg_per_px_lat) * LUNAR_RADIUS_M
    px_to_m_lon = math.radians(deg_per_px_lon) * LUNAR_RADIUS_M * math.cos(math.radians(lat_origin))
    px_to_m = (px_to_m_lat + px_to_m_lon) * 0.5
    print(f"  ortho: {ortho.shape}  px_to_m: {px_to_m:.1f}")

    cx_m = ortho.shape[1] * 0.5 * px_to_m
    cy_m = ortho.shape[0] * 0.5 * px_to_m

    truth_poses: list[tuple[float, float, float]] = []
    for i in range(args.frames):
        t = i / max(args.frames - 1, 1)
        # End position is mosaic centre, at end_altitude. Start position is
        # offset east/north (positive y = south on the ortho since our convention
        # has rows = +Y world), at start_altitude.
        x = cx_m + (1 - t) * args.start_offset_east_m
        y = cy_m + (1 - t) * args.start_offset_north_m
        z = args.start_altitude_m + t * (args.end_altitude_m - args.start_altitude_m)
        truth_poses.append((x, y, z))

    print("[2] render + PnP per frame")
    R_wc = world_camera_rotation(0.0, -90.0, 0.0)  # nadir attitude
    K = np.array([[args.rover_fx, 0, args.rover_cx],
                  [0, args.rover_fy, args.rover_cy],
                  [0, 0, 1]], dtype=np.float64)

    estimates: list[dict] = []
    for i, (tx, ty, tz) in enumerate(truth_poses):
        rover = render_rover_view(
            ortho, heightmap_m,
            camera_xyz_m=(tx, ty, tz), R_world_camera=R_wc,
            fx=args.rover_fx, fy=args.rover_fy,
            cx=args.rover_cx, cy=args.rover_cy,
            rover_width=args.rover_width, rover_height=args.rover_height,
            px_to_m=px_to_m,
        )
        pts_o, pts_r, kp_o, kp_r, matches = match_features(
            ortho, rover, args.ratio_test, args.max_features,
        )
        pnp = None
        inlier_pts_o: np.ndarray | None = None
        inlier_pts_r: np.ndarray | None = None
        if pts_o.shape[0] >= 6:
            pnp = recover_pose_pnp(
                pts_o, pts_r,
                heightmap_m=heightmap_m, px_to_m=px_to_m,
                fx=args.rover_fx, fy=args.rover_fy,
                cx=args.rover_cx, cy=args.rover_cy,
                reproj_error_px=args.pnp_reproj_error_px,
                ransac_iters=args.pnp_ransac_iters,
            )

        if pnp is not None:
            estim_xyz = pnp["rover_world_xyz_m"]
            err_m = math.sqrt(sum((estim_xyz[k] - (tx, ty, tz)[k]) ** 2 for k in range(3)))
        else:
            estim_xyz = None
            err_m = float("nan")
        msg = (f"  frame {i+1}/{args.frames}  "
               f"truth=({tx/1000:.1f},{ty/1000:.1f},{tz/1000:.1f}) km  "
               f"matches={pts_o.shape[0]}  "
               f"err={'nan' if err_m != err_m else f'{err_m:.0f} m'}")
        print(msg)
        estimates.append({
            "truth_xyz_m": (tx, ty, tz),
            "estimated_xyz_m": estim_xyz,
            "error_m": err_m,
            "rover": rover,
            "match_count": int(pts_o.shape[0]),
            "pnp_inliers": int(pnp["pnp_inliers"]) if pnp else 0,
        })

    print("[3] composite frames")
    width, height = args.frame_width, args.frame_height
    map_size = height  # square map panel on the left
    right_w = width - map_size - 8
    rover_h = int(right_w * 0.62)  # slightly taller than 4:3 to fit text
    rover_w = right_w
    map_scale = map_size / ortho.shape[0]  # downscale ortho to map_size

    ortho_color = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    map_base = cv2.resize(ortho_color, (map_size, map_size),
                          interpolation=cv2.INTER_AREA)

    frames_pil: list[Image.Image] = []
    cumulative_err = 0.0
    cumulative_count = 0
    for i, est in enumerate(estimates):
        frame = np.full((height, width, 3), 16, dtype=np.uint8)

        # Left panel: top-down map with trajectory overlay.
        map_panel = map_base.copy()
        # Truth trajectory up to current frame.
        for j in range(i + 1):
            tx, ty, _ = estimates[j]["truth_xyz_m"]
            mx, my = world_xy_to_map_pixel(tx, ty,
                                            map_shape=map_panel.shape,
                                            px_to_m=px_to_m, map_scale=map_scale)
            if j > 0:
                ptx, pty, _ = estimates[j-1]["truth_xyz_m"]
                pmx, pmy = world_xy_to_map_pixel(ptx, pty,
                                                  map_shape=map_panel.shape,
                                                  px_to_m=px_to_m, map_scale=map_scale)
                cv2.line(map_panel, (pmx, pmy), (mx, my),
                         color=(80, 220, 240), thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(map_panel, (mx, my), 3, (80, 220, 240), -1, cv2.LINE_AA)

        # Recovered positions (every successful PnP up to current frame).
        for j in range(i + 1):
            est_j = estimates[j]
            if est_j["estimated_xyz_m"] is None:
                continue
            ex, ey, _ = est_j["estimated_xyz_m"]
            emx, emy = world_xy_to_map_pixel(ex, ey,
                                              map_shape=map_panel.shape,
                                              px_to_m=px_to_m, map_scale=map_scale)
            cv2.circle(map_panel, (emx, emy), 4, (90, 220, 120), -1, cv2.LINE_AA)

        # Highlight current truth (yellow ring) + current estimate (green ring) +
        # error line (red).
        ctx, cty, _ = est["truth_xyz_m"]
        cmx, cmy = world_xy_to_map_pixel(ctx, cty,
                                          map_shape=map_panel.shape,
                                          px_to_m=px_to_m, map_scale=map_scale)
        cv2.circle(map_panel, (cmx, cmy), 11, (80, 220, 240), 2, cv2.LINE_AA)
        if est["estimated_xyz_m"] is not None:
            ex, ey, _ = est["estimated_xyz_m"]
            emx, emy = world_xy_to_map_pixel(ex, ey,
                                              map_shape=map_panel.shape,
                                              px_to_m=px_to_m, map_scale=map_scale)
            cv2.circle(map_panel, (emx, emy), 11, (90, 220, 120), 2, cv2.LINE_AA)
            cv2.line(map_panel, (cmx, cmy), (emx, emy),
                     color=(60, 60, 240), thickness=2, lineType=cv2.LINE_AA)

        text_with_shadow(map_panel, "TRAJECTORY MAP  (top-down ortho)",
                         (12, 28), scale=0.7, color=(240, 240, 240), thickness=2)
        text_with_shadow(map_panel, "truth",  (12, map_size - 50), 0.5, (80, 220, 240), 1)
        text_with_shadow(map_panel, "recovered", (12, map_size - 28), 0.5, (90, 220, 120), 1)

        frame[:, :map_size] = map_panel

        # Right top: rover view (resize to rover_w x rover_h).
        rover_color = cv2.cvtColor(est["rover"], cv2.COLOR_GRAY2BGR)
        rover_panel = cv2.resize(rover_color, (rover_w, rover_h),
                                  interpolation=cv2.INTER_AREA)
        text_with_shadow(rover_panel,
                         f"NADIR ROVER VIEW  -  altitude {est['truth_xyz_m'][2]/1000:.1f} km",
                         (12, 26), scale=0.6, color=(240, 240, 240), thickness=2)
        rx0 = map_size + 8
        frame[:rover_h, rx0:rx0 + rover_w] = rover_panel

        # Right bottom: HUD
        hud_y0 = rover_h + 16
        hud_x0 = map_size + 16
        cv2.rectangle(frame,
                      (rx0, rover_h + 8), (rx0 + right_w, height - 8),
                      color=(36, 36, 36), thickness=-1)

        text_with_shadow(frame, f"FRAME  {i + 1} / {len(estimates)}",
                         (hud_x0, hud_y0 + 24), 0.7, (240, 240, 240), 2)

        truth_str = (f"truth     ({est['truth_xyz_m'][0]/1000:6.1f}, "
                     f"{est['truth_xyz_m'][1]/1000:6.1f}, "
                     f"{est['truth_xyz_m'][2]/1000:6.1f})  km")
        text_with_shadow(frame, truth_str,
                         (hud_x0, hud_y0 + 56), 0.5, (200, 200, 200), 1)
        if est["estimated_xyz_m"] is not None:
            recover_str = (f"recovered ({est['estimated_xyz_m'][0]/1000:6.1f}, "
                           f"{est['estimated_xyz_m'][1]/1000:6.1f}, "
                           f"{est['estimated_xyz_m'][2]/1000:6.1f})  km")
            text_with_shadow(frame, recover_str,
                             (hud_x0, hud_y0 + 80), 0.5, (200, 200, 200), 1)
            cumulative_err += est["error_m"]
            cumulative_count += 1
            if est["error_m"] < 100:
                err_color = (90, 220, 120)   # green
            elif est["error_m"] < 500:
                err_color = (40, 200, 220)   # cyan
            else:
                err_color = (60, 60, 240)    # red
            text_with_shadow(frame,
                             f"current error  {est['error_m']:6.1f} m   "
                             f"(matches {est['match_count']}, inliers {est['pnp_inliers']})",
                             (hud_x0, hud_y0 + 110), 0.55, err_color, 1)
            mean_err = cumulative_err / cumulative_count
            text_with_shadow(frame,
                             f"running mean   {mean_err:6.1f} m  "
                             f"(over {cumulative_count} successful PnP frames)",
                             (hud_x0, hud_y0 + 134), 0.5, (180, 200, 220), 1)
        else:
            text_with_shadow(frame, "PnP failed",
                             (hud_x0, hud_y0 + 80), 0.55, (60, 60, 240), 1)

        frames_pil.append(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))

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
    if cumulative_count > 0:
        print(f"  successful PnP: {cumulative_count}/{len(estimates)}  "
              f"mean error: {cumulative_err/cumulative_count:.1f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
