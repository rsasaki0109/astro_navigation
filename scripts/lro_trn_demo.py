#!/usr/bin/env python3
"""Real-LRO TRN bench: orbital descent on actual lunar imagery + LOLA elevation.

The synthetic demo (`scripts/synthetic_trn_demo.py`) validates the heightmap
forward render + AP3P PnP pipeline on procedural craters. This script swaps
both inputs for real LRO data and reuses the same pipeline:

  - Ortho intensity:  LROC WAC global mosaic, fetched as 256x256 grayscale
                      JPEG tiles from NASA Trek WMTS at the requested zoom.
  - Heightmap:        LOLA LDEM_<ppd>.img (PDS3 raw int16, 0.5 m / DN scaling
                      relative to a 1737.4 km lunar radius), cropped to the
                      WAC mosaic bbox and bilinear-resampled to its grid.

A virtual "rover" camera is placed at a known (X, Y, Z) above the surface and
its view rendered by per-pixel ray-march against the real heightmap. The
pipeline then runs SIFT + AP3P PnP exactly as the synthetic demo, and reports
the position-recovery error vs the truth pose.

Defaults target Apollo 11 (Mare Tranquillitatis, lat=0.67, lon=23.47) at
70 km altitude — orbital-descent scale that matches the LROC WAC ~660 m/px
sample distance at zoom 5.

Why a separate script (vs flags on synthetic_trn_demo): the data plumbing
(Trek tile URL stitching, PDS3 IMG reading, lat/lon -> local metres
conversion) is its own slug of code with its own failure modes. Keeping it
out of the synthetic demo lets that file stay small and focused on the
algorithm itself.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from synthetic_trn_demo import (  # noqa: E402
    match_features,
    recover_pose_pnp,
    render_rover_view,
    world_camera_rotation,
)


LUNAR_RADIUS_M = 1_737_400.0


# Curated landmarks. Reuse `lat_deg, lon_deg` (lon east-positive; LDEM uses
# 0..360 E, NASA Trek uses -180..180; both are handled by the math below).
TARGETS: dict[str, tuple[float, float]] = {
    "apollo11":     (  0.674,   23.473),  # Mare Tranquillitatis
    "apollo12":     ( -3.012,  -23.421),  # Oceanus Procellarum
    "apollo15":     ( 26.132,    3.633),  # Hadley Rille
    "apollo17":     ( 20.190,   30.770),  # Taurus-Littrow
    "tycho":        (-43.310,  -11.360),  # bright crater, sharp rim
    "shackleton":   (-89.660,    0.000),  # south pole crater
    "copernicus":   (  9.620,  -20.080),  # iconic ray crater
}


WAC_TILE_URL = (
    "https://trek.nasa.gov/tiles/Moon/EQ/LRO_WAC_Mosaic_Global_303ppd_v02/"
    "1.0.0//default/default028mm/{zoom}/{row}/{col}.jpg"
)

LDEM_URL = (
    "https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/"
    "lrolol_1xxx/data/lola_gdr/cylindrical/img/ldem_{ppd}.img"
)


def _download_with_progress(url: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        return
    print(f"download: {url} -> {destination}")
    start = time.time()
    urllib.request.urlretrieve(url, destination)
    print(f"  done in {time.time() - start:.1f} s ({destination.stat().st_size/1e6:.1f} MB)")


def fetch_wac_mosaic(
    target_lat: float, target_lon: float, *,
    zoom: int, tile_radius: int, cache_dir: Path,
) -> tuple[np.ndarray, float, float, float, float]:
    """Download a (2*tile_radius+1)^2 patch of WAC tiles around the target.

    Returns (mosaic_uint8, lat_min, lat_max, lon_min, lon_max). Lon is in
    [-180, 180] east-positive so it matches Trek's tile addressing.
    """
    n_horizontal = 2 ** (zoom + 1)
    n_vertical = 2 ** zoom
    deg_per_tile_lon = 360.0 / n_horizontal
    deg_per_tile_lat = 180.0 / n_vertical

    # Trek uses lon in [-180, 180]. Wrap if user gave 0..360.
    target_lon_wrapped = ((target_lon + 180.0) % 360.0) - 180.0
    target_col = int((target_lon_wrapped + 180.0) / deg_per_tile_lon)
    target_row = int((90.0 - target_lat) / deg_per_tile_lat)

    cols = list(range(target_col - tile_radius, target_col + tile_radius + 1))
    rows = list(range(target_row - tile_radius, target_row + tile_radius + 1))

    tile_size = 256
    mosaic = np.zeros((len(rows) * tile_size, len(cols) * tile_size), dtype=np.uint8)

    for ti, row in enumerate(rows):
        for tj, col in enumerate(cols):
            # Wrap longitude (lat is clamped — polar tiles will fail, but we
            # don't expect to demo at the poles with this defaults set).
            col_wrapped = col % n_horizontal
            tile_path = cache_dir / f"wac_z{zoom}_r{row}_c{col_wrapped}.jpg"
            url = WAC_TILE_URL.format(zoom=zoom, row=row, col=col_wrapped)
            _download_with_progress(url, tile_path)
            tile = cv2.imread(str(tile_path), cv2.IMREAD_GRAYSCALE)
            if tile is None:
                raise SystemExit(f"could not decode WAC tile: {tile_path}")
            if tile.shape != (tile_size, tile_size):
                raise SystemExit(
                    f"unexpected tile shape {tile.shape} in {tile_path}"
                )
            mosaic[ti*tile_size:(ti+1)*tile_size,
                   tj*tile_size:(tj+1)*tile_size] = tile

    lat_max = 90.0 - rows[0] * deg_per_tile_lat
    lat_min = 90.0 - (rows[-1] + 1) * deg_per_tile_lat
    lon_min = -180.0 + cols[0] * deg_per_tile_lon
    lon_max = -180.0 + (cols[-1] + 1) * deg_per_tile_lon
    return mosaic, lat_min, lat_max, lon_min, lon_max


def fetch_ldem(ppd: int, cache_dir: Path) -> np.ndarray:
    """Download (if missing) and load the global LOLA LDEM at the chosen ppd.

    Returns a 2D float32 array of elevation in metres, shape (180*ppd,
    360*ppd). Row 0 is +90 deg latitude, column 0 is 0 deg east longitude
    (PDS convention). Conversion: HEIGHT_M = DN * 0.5.
    """
    if ppd not in (4, 16, 64):
        raise SystemExit(f"unsupported LDEM ppd={ppd} (use 4, 16, or 64)")
    img_path = cache_dir / f"ldem_{ppd}.img"
    _download_with_progress(LDEM_URL.format(ppd=ppd), img_path)
    n_lat = 180 * ppd
    n_lon = 360 * ppd
    dn = np.fromfile(img_path, dtype="<i2")
    if dn.size != n_lat * n_lon:
        raise SystemExit(
            f"LDEM_{ppd} expected {n_lat*n_lon} samples, got {dn.size}"
        )
    return dn.reshape(n_lat, n_lon).astype(np.float32) * 0.5


def crop_ldem_to_wac(
    ldem: np.ndarray, ppd: int,
    lat_min: float, lat_max: float, lon_min: float, lon_max: float,
) -> np.ndarray:
    """Crop the global LDEM to the WAC bbox. Wraps longitude across the
    PDS dateline (LDEM is 0..360 E, WAC bbox is -180..180 E).
    """
    n_lon = 360 * ppd
    n_lat = 180 * ppd
    row_top = max(0, int(round((90.0 - lat_max) * ppd)))
    row_bot = min(n_lat, int(round((90.0 - lat_min) * ppd)))
    # LDEM lon: 0..360. WAC bbox lon: -180..180. Convert WAC -> LDEM by
    # adding 360 to negative values.
    lon_min_pds = lon_min if lon_min >= 0 else lon_min + 360.0
    lon_max_pds = lon_max if lon_max >= 0 else lon_max + 360.0
    col_left = int(round(lon_min_pds * ppd))
    col_right = int(round(lon_max_pds * ppd))
    if col_left <= col_right:
        return ldem[row_top:row_bot, col_left:col_right]
    # Wrap: bbox spans the dateline (lon_min > 0 but lon_max wraps past 360).
    left = ldem[row_top:row_bot, col_left:n_lon]
    right = ldem[row_top:row_bot, 0:col_right]
    return np.concatenate([left, right], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target", default="apollo11", choices=sorted(TARGETS.keys()))
    parser.add_argument("--cache-dir", type=Path,
                        default=REPO_ROOT / "datasets" / "lro_cache")
    parser.add_argument("--zoom", type=int, default=5,
                        help="Trek WAC tile zoom level. z=5 ~660 m/px (default), "
                        "z=6 ~330 m/px, z=7 ~165 m/px. Each step quadruples download size.")
    parser.add_argument("--tile-radius", type=int, default=1,
                        help="Half-extent in tiles around the target. radius=1 -> 3x3 "
                        "tiles -> 768x768 px mosaic (~500 km wide at z=5).")
    parser.add_argument("--ldem-ppd", type=int, default=4, choices=[4, 16, 64],
                        help="LOLA LDEM resolution. 4 ppd ~7.6 km/px (~2 MB, default), "
                        "16 ppd ~1.9 km/px (~33 MB), 64 ppd ~470 m/px (~530 MB). LDEM_4 is "
                        "much coarser than the WAC ortho — heightmap appears almost flat per "
                        "rover pixel — but lets the demo run on a 2-second download. Use "
                        "--ldem-ppd 64 for an honest heightmap-driven render at z=5.")
    parser.add_argument("--rover-altitude-m", type=float, default=400_000.0,
                        help="Camera height above the local mean. Default 400 km gives a rover-view "
                        "sample distance (~770 m/px) that's within 2x of the WAC ortho at z=5 "
                        "(~660 m/px), so SIFT scale-space matches reliably. Going lower starves the "
                        "matcher of resolved features (sweep on Tycho z=5: 35 km -> matches=52 PnP "
                        "fail; 70 km -> 77 / fail; 200 km -> 86 / 12 inliers / 98 m err; 400 km -> "
                        "113 / 24 / 179 m err). 6-target sweep at 400 km / z=5: all six recover with "
                        "errors 179-1383 m on a ~500 km mosaic (apollo11/12 in featureless mare are "
                        "the harder cases at ~300-1400 m; Tycho is the easiest at 179 m).")
    parser.add_argument("--rover-x-frac", type=float, default=0.5,
                        help="Rover (X) position as a fraction of mosaic width.")
    parser.add_argument("--rover-y-frac", type=float, default=0.5)
    parser.add_argument("--rover-width", type=int, default=640)
    parser.add_argument("--rover-height", type=int, default=480)
    parser.add_argument("--rover-fx", type=float, default=520.0)
    parser.add_argument("--rover-fy", type=float, default=520.0)
    parser.add_argument("--rover-cx", type=float, default=320.0)
    parser.add_argument("--rover-cy", type=float, default=240.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=-90.0,
                        help="Default -90 = nadir. Tilted views are out of envelope on the "
                        "synthetic demo and likely to be even harder on real WAC.")
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--ratio-test", type=float, default=0.85)
    parser.add_argument("--max-features", type=int, default=2000)
    parser.add_argument("--pnp-reproj-error-px", type=float, default=5.0)
    parser.add_argument("--pnp-ransac-iters", type=int, default=200_000)
    parser.add_argument("--ray-march-iters", type=int, default=6)
    args = parser.parse_args()

    args.cache_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    target_lat, target_lon = TARGETS[args.target]
    print(f"target: {args.target} (lat={target_lat:+.3f} lon={target_lon:+.3f})")

    print("\n[1/4] fetch WAC mosaic")
    ortho, lat_min, lat_max, lon_min, lon_max = fetch_wac_mosaic(
        target_lat, target_lon,
        zoom=args.zoom, tile_radius=args.tile_radius, cache_dir=args.cache_dir,
    )
    print(f"  mosaic shape: {ortho.shape}  bbox: lat ({lat_min:+.3f}, {lat_max:+.3f}) "
          f"lon ({lon_min:+.3f}, {lon_max:+.3f})")

    # Local-tangent metres-per-pixel. Use the WAC bbox geometry plus lunar
    # radius. For small bboxes the small-angle approximation is fine.
    deg_per_px_lat = (lat_max - lat_min) / ortho.shape[0]
    deg_per_px_lon = (lon_max - lon_min) / ortho.shape[1]
    lat_origin = (lat_min + lat_max) * 0.5
    px_to_m_lat = math.radians(deg_per_px_lat) * LUNAR_RADIUS_M
    px_to_m_lon = math.radians(deg_per_px_lon) * LUNAR_RADIUS_M * math.cos(math.radians(lat_origin))
    px_to_m = (px_to_m_lat + px_to_m_lon) * 0.5
    print(f"  px_to_m: lat {px_to_m_lat:.1f}, lon {px_to_m_lon:.1f}, mean {px_to_m:.1f}")

    print("\n[2/4] fetch LOLA LDEM")
    ldem = fetch_ldem(args.ldem_ppd, args.cache_dir)
    dem_cropped = crop_ldem_to_wac(ldem, args.ldem_ppd,
                                   lat_min, lat_max, lon_min, lon_max)
    print(f"  cropped DEM shape: {dem_cropped.shape}  range: "
          f"{dem_cropped.min():.1f} -> {dem_cropped.max():.1f} m")
    heightmap_full = cv2.resize(dem_cropped, (ortho.shape[1], ortho.shape[0]),
                                interpolation=cv2.INTER_LINEAR)
    heightmap_m = heightmap_full - heightmap_full.mean()
    print(f"  resampled heightmap: relief {heightmap_m.max() - heightmap_m.min():.1f} m "
          f"(std {heightmap_m.std():.1f} m)")

    print("\n[3/4] render rover view")
    rover_x_m = args.rover_x_frac * ortho.shape[1] * px_to_m
    rover_y_m = args.rover_y_frac * ortho.shape[0] * px_to_m
    truth_xyz = (rover_x_m, rover_y_m, args.rover_altitude_m)
    R_wc = world_camera_rotation(args.yaw_deg, args.pitch_deg, args.roll_deg)
    cv2.imwrite(str(args.output_dir / "ortho.png"), ortho)
    rover = render_rover_view(
        ortho, heightmap_m,
        camera_xyz_m=truth_xyz, R_world_camera=R_wc,
        fx=args.rover_fx, fy=args.rover_fy, cx=args.rover_cx, cy=args.rover_cy,
        rover_width=args.rover_width, rover_height=args.rover_height,
        px_to_m=px_to_m, max_iters=args.ray_march_iters,
    )
    cv2.imwrite(str(args.output_dir / "rover.png"), rover)

    print("\n[4/4] match + PnP")
    pts_o, pts_r, kp_o, kp_r, matches = match_features(
        ortho, rover, args.ratio_test, args.max_features,
    )
    if pts_o.shape[0] < 6:
        sys.stderr.write(f"too few feature matches: {pts_o.shape[0]}\n")
        return 1
    pnp = recover_pose_pnp(
        pts_o, pts_r,
        heightmap_m=heightmap_m, px_to_m=px_to_m,
        fx=args.rover_fx, fy=args.rover_fy, cx=args.rover_cx, cy=args.rover_cy,
        reproj_error_px=args.pnp_reproj_error_px,
        ransac_iters=args.pnp_ransac_iters,
        plausible_xy_margin_factor=0.5,
    )
    if pnp is None:
        position_error_m = float("nan")
        estimated_xyz = None
    else:
        estimated_xyz = pnp["rover_world_xyz_m"]
        position_error_m = math.sqrt(sum(
            (estimated_xyz[i] - truth_xyz[i]) ** 2 for i in range(3)
        ))

    visual = cv2.drawMatches(
        ortho, kp_o, rover, kp_r, matches[:80], None,
        matchColor=(80, 220, 80), singlePointColor=(80, 80, 220),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    cv2.imwrite(str(args.output_dir / "matches.png"), visual)

    metadata = {
        "target": args.target,
        "target_lat_lon_deg": [target_lat, target_lon],
        "wac": {
            "zoom": args.zoom, "tile_radius": args.tile_radius,
            "mosaic_shape": list(ortho.shape),
            "bbox_lat": [lat_min, lat_max], "bbox_lon": [lon_min, lon_max],
            "px_to_m": px_to_m,
        },
        "ldem": {
            "ppd": args.ldem_ppd,
            "cropped_shape": list(dem_cropped.shape),
            "raw_elevation_range_m": [float(dem_cropped.min()), float(dem_cropped.max())],
            "relief_after_resample_m": float(heightmap_m.max() - heightmap_m.min()),
        },
        "rover_intrinsics": {
            "fx": args.rover_fx, "fy": args.rover_fy,
            "cx": args.rover_cx, "cy": args.rover_cy,
            "width": args.rover_width, "height": args.rover_height,
        },
        "rover_truth": {
            "xyz_m": list(truth_xyz),
            "yaw_deg": args.yaw_deg, "pitch_deg": args.pitch_deg, "roll_deg": args.roll_deg,
        },
        "rover_estimated_xyz_m": estimated_xyz,
        "position_error_m": position_error_m,
        "match_count": int(pts_o.shape[0]),
        "pnp": pnp,
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print()
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
