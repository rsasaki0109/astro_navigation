#!/usr/bin/env python3
"""Render a dynamic hazard replanning lunar navigation GIF.

The demo starts with the Tycho terminal TRN cost map, plans an initial route
with the C++ hazard planner, then injects a new blocked hazard on the planned
route. The rover invalidates the old route, replans from its current position,
and continues along the new route.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from render_hazard_aware_navigation_demo import (
    REPO_ROOT,
    GridPoint,
    build_hazard_cost,
    grid_to_pixel,
    local_to_pixel,
    nearest_low_cost,
    pixel_to_local,
    plan_with_cpp_cli,
    resample_path,
    text,
)


def draw_polyline(image: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int], thickness: int) -> None:
    if len(points) >= 2:
        cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)


def add_dynamic_hazard(
    *,
    cost: np.ndarray,
    hazard: np.ndarray,
    center: GridPoint,
    radius_cells: int,
    blocked_cost: float,
) -> tuple[np.ndarray, np.ndarray, list[GridPoint]]:
    updated_cost = cost.copy()
    updated_hazard = hazard.copy()
    blocked_cells: list[GridPoint] = []
    height, width = cost.shape
    for y in range(max(0, center.y - radius_cells), min(height, center.y + radius_cells + 1)):
        for x in range(max(0, center.x - radius_cells), min(width, center.x + radius_cells + 1)):
            distance = math.hypot(x - center.x, y - center.y)
            if distance > radius_cells:
                continue
            blocked_cells.append(GridPoint(x, y))
            updated_cost[y, x] = blocked_cost
            updated_hazard[y, x] = 1.0
    return updated_cost, updated_hazard, blocked_cells


def render_frame(
    *,
    ortho: np.ndarray,
    base_hazard: np.ndarray,
    dynamic_hazard: np.ndarray,
    old_path_pixels: list[tuple[int, int]],
    new_path_pixels: list[tuple[int, int]],
    progress_pixels: list[tuple[int, int]],
    rover_pixel: tuple[int, int],
    waypoint_pixel: tuple[int, int],
    hazard_pixel: tuple[int, int],
    sigma_m: float,
    px_to_m: float,
    phase: str,
    nav_status: str,
    distance_m: float,
    old_metrics: dict[str, float | None],
    new_metrics: dict[str, float | None],
    replan_count: int,
    output_size: tuple[int, int],
) -> np.ndarray:
    width, height = output_size
    map_w = 880
    side_w = width - map_w

    base = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    base_hazard_full = cv2.resize(base_hazard, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_CUBIC)
    dynamic_full = cv2.resize(dynamic_hazard, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_NEAREST)
    hazard_visible = bool(np.max(dynamic_hazard) > 0.0)

    overlay = base.copy()
    overlay[base_hazard_full > 0.57] = (45, 40, 175)
    overlay[dynamic_full > 0.5] = (20, 40, 245)
    base = cv2.addWeighted(overlay, 0.38, base, 0.62, 0.0)

    scale_x = map_w / ortho.shape[1]
    scale_y = height / ortho.shape[0]
    map_panel = cv2.resize(base, (map_w, height), interpolation=cv2.INTER_AREA)

    def scale_point(point: tuple[int, int]) -> tuple[int, int]:
        return int(round(point[0] * scale_x)), int(round(point[1] * scale_y))

    old_scaled = [scale_point(point) for point in old_path_pixels]
    new_scaled = [scale_point(point) for point in new_path_pixels]
    progress_scaled = [scale_point(point) for point in progress_pixels]
    rover_scaled = scale_point(rover_pixel)
    waypoint_scaled = scale_point(waypoint_pixel)
    hazard_scaled = scale_point(hazard_pixel)
    sigma_px = int(round(sigma_m / px_to_m * (scale_x + scale_y) * 0.5))

    draw_polyline(map_panel, old_scaled, (170, 165, 145), 3)
    if new_scaled:
        draw_polyline(map_panel, new_scaled, (65, 210, 255), 4)
    draw_polyline(map_panel, progress_scaled, (95, 245, 145), 5)

    cv2.circle(map_panel, waypoint_scaled, 15, (65, 230, 255), 3, cv2.LINE_AA)
    cv2.circle(map_panel, waypoint_scaled, 5, (65, 230, 255), -1, cv2.LINE_AA)
    if hazard_visible:
        cv2.circle(map_panel, hazard_scaled, 18, (20, 40, 245), 3, cv2.LINE_AA)
        cv2.line(
            map_panel,
            (hazard_scaled[0] - 13, hazard_scaled[1] - 13),
            (hazard_scaled[0] + 13, hazard_scaled[1] + 13),
            (20, 40, 245),
            3,
            cv2.LINE_AA,
        )
        cv2.line(
            map_panel,
            (hazard_scaled[0] + 13, hazard_scaled[1] - 13),
            (hazard_scaled[0] - 13, hazard_scaled[1] + 13),
            (20, 40, 245),
            3,
            cv2.LINE_AA,
        )
    cv2.circle(map_panel, rover_scaled, max(sigma_px, 1), (75, 205, 255), 2, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 13, (80, 255, 150), -1, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 16, (15, 30, 20), 2, cv2.LINE_AA)

    text(map_panel, "DYNAMIC HAZARD REPLANNING", (24, 42), scale=0.78, color=(245, 246, 242), thickness=2)
    text(map_panel, "gray = old route   yellow = replanned route   red X = new blocked hazard", (24, 76), scale=0.47, color=(220, 225, 220))
    text(map_panel, "WAYPOINT", (waypoint_scaled[0] + 18, waypoint_scaled[1] - 12), scale=0.5, color=(65, 230, 255))
    if hazard_visible:
        text(map_panel, "NEW HAZARD", (hazard_scaled[0] + 20, hazard_scaled[1] - 16), scale=0.48, color=(40, 90, 255))

    side = np.full((height, side_w, 3), (22, 25, 30), dtype=np.uint8)
    x0 = 28
    text(side, "LUNAR", (x0, 52), scale=0.9, color=(245, 245, 240), thickness=2)
    text(side, "AUTOPILOT", (x0, 90), scale=0.9, color=(245, 245, 240), thickness=2)
    text(side, phase, (x0, 138), scale=0.58, color=(95, 220, 255), thickness=2)

    old_detour = old_metrics.get("detour_ratio")
    new_detour = new_metrics.get("detour_ratio") if replan_count > 0 else None
    clearance_m = new_metrics.get("min_clearance_m") if replan_count > 0 else None
    rows = [
        ("NAV", nav_status),
        ("REPLANS", str(replan_count)),
        ("DISTANCE", f"{distance_m:6.0f} m"),
        ("OLD DETOUR", "--" if old_detour is None else f"{old_detour:4.2f} x"),
        ("NEW DETOUR", "--" if new_detour is None else f"{new_detour:4.2f} x"),
        ("CLEARANCE", "--" if clearance_m is None else f"{clearance_m:6.0f} m"),
    ]
    y = 195
    for label, value in rows:
        text(side, label, (x0, y), scale=0.43, color=(125, 175, 220))
        text(side, value, (x0, y + 27), scale=0.53, color=(236, 238, 236))
        y += 55

    stages = ["TRN LOCK", "ROUTE LOCK", "HAZARD", "REPLAN", "NEW ROUTE", "ARRIVED"]
    active_count = {
        "GUIDANCE ACTIVE": 2,
        "ROUTE INVALID": 3,
        "REPLANNING": 4,
        "NEW ROUTE LOCK": 5,
        "ARRIVED": 6,
    }.get(phase, 1)
    y_stage = height - 150
    for idx, label in enumerate(stages):
        active = idx < active_count
        color = (92, 215, 125) if active else (70, 78, 88)
        if label == "HAZARD" and active:
            color = (55, 95, 255)
        cy = y_stage + idx * 23
        cv2.circle(side, (x0 + 8, cy - 5), 5, color, -1, cv2.LINE_AA)
        text(side, label, (x0 + 25, cy), scale=0.36, color=(215, 225, 220) if active else (130, 140, 145))

    frame = np.full((height, width, 3), (10, 12, 15), dtype=np.uint8)
    frame[:, :map_w] = map_panel
    frame[:, map_w:] = side
    return frame


def resolve_planner_app(path: Path | None) -> Path:
    candidates = []
    if path is not None:
        candidates.append(path)
    candidates.extend(
        [
            REPO_ROOT / "build" / "apps" / "hazard_route_demo",
            Path("/tmp/astro_navigation-build/apps/hazard_route_demo"),
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit("hazard_route_demo not found; build it or pass --planner-app")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trn-fixture-dir", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_lro_tycho_terminal")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "dynamic_hazard_replanning_demo.gif")
    parser.add_argument("--planner-app", type=Path)
    parser.add_argument("--grid-size", type=int, default=170)
    parser.add_argument("--frames", type=int, default=58)
    parser.add_argument("--dynamic-hazard-radius", type=int, default=9)
    args = parser.parse_args()

    planner_app = resolve_planner_app(args.planner_app)
    summary_path = args.trn_fixture_dir / "summary.json"
    ortho_path = args.trn_fixture_dir / "ortho.png"
    trn = json.loads(summary_path.read_text(encoding="utf-8"))
    ortho = cv2.imread(str(ortho_path), cv2.IMREAD_GRAYSCALE)
    if ortho is None:
        raise SystemExit(f"failed to read {ortho_path}")

    px_to_m = float(trn["wac"]["px_to_m"])
    start_pixel = local_to_pixel(
        float(trn["rover_estimated_xyz_m"][0]),
        float(trn["rover_estimated_xyz_m"][1]),
        px_to_m,
        ortho.shape,
    )
    waypoint_pixel = (900, 430)

    hazard, cost = build_hazard_cost(ortho, args.grid_size)
    scale = args.grid_size / ortho.shape[0]
    start_grid = nearest_low_cost(cost, GridPoint(int(start_pixel[0] * scale), int(start_pixel[1] * scale)))
    goal_grid = nearest_low_cost(cost, GridPoint(int(waypoint_pixel[0] * scale), int(waypoint_pixel[1] * scale)))
    grid_resolution_m = px_to_m * ortho.shape[0] / args.grid_size
    blocked_cost = 8.5

    with tempfile.TemporaryDirectory(prefix="astro_dynamic_replan_") as temp_dir:
        workdir = Path(temp_dir)
        old_route, old_metrics = plan_with_cpp_cli(
            planner_app=planner_app,
            cost=cost,
            start=start_grid,
            goal=goal_grid,
            workdir=workdir / "old_route",
            grid_resolution_m=grid_resolution_m,
            blocked_cost=blocked_cost,
        )

        trigger_index = max(3, int(len(old_route) * 0.42))
        blocked_index = min(len(old_route) - 3, int(len(old_route) * 0.62))
        replan_start = old_route[trigger_index]
        blocked_route_cell = old_route[blocked_index]
        dynamic_cost, dynamic_hazard, blocked_cells = add_dynamic_hazard(
            cost=cost,
            hazard=np.zeros_like(hazard),
            center=blocked_route_cell,
            radius_cells=args.dynamic_hazard_radius,
            blocked_cost=blocked_cost,
        )
        combined_hazard = np.maximum(hazard, dynamic_hazard)

        new_route, new_metrics = plan_with_cpp_cli(
            planner_app=planner_app,
            cost=dynamic_cost,
            start=replan_start,
            goal=goal_grid,
            workdir=workdir / "new_route",
            grid_resolution_m=grid_resolution_m,
            blocked_cost=blocked_cost,
        )

    prefix = old_route[: trigger_index + 1]
    final_route = prefix + new_route[1:]
    old_path_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in old_route]
    new_path_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in new_route]
    final_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in final_route]
    rover_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in resample_path(final_route, args.frames)]
    hazard_pixel = grid_to_pixel(blocked_route_cell, ortho.shape, args.grid_size)
    sigma_m = max(2.0 * px_to_m, px_to_m * math.sqrt(12.0 / max(int(trn["pnp"]["pnp_inliers"]), 1)))

    replan_frame = max(8, int(args.frames * 0.42))
    frames: list[Image.Image] = []
    for idx, rover_pixel in enumerate(rover_pixels):
        distance_m = math.hypot(rover_pixel[0] - waypoint_pixel[0], rover_pixel[1] - waypoint_pixel[1]) * px_to_m
        if idx < replan_frame - 2:
            phase = "GUIDANCE ACTIVE"
            nav_status = "OK / NONE"
            visible_new_path: list[tuple[int, int]] = []
            active_hazard = np.zeros_like(dynamic_hazard)
            replan_count = 0
        elif idx < replan_frame + 2:
            phase = "ROUTE INVALID"
            nav_status = "RELOCALIZING"
            visible_new_path = []
            active_hazard = dynamic_hazard
            replan_count = 0
        elif idx < replan_frame + 6:
            phase = "REPLANNING"
            nav_status = "DEGRADED / POSITION_ONLY"
            visible_new_path = new_path_pixels
            active_hazard = dynamic_hazard
            replan_count = 1
        elif idx >= len(rover_pixels) - 4:
            phase = "ARRIVED"
            nav_status = "OK / NONE"
            visible_new_path = new_path_pixels
            active_hazard = dynamic_hazard
            replan_count = 1
            distance_m = 0.0
        else:
            phase = "NEW ROUTE LOCK"
            nav_status = "OK / NONE"
            visible_new_path = new_path_pixels
            active_hazard = dynamic_hazard
            replan_count = 1

        progress_count = max(1, int(round((idx + 1) / args.frames * len(final_pixels))))
        frame = render_frame(
            ortho=ortho,
            base_hazard=hazard,
            dynamic_hazard=active_hazard,
            old_path_pixels=old_path_pixels,
            new_path_pixels=visible_new_path,
            progress_pixels=final_pixels[:progress_count],
            rover_pixel=rover_pixel,
            waypoint_pixel=waypoint_pixel,
            hazard_pixel=hazard_pixel,
            sigma_m=sigma_m,
            px_to_m=px_to_m,
            phase=phase,
            nav_status=nav_status,
            distance_m=distance_m,
            old_metrics=old_metrics,
            new_metrics=new_metrics,
            replan_count=replan_count,
            output_size=(1280, 720),
        )
        frames.append(
            Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).quantize(
                colors=256,
                method=Image.Quantize.MAXCOVERAGE,
                dither=Image.Dither.NONE,
            )
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=85, loop=0, optimize=True)

    waypoint_local = pixel_to_local(waypoint_pixel, px_to_m)
    summary = {
        "output": str(args.output),
        "planner": "cpp_hazard_route_demo",
        "replan_count": 1,
        "replan_frame": replan_frame,
        "start_pixel": list(start_pixel),
        "waypoint_pixel": list(waypoint_pixel),
        "waypoint_xy_m": list(waypoint_local),
        "blocked_route_cell": [blocked_route_cell.x, blocked_route_cell.y],
        "blocked_route_pixel": list(hazard_pixel),
        "blocked_cells": len(blocked_cells),
        "old_route_points": len(old_route),
        "new_route_points": len(new_route),
        "final_route_points": len(final_route),
        "old_route_metrics": old_metrics,
        "new_route_metrics": new_metrics,
        "blocked_cost": blocked_cost,
        "states": ["GUIDANCE ACTIVE", "ROUTE INVALID", "REPLANNING", "NEW ROUTE LOCK", "ARRIVED"],
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
