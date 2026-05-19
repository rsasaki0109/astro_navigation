#!/usr/bin/env python3
"""Render dynamic hazard replanning with TRN localizability cost.

This is the mission-facing version of the localizability map: the rover plans
through terrain that is both traversable and visually useful for TRN, then
replans with the same fused cost when a new blocked hazard invalidates the
active route.
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

from render_dynamic_hazard_replanning_demo import add_dynamic_hazard
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
from render_localizability_aware_route import build_route_costs, resolve_planner_app, route_confidence_metrics
from render_trn_confidence_heatmap import build_confidence


def draw_polyline(image: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int], thickness: int) -> None:
    if len(points) >= 2:
        cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)


def render_frame(
    *,
    ortho: np.ndarray,
    confidence: np.ndarray,
    base_hazard: np.ndarray,
    dynamic_hazard: np.ndarray,
    old_path_pixels: list[tuple[int, int]],
    new_path_pixels: list[tuple[int, int]],
    progress_pixels: list[tuple[int, int]],
    rover_pixel: tuple[int, int],
    waypoint_pixel: tuple[int, int],
    hazard_pixel: tuple[int, int],
    phase: str,
    nav_status: str,
    distance_m: float,
    route_confidence: float,
    risk_score: float,
    low_confidence_fraction: float,
    replan_count: int,
    output_size: tuple[int, int],
) -> np.ndarray:
    width, height = output_size
    map_w = 880
    side_w = width - map_w

    base = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    confidence_full = cv2.resize(confidence, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_CUBIC)
    heat = cv2.applyColorMap((confidence_full * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    overlay = cv2.addWeighted(base, 0.50, heat, 0.50, 0.0)

    base_hazard_full = cv2.resize(base_hazard, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_CUBIC)
    dynamic_full = cv2.resize(dynamic_hazard, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_NEAREST)
    overlay[base_hazard_full > 0.57] = (40, 45, 210)
    overlay[dynamic_full > 0.5] = (20, 40, 245)
    map_panel = cv2.resize(overlay, (map_w, height), interpolation=cv2.INTER_AREA)

    scale_x = map_w / ortho.shape[1]
    scale_y = height / ortho.shape[0]

    def scale_point(point: tuple[int, int]) -> tuple[int, int]:
        return int(round(point[0] * scale_x)), int(round(point[1] * scale_y))

    old_scaled = [scale_point(point) for point in old_path_pixels]
    new_scaled = [scale_point(point) for point in new_path_pixels]
    progress_scaled = [scale_point(point) for point in progress_pixels]
    rover_scaled = scale_point(rover_pixel)
    waypoint_scaled = scale_point(waypoint_pixel)
    hazard_scaled = scale_point(hazard_pixel)

    draw_polyline(map_panel, old_scaled, (170, 165, 155), 3)
    if new_scaled:
        draw_polyline(map_panel, new_scaled, (75, 250, 150), 5)
    draw_polyline(map_panel, progress_scaled, (75, 230, 255), 5)

    cv2.circle(map_panel, waypoint_scaled, 15, (65, 230, 255), 3, cv2.LINE_AA)
    cv2.circle(map_panel, waypoint_scaled, 5, (65, 230, 255), -1, cv2.LINE_AA)
    if np.max(dynamic_hazard) > 0.0:
        cv2.circle(map_panel, hazard_scaled, 18, (20, 40, 245), 3, cv2.LINE_AA)
        cv2.line(map_panel, (hazard_scaled[0] - 13, hazard_scaled[1] - 13),
                 (hazard_scaled[0] + 13, hazard_scaled[1] + 13), (20, 40, 245), 3, cv2.LINE_AA)
        cv2.line(map_panel, (hazard_scaled[0] + 13, hazard_scaled[1] - 13),
                 (hazard_scaled[0] - 13, hazard_scaled[1] + 13), (20, 40, 245), 3, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 13, (80, 255, 150), -1, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 16, (15, 30, 20), 2, cv2.LINE_AA)

    text(map_panel, "CONFIDENCE-AWARE REPLANNING", (24, 42), scale=0.72, color=(245, 246, 242), thickness=2)
    text(map_panel, "heat = TRN confidence   red = blocked hazard   green = replanned localizable route",
         (24, 76), scale=0.46, color=(220, 225, 220))
    text(map_panel, "WAYPOINT", (waypoint_scaled[0] + 18, waypoint_scaled[1] - 12), scale=0.5, color=(65, 230, 255))
    if np.max(dynamic_hazard) > 0.0:
        text(map_panel, "NEW HAZARD", (hazard_scaled[0] + 20, hazard_scaled[1] - 16), scale=0.48, color=(40, 90, 255))

    side = np.full((height, side_w, 3), (22, 25, 30), dtype=np.uint8)
    x0 = 28
    text(side, "TRN-AWARE", (x0, 52), scale=0.88, color=(245, 245, 240), thickness=2)
    text(side, "REPLAN", (x0, 90), scale=0.88, color=(245, 245, 240), thickness=2)
    text(side, phase, (x0, 138), scale=0.58, color=(95, 220, 255), thickness=2)

    rows = [
        ("NAV", nav_status),
        ("REPLANS", str(replan_count)),
        ("DISTANCE", f"{distance_m:6.0f} m"),
        ("ROUTE TRN", f"{route_confidence:.2f}"),
        ("LOW TRN", f"{100.0 * low_confidence_fraction:.0f} %"),
        ("RISK", f"{risk_score:.2f}"),
    ]
    y = 195
    for label, value in rows:
        text(side, label, (x0, y), scale=0.43, color=(125, 175, 220))
        text(side, value, (x0, y + 27), scale=0.54, color=(236, 238, 236))
        y += 55

    stages = ["TRN MAP", "FUSED COST", "ROUTE LOCK", "HAZARD", "REPLAN", "SAFE LOCK"]
    active_count = {
        "GUIDANCE ACTIVE": 3,
        "ROUTE INVALID": 4,
        "REPLANNING": 5,
        "NEW ROUTE LOCK": 6,
        "ARRIVED": 6,
    }.get(phase, 2)
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trn-fixture-dir", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_lro_tycho_terminal")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "confidence_aware_replanning_demo.gif")
    parser.add_argument("--planner-app", type=Path)
    parser.add_argument("--grid-size", type=int, default=170)
    parser.add_argument("--frames", type=int, default=58)
    parser.add_argument("--dynamic-hazard-radius", type=int, default=9)
    parser.add_argument("--localizability-weight", type=float, default=7.0)
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

    hazard, _hazard_cost = build_hazard_cost(ortho, args.grid_size)
    confidence, terms = build_confidence(ortho, args.grid_size)
    blocked_cost = 50.0
    _hazard_only_cost, nav_aware_cost, _low_confidence_penalty = build_route_costs(
        hazard=hazard,
        confidence=confidence,
        localizability_weight=args.localizability_weight,
        hazard_threshold=0.60,
        blocked_cost=blocked_cost,
    )

    scale = args.grid_size / ortho.shape[0]
    start_grid = nearest_low_cost(nav_aware_cost, GridPoint(int(start_pixel[0] * scale), int(start_pixel[1] * scale)))
    goal_grid = nearest_low_cost(nav_aware_cost, GridPoint(int(waypoint_pixel[0] * scale), int(waypoint_pixel[1] * scale)))
    grid_resolution_m = px_to_m * ortho.shape[0] / args.grid_size

    with tempfile.TemporaryDirectory(prefix="astro_confidence_replan_") as temp_dir:
        workdir = Path(temp_dir)
        old_route, old_metrics = plan_with_cpp_cli(
            planner_app=planner_app,
            cost=nav_aware_cost,
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
            cost=nav_aware_cost,
            hazard=np.zeros_like(hazard),
            center=blocked_route_cell,
            radius_cells=args.dynamic_hazard_radius,
            blocked_cost=blocked_cost,
        )

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
    final_confidence = route_confidence_metrics(
        route=final_route,
        confidence=confidence,
        grid_resolution_m=grid_resolution_m,
    )
    route_trn_confidence = float(final_confidence["mean_trn_confidence"])
    risk_score = 1.0 - min(route_trn_confidence, float(confidence[start_grid.y, start_grid.x]))

    old_path_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in old_route]
    new_path_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in new_route]
    final_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in final_route]
    rover_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in resample_path(final_route, args.frames)]
    hazard_pixel = grid_to_pixel(blocked_route_cell, ortho.shape, args.grid_size)

    replan_frame = max(8, int(args.frames * 0.42))
    frames: list[Image.Image] = []
    for idx, rover_pixel in enumerate(rover_pixels):
        distance_m = math.hypot(rover_pixel[0] - waypoint_pixel[0], rover_pixel[1] - waypoint_pixel[1]) * px_to_m
        if idx < replan_frame - 2:
            phase = "GUIDANCE ACTIVE"
            nav_status = "OK / ROUTE LOCK"
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
            nav_status = "DEGRADED / RISK"
            visible_new_path = new_path_pixels
            active_hazard = dynamic_hazard
            replan_count = 1
        elif idx >= len(rover_pixels) - 4:
            phase = "ARRIVED"
            nav_status = "OK / ROUTE LOCK"
            visible_new_path = new_path_pixels
            active_hazard = dynamic_hazard
            replan_count = 1
            distance_m = 0.0
        else:
            phase = "NEW ROUTE LOCK"
            nav_status = "OK / ROUTE LOCK"
            visible_new_path = new_path_pixels
            active_hazard = dynamic_hazard
            replan_count = 1

        progress_count = max(1, int(round((idx + 1) / args.frames * len(final_pixels))))
        frame = render_frame(
            ortho=ortho,
            confidence=confidence,
            base_hazard=hazard,
            dynamic_hazard=active_hazard,
            old_path_pixels=old_path_pixels,
            new_path_pixels=visible_new_path,
            progress_pixels=final_pixels[:progress_count],
            rover_pixel=rover_pixel,
            waypoint_pixel=waypoint_pixel,
            hazard_pixel=hazard_pixel,
            phase=phase,
            nav_status=nav_status,
            distance_m=distance_m,
            route_confidence=route_trn_confidence,
            risk_score=risk_score,
            low_confidence_fraction=float(final_confidence["low_confidence_fraction"]),
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
        "feature_detector": str(terms["feature_detector"]),
        "feature_count": int(terms["feature_count"]),
        "localizability_weight": args.localizability_weight,
        "localizability_score": float(confidence[start_grid.y, start_grid.x]),
        "route_trn_confidence": route_trn_confidence,
        "navigation_risk_score": risk_score,
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
        "final_route_confidence": final_confidence,
        "blocked_cost": blocked_cost,
        "states": ["GUIDANCE ACTIVE", "ROUTE INVALID", "REPLANNING", "NEW ROUTE LOCK", "ARRIVED"],
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
