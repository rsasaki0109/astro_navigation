#!/usr/bin/env python3
"""Render a hazard-aware lunar navigation demo GIF.

This is a deliberately lightweight guidance demo on the committed Tycho TRN
fixture. It derives a hazard cost map from shadowed pixels and sharp terrain
edges, plans a route with A*, and animates a rover from the TRN lock toward a
waypoint while preserving the navigation-state story:

  LOST -> DEGRADED -> OK -> GUIDANCE ACTIVE -> RELOCALIZING -> OK -> ARRIVED
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class GridPoint:
    x: int
    y: int


def text(
    image: np.ndarray,
    value: str,
    origin: tuple[int, int],
    *,
    scale: float,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        value,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        value,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def local_to_pixel(x_m: float, y_m: float, px_to_m: float, image_shape: tuple[int, int]) -> tuple[int, int]:
    height, width = image_shape
    x = int(round(x_m / px_to_m))
    y = int(round(y_m / px_to_m))
    return max(0, min(width - 1, x)), max(0, min(height - 1, y))


def pixel_to_local(pixel: tuple[int, int], px_to_m: float) -> tuple[float, float]:
    return pixel[0] * px_to_m, pixel[1] * px_to_m


def normalize01(values: np.ndarray) -> np.ndarray:
    lo = float(np.percentile(values, 2.0))
    hi = float(np.percentile(values, 98.0))
    if hi <= lo:
        return np.zeros_like(values, dtype=np.float32)
    return np.clip((values.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)


def build_hazard_cost(ortho: np.ndarray, grid_size: int) -> tuple[np.ndarray, np.ndarray]:
    small = cv2.resize(ortho, (grid_size, grid_size), interpolation=cv2.INTER_AREA)
    blurred = cv2.GaussianBlur(small, (0, 0), 1.2)

    darkness = 1.0 - normalize01(blurred)
    grad_x = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3)
    gradient = normalize01(cv2.magnitude(grad_x, grad_y))

    hazard = np.clip(0.65 * darkness + 0.55 * gradient, 0.0, 1.0)
    hazard = cv2.GaussianBlur(hazard, (0, 0), 0.8)
    cost = 1.0 + 13.0 * hazard
    return hazard.astype(np.float32), cost.astype(np.float32)


def nearest_low_cost(cost: np.ndarray, point: GridPoint, radius: int = 10) -> GridPoint:
    h, w = cost.shape
    best = point
    best_cost = float("inf")
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x = min(w - 1, max(0, point.x + dx))
            y = min(h - 1, max(0, point.y + dy))
            value = float(cost[y, x])
            if value < best_cost:
                best = GridPoint(x, y)
                best_cost = value
    return best


def astar(cost: np.ndarray, start: GridPoint, goal: GridPoint) -> list[GridPoint]:
    h, w = cost.shape
    neighbors = [
        (-1, -1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, 1, 1.0),
        (1, 1, math.sqrt(2.0)),
    ]

    def heuristic(a: GridPoint, b: GridPoint) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    frontier: list[tuple[float, int, GridPoint]] = []
    heapq.heappush(frontier, (0.0, 0, start))
    came_from: dict[GridPoint, GridPoint | None] = {start: None}
    cost_so_far: dict[GridPoint, float] = {start: 0.0}
    counter = 0

    while frontier:
        _priority, _counter, current = heapq.heappop(frontier)
        if current == goal:
            break

        for dx, dy, step in neighbors:
            nx = current.x + dx
            ny = current.y + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            next_point = GridPoint(nx, ny)
            transition_cost = step * (0.5 * float(cost[current.y, current.x]) + 0.5 * float(cost[ny, nx]))
            new_cost = cost_so_far[current] + transition_cost
            if next_point not in cost_so_far or new_cost < cost_so_far[next_point]:
                cost_so_far[next_point] = new_cost
                counter += 1
                priority = new_cost + 1.3 * heuristic(next_point, goal)
                heapq.heappush(frontier, (priority, counter, next_point))
                came_from[next_point] = current

    if goal not in came_from:
        raise RuntimeError("A* failed to find a route")

    path: list[GridPoint] = []
    current: GridPoint | None = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def plan_with_cpp_cli(
    *,
    planner_app: Path,
    cost: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    workdir: Path,
    grid_resolution_m: float,
    blocked_cost: float,
) -> tuple[list[GridPoint], dict[str, float | None]]:
    workdir.mkdir(parents=True, exist_ok=True)
    cost_csv = workdir / "hazard_cost_map.csv"
    route_csv = workdir / "hazard_route.csv"
    route_json = workdir / "hazard_route.json"
    np.savetxt(cost_csv, cost, delimiter=",", fmt="%.9g")

    subprocess.run(
        [
            str(planner_app),
            "--cost-map",
            str(cost_csv),
            "--start-cell-x",
            str(start.x),
            "--start-cell-y",
            str(start.y),
            "--goal-cell-x",
            str(goal.x),
            "--goal-cell-y",
            str(goal.y),
            "--resolution-m",
            str(grid_resolution_m),
            "--blocked-cost",
            str(blocked_cost),
            "--snap-radius-cells",
            "10",
            "--output-csv",
            str(route_csv),
            "--output-json",
            str(route_json),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
    )

    route: list[GridPoint] = []
    for line in route_csv.read_text(encoding="utf-8").splitlines()[1:]:
        if not line:
            continue
        fields = line.split(",")
        route.append(GridPoint(int(fields[1]), int(fields[2])))
    if not route:
        raise RuntimeError("C++ hazard_route_demo returned an empty route")
    route_summary = json.loads(route_json.read_text(encoding="utf-8"))
    return route, route_summary.get("metrics", {})


def compute_python_route_metrics(
    *,
    cost: np.ndarray,
    route: list[GridPoint],
    grid_resolution_m: float,
    blocked_cost: float,
) -> dict[str, float | None]:
    if not route:
        return {}

    length_m = 0.0
    for previous, current in zip(route[:-1], route[1:]):
        length_m += math.hypot(current.x - previous.x, current.y - previous.y) * grid_resolution_m
    straight_m = math.hypot(route[-1].x - route[0].x, route[-1].y - route[0].y) * grid_resolution_m
    route_costs = [float(cost[point.y, point.x]) for point in route]

    blocked_yx = np.argwhere(~np.isfinite(cost) | (cost >= blocked_cost))
    min_clearance_cells: float | None = None
    if len(blocked_yx) > 0:
        best = float("inf")
        for point in route:
            deltas = blocked_yx - np.asarray([point.y, point.x])
            best = min(best, float(np.sqrt(np.min(np.sum(deltas * deltas, axis=1)))))
        min_clearance_cells = best

    return {
        "route_length_m": length_m,
        "straight_line_length_m": straight_m,
        "detour_ratio": length_m / straight_m if straight_m > 0.0 else 1.0,
        "mean_cost": float(np.mean(route_costs)),
        "max_cost": float(np.max(route_costs)),
        "min_clearance_cells": min_clearance_cells,
        "min_clearance_m": None if min_clearance_cells is None else min_clearance_cells * grid_resolution_m,
    }


def resample_path(path: list[GridPoint], count: int) -> list[GridPoint]:
    if len(path) <= count:
        return path
    indices = np.linspace(0, len(path) - 1, count)
    return [path[int(round(index))] for index in indices]


def grid_to_pixel(point: GridPoint, image_shape: tuple[int, int], grid_size: int) -> tuple[int, int]:
    h, w = image_shape
    return (
        int(round((point.x + 0.5) * w / grid_size)),
        int(round((point.y + 0.5) * h / grid_size)),
    )


def draw_polyline(image: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int], thickness: int) -> None:
    if len(points) < 2:
        return
    cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)


def render_frame(
    *,
    ortho: np.ndarray,
    hazard: np.ndarray,
    path_pixels: list[tuple[int, int]],
    rover_pixel: tuple[int, int],
    waypoint_pixel: tuple[int, int],
    start_pixel: tuple[int, int],
    px_to_m: float,
    sigma_m: float,
    frame_index: int,
    frame_count: int,
    status: str,
    reason: str,
    phase: str,
    distance_m: float,
    route_metrics: dict[str, float | None],
    output_size: tuple[int, int],
) -> np.ndarray:
    width, height = output_size
    map_w = 880
    side_w = width - map_w
    map_h = height

    base = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    hazard_full = cv2.resize(hazard, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_CUBIC)
    hazard_mask = hazard_full > 0.57
    overlay = base.copy()
    overlay[hazard_mask] = (40, 45, 210)
    base = cv2.addWeighted(overlay, 0.34, base, 0.66, 0.0)

    scale_x = map_w / ortho.shape[1]
    scale_y = map_h / ortho.shape[0]
    map_panel = cv2.resize(base, (map_w, map_h), interpolation=cv2.INTER_AREA)

    def scale_point(point: tuple[int, int]) -> tuple[int, int]:
        return int(round(point[0] * scale_x)), int(round(point[1] * scale_y))

    scaled_path = [scale_point(point) for point in path_pixels]
    rover_scaled = scale_point(rover_pixel)
    waypoint_scaled = scale_point(waypoint_pixel)
    start_scaled = scale_point(start_pixel)
    sigma_px = int(round(sigma_m / px_to_m * (scale_x + scale_y) * 0.5))

    draw_polyline(map_panel, scaled_path, (235, 190, 70), 4)
    traveled_count = max(1, int(round((frame_index + 1) / frame_count * len(scaled_path))))
    draw_polyline(map_panel, scaled_path[:traveled_count], (90, 245, 140), 5)

    cv2.circle(map_panel, start_scaled, 9, (100, 245, 140), 3, cv2.LINE_AA)
    cv2.circle(map_panel, waypoint_scaled, 15, (60, 230, 255), 3, cv2.LINE_AA)
    cv2.circle(map_panel, waypoint_scaled, 5, (60, 230, 255), -1, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, max(sigma_px, 1), (75, 205, 255), 2, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 13, (80, 255, 150), -1, cv2.LINE_AA)
    cv2.circle(map_panel, rover_scaled, 16, (15, 30, 20), 2, cv2.LINE_AA)

    text(map_panel, "HAZARD COST MAP", (24, 42), scale=0.86, color=(245, 246, 242), thickness=2)
    text(map_panel, "red = shadow/steep edge cost   blue = planned route   green = rover progress", (24, 76), scale=0.48, color=(210, 220, 225))
    text(map_panel, "WAYPOINT", (waypoint_scaled[0] + 18, waypoint_scaled[1] - 12), scale=0.5, color=(60, 230, 255))

    side = np.full((height, side_w, 3), (22, 25, 30), dtype=np.uint8)
    x0 = 28
    text(side, "AUTONOMOUS", (x0, 52), scale=0.9, color=(245, 245, 240), thickness=2)
    text(side, "LUNAR NAV", (x0, 90), scale=0.9, color=(245, 245, 240), thickness=2)
    text(side, phase, (x0, 138), scale=0.62, color=(95, 220, 255), thickness=2)

    detour_ratio = route_metrics.get("detour_ratio")
    clearance_m = route_metrics.get("min_clearance_m")
    rows = [
        ("NAV", f"{status} / {reason}"),
        ("GUIDANCE", "ACTIVE" if status == "OK" and phase != "ARRIVED" else phase),
        ("DISTANCE", f"{distance_m:6.0f} m"),
        ("SIGMA", f"{sigma_m:6.1f} m"),
        ("DETOUR", "--" if detour_ratio is None else f"{detour_ratio:4.2f} x"),
        ("CLEARANCE", "--" if clearance_m is None else f"{clearance_m:6.0f} m"),
    ]
    y = 195
    for label, value in rows:
        text(side, label, (x0, y), scale=0.45, color=(125, 175, 220))
        text(side, value, (x0, y + 28), scale=0.55, color=(236, 238, 236))
        y += 56

    stages = ["LOST", "STAR LOCK", "TRN LOCK", "GUIDANCE", "RELOCK", "ARRIVED"]
    y_stage = height - 150
    for idx, label in enumerate(stages):
        active = idx <= min(frame_index // max(1, frame_count // len(stages)), len(stages) - 1)
        color = (92, 215, 125) if active else (70, 78, 88)
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
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "hazard_aware_navigation_demo.gif")
    parser.add_argument("--grid-size", type=int, default=170)
    parser.add_argument("--frames", type=int, default=42)
    parser.add_argument("--planner-app", type=Path, help="Optional hazard_route_demo binary to plan with the C++ navigation API")
    parser.add_argument("--planner-workdir", type=Path, help="Optional directory for planner CSV exchange files")
    args = parser.parse_args()

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
    # A visibly distinct target near the north-east part of the map. It is
    # intentionally chosen in image coordinates so the demo remains stable with
    # the committed fixture.
    waypoint_pixel = (900, 430)

    hazard, cost = build_hazard_cost(ortho, args.grid_size)
    scale = args.grid_size / ortho.shape[0]
    start_grid = nearest_low_cost(cost, GridPoint(int(start_pixel[0] * scale), int(start_pixel[1] * scale)))
    goal_grid = nearest_low_cost(cost, GridPoint(int(waypoint_pixel[0] * scale), int(waypoint_pixel[1] * scale)))
    grid_resolution_m = px_to_m * ortho.shape[0] / args.grid_size
    blocked_cost = 8.5
    planner = "python_astar"
    if args.planner_app:
        planner = "cpp_hazard_route_demo"
        if args.planner_workdir:
            grid_path, route_metrics = plan_with_cpp_cli(
                planner_app=args.planner_app,
                cost=cost,
                start=start_grid,
                goal=goal_grid,
                workdir=args.planner_workdir,
                grid_resolution_m=grid_resolution_m,
                blocked_cost=blocked_cost,
            )
        else:
            with tempfile.TemporaryDirectory(prefix="astro_hazard_route_") as temp_dir:
                grid_path, route_metrics = plan_with_cpp_cli(
                    planner_app=args.planner_app,
                    cost=cost,
                    start=start_grid,
                    goal=goal_grid,
                    workdir=Path(temp_dir),
                    grid_resolution_m=grid_resolution_m,
                    blocked_cost=blocked_cost,
                )
    else:
        grid_path = astar(cost, start_grid, goal_grid)
        route_metrics = compute_python_route_metrics(
            cost=cost,
            route=grid_path,
            grid_resolution_m=grid_resolution_m,
            blocked_cost=blocked_cost,
        )
    sampled = resample_path(grid_path, args.frames)
    path_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in grid_path]
    rover_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in sampled]

    sigma_m = max(2.0 * px_to_m, px_to_m * math.sqrt(12.0 / max(int(trn["pnp"]["pnp_inliers"]), 1)))
    waypoint_local = pixel_to_local(waypoint_pixel, px_to_m)

    frames: list[Image.Image] = []
    for idx, rover_pixel in enumerate(rover_pixels):
        distance_m = math.hypot(rover_pixel[0] - waypoint_pixel[0], rover_pixel[1] - waypoint_pixel[1]) * px_to_m
        if idx < 5:
            status, reason, phase = "LOST", "NO_LOCKS", "BOOTING"
        elif idx < 10:
            status, reason, phase = "DEGRADED", "ATTITUDE_ONLY", "STAR LOCK"
        elif 24 <= idx < 29:
            status, reason, phase = "DEGRADED", "ATTITUDE_ONLY", "RELOCALIZING"
        elif idx >= len(rover_pixels) - 4:
            status, reason, phase = "OK", "NONE", "ARRIVED"
            distance_m = 0.0
        else:
            status, reason, phase = "OK", "NONE", "GUIDANCE ACTIVE"

        frame = render_frame(
            ortho=ortho,
            hazard=hazard,
            path_pixels=path_pixels,
            rover_pixel=rover_pixel,
            waypoint_pixel=waypoint_pixel,
            start_pixel=start_pixel,
            px_to_m=px_to_m,
            sigma_m=sigma_m,
            frame_index=idx,
            frame_count=len(rover_pixels),
            status=status,
            reason=reason,
            phase=phase,
            distance_m=distance_m,
            route_metrics=route_metrics,
            output_size=(1280, 720),
        )
        # The source map is photographic, so raw RGB GIF frames become very
        # large. Quantizing each rendered frame keeps the README asset portable
        # while preserving the route and telemetry overlays.
        frames.append(
            Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).quantize(
                colors=256,
                method=Image.Quantize.MAXCOVERAGE,
                dither=Image.Dither.NONE,
            )
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=85,
        loop=0,
        optimize=True,
    )

    summary = {
        "output": str(args.output),
        "start_pixel": list(start_pixel),
        "waypoint_pixel": list(waypoint_pixel),
        "waypoint_xy_m": list(waypoint_local),
        "path_points": len(grid_path),
        "frames": len(frames),
        "position_sigma_m": sigma_m,
        "hazard_model": "0.65*darkness + 0.55*gradient",
        "blocked_cost": blocked_cost,
        "planner": planner,
        "route_metrics": route_metrics,
        "states": ["LOST", "DEGRADED", "OK", "DEGRADED", "OK", "ARRIVED"],
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
