#!/usr/bin/env python3
"""Render a route that prefers terrain where TRN can localize well.

Hazard-aware routing answers "where is it safe to drive?"  This demo adds the
navigation-side question: "where is the rover less likely to lose TRN lock?"

The script keeps the C++ planner unchanged. It builds two cost maps from the
same Tycho fixture:

- hazard-only: shadow/edge hazards drive the route
- localizability-aware: the same hazards plus a penalty for weak TRN confidence

The output compares both routes and reports route-level TRN confidence.
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
    plan_with_cpp_cli,
    text,
)
from render_trn_confidence_heatmap import build_confidence


def resolve_planner_app(path: Path | None) -> Path:
    candidates: list[Path] = []
    if path is not None:
        candidates.append(path)
    candidates.extend(
        [
            REPO_ROOT / "build" / "apps" / "hazard_route_demo",
            Path("/tmp/astro_navigation-build-renamed/apps/hazard_route_demo"),
            Path("/tmp/astro_navigation-build/apps/hazard_route_demo"),
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit("hazard_route_demo not found; build it or pass --planner-app")


def normalize01(values: np.ndarray) -> np.ndarray:
    lo = float(np.percentile(values, 2.0))
    hi = float(np.percentile(values, 98.0))
    if hi <= lo:
        return np.zeros_like(values, dtype=np.float32)
    return np.clip((values.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)


def harden_hazards(cost: np.ndarray, hazard: np.ndarray, hazard_threshold: float, blocked_cost: float) -> np.ndarray:
    hardened = cost.copy()
    hardened[hazard >= hazard_threshold] = blocked_cost
    return hardened.astype(np.float32)


def build_route_costs(
    *,
    hazard: np.ndarray,
    confidence: np.ndarray,
    localizability_weight: float,
    hazard_threshold: float,
    blocked_cost: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    hazard_penalty = 1.0 + 8.0 * normalize01(hazard)
    low_confidence_penalty = np.power(np.clip(1.0 - confidence, 0.0, 1.0), 1.35)

    hazard_only = harden_hazards(
        hazard_penalty,
        hazard,
        hazard_threshold=hazard_threshold,
        blocked_cost=blocked_cost,
    )
    nav_aware = harden_hazards(
        hazard_penalty + localizability_weight * low_confidence_penalty,
        hazard,
        hazard_threshold=hazard_threshold,
        blocked_cost=blocked_cost,
    )
    return hazard_only, nav_aware, low_confidence_penalty.astype(np.float32)


def route_confidence_metrics(
    *,
    route: list[GridPoint],
    confidence: np.ndarray,
    grid_resolution_m: float,
) -> dict[str, float]:
    values = np.asarray([float(confidence[point.y, point.x]) for point in route], dtype=np.float32)
    length_m = 0.0
    for previous, current in zip(route[:-1], route[1:]):
        length_m += math.hypot(current.x - previous.x, current.y - previous.y) * grid_resolution_m
    straight_m = math.hypot(route[-1].x - route[0].x, route[-1].y - route[0].y) * grid_resolution_m
    return {
        "mean_trn_confidence": float(np.mean(values)),
        "min_trn_confidence": float(np.min(values)),
        "p10_trn_confidence": float(np.percentile(values, 10.0)),
        "low_confidence_fraction": float(np.mean(values < 0.35)),
        "route_length_m": length_m,
        "detour_ratio": length_m / straight_m if straight_m > 0.0 else 1.0,
    }


def draw_polyline(image: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int], thickness: int) -> None:
    if len(points) >= 2:
        cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)


def render_panel(
    *,
    ortho: np.ndarray,
    hazard: np.ndarray,
    confidence: np.ndarray,
    hazard_route_pixels: list[tuple[int, int]],
    nav_route_pixels: list[tuple[int, int]],
    start_pixel: tuple[int, int],
    waypoint_pixel: tuple[int, int],
    hazard_metrics: dict[str, float],
    nav_metrics: dict[str, float],
    localizability_weight: float,
    output_size: tuple[int, int],
) -> np.ndarray:
    width, height = output_size
    map_w = 880
    side_w = width - map_w

    base = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    confidence_full = cv2.resize(confidence, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_CUBIC)
    hazard_full = cv2.resize(hazard, (ortho.shape[1], ortho.shape[0]), interpolation=cv2.INTER_CUBIC)
    heat = cv2.applyColorMap((confidence_full * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    overlay = cv2.addWeighted(base, 0.50, heat, 0.50, 0.0)
    overlay[hazard_full > 0.57] = (40, 45, 210)
    map_panel = cv2.resize(overlay, (map_w, height), interpolation=cv2.INTER_AREA)

    scale_x = map_w / ortho.shape[1]
    scale_y = height / ortho.shape[0]

    def scale_point(point: tuple[int, int]) -> tuple[int, int]:
        return int(round(point[0] * scale_x)), int(round(point[1] * scale_y))

    hazard_scaled = [scale_point(point) for point in hazard_route_pixels]
    nav_scaled = [scale_point(point) for point in nav_route_pixels]
    start_scaled = scale_point(start_pixel)
    waypoint_scaled = scale_point(waypoint_pixel)

    draw_polyline(map_panel, hazard_scaled, (160, 160, 170), 4)
    draw_polyline(map_panel, nav_scaled, (70, 245, 150), 5)
    cv2.circle(map_panel, start_scaled, 13, (70, 245, 150), -1, cv2.LINE_AA)
    cv2.circle(map_panel, start_scaled, 16, (15, 30, 20), 2, cv2.LINE_AA)
    cv2.circle(map_panel, waypoint_scaled, 15, (60, 230, 255), 3, cv2.LINE_AA)
    cv2.circle(map_panel, waypoint_scaled, 5, (60, 230, 255), -1, cv2.LINE_AA)

    text(map_panel, "LOCALIZABILITY-AWARE ROUTE", (24, 42), scale=0.76, color=(245, 246, 242), thickness=2)
    text(
        map_panel,
        "gray = hazard-only route   green = route biased toward stronger TRN confidence",
        (24, 76),
        scale=0.47,
        color=(220, 225, 220),
    )
    text(map_panel, "START", (start_scaled[0] + 18, start_scaled[1] - 12), scale=0.48, color=(100, 245, 150))
    text(map_panel, "WAYPOINT", (waypoint_scaled[0] + 18, waypoint_scaled[1] - 12), scale=0.48, color=(60, 230, 255))

    side = np.full((height, side_w, 3), (22, 25, 30), dtype=np.uint8)
    x0 = 28
    text(side, "TRN-AWARE", (x0, 52), scale=0.88, color=(245, 245, 240), thickness=2)
    text(side, "GUIDANCE", (x0, 90), scale=0.88, color=(245, 245, 240), thickness=2)
    text(side, "LOCALIZABILITY COST", (x0, 138), scale=0.48, color=(95, 220, 255), thickness=2)

    rows = [
        ("WEIGHT", f"{localizability_weight:4.1f} x"),
        ("HAZARD MEAN TRN", f"{hazard_metrics['mean_trn_confidence']:.2f}"),
        ("TRN-AWARE MEAN", f"{nav_metrics['mean_trn_confidence']:.2f}"),
        ("HAZARD LOW TRN", f"{100.0 * hazard_metrics['low_confidence_fraction']:.0f} %"),
        ("TRN-AWARE LOW", f"{100.0 * nav_metrics['low_confidence_fraction']:.0f} %"),
        ("LENGTH RATIO", f"{nav_metrics['route_length_m'] / hazard_metrics['route_length_m']:.2f} x"),
    ]
    y = 195
    for label, value in rows:
        text(side, label, (x0, y), scale=0.42, color=(125, 175, 220))
        text(side, value, (x0, y + 28), scale=0.56, color=(236, 238, 236))
        y += 57

    checkpoints = ["HAZARD MAP", "TRN CONF MAP", "FUSED COST", "C++ A*", "ROUTE LOCK"]
    y_stage = height - 140
    for idx, label in enumerate(checkpoints):
        cy = y_stage + idx * 24
        cv2.circle(side, (x0 + 8, cy - 5), 5, (92, 215, 125), -1, cv2.LINE_AA)
        text(side, label, (x0 + 25, cy), scale=0.36, color=(215, 225, 220))

    frame = np.full((height, width, 3), (10, 12, 15), dtype=np.uint8)
    frame[:, :map_w] = map_panel
    frame[:, map_w:] = side
    return frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trn-fixture-dir", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_lro_tycho_terminal")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "localizability_aware_route.png")
    parser.add_argument("--planner-app", type=Path)
    parser.add_argument("--grid-size", type=int, default=170)
    parser.add_argument("--localizability-weight", type=float, default=7.0)
    parser.add_argument("--hazard-threshold", type=float, default=0.60)
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
    hazard_only_cost, nav_aware_cost, low_confidence_penalty = build_route_costs(
        hazard=hazard,
        confidence=confidence,
        localizability_weight=args.localizability_weight,
        hazard_threshold=args.hazard_threshold,
        blocked_cost=blocked_cost,
    )

    scale = args.grid_size / ortho.shape[0]
    start_grid = nearest_low_cost(hazard_only_cost, GridPoint(int(start_pixel[0] * scale), int(start_pixel[1] * scale)))
    goal_grid = nearest_low_cost(hazard_only_cost, GridPoint(int(waypoint_pixel[0] * scale), int(waypoint_pixel[1] * scale)))
    grid_resolution_m = px_to_m * ortho.shape[0] / args.grid_size

    with tempfile.TemporaryDirectory(prefix="astro_localizability_route_") as temp_dir:
        workdir = Path(temp_dir)
        hazard_route, hazard_planner_metrics = plan_with_cpp_cli(
            planner_app=planner_app,
            cost=hazard_only_cost,
            start=start_grid,
            goal=goal_grid,
            workdir=workdir / "hazard_only",
            grid_resolution_m=grid_resolution_m,
            blocked_cost=blocked_cost,
        )
        nav_route, nav_planner_metrics = plan_with_cpp_cli(
            planner_app=planner_app,
            cost=nav_aware_cost,
            start=start_grid,
            goal=goal_grid,
            workdir=workdir / "nav_aware",
            grid_resolution_m=grid_resolution_m,
            blocked_cost=blocked_cost,
        )

    hazard_metrics = route_confidence_metrics(
        route=hazard_route,
        confidence=confidence,
        grid_resolution_m=grid_resolution_m,
    )
    nav_metrics = route_confidence_metrics(
        route=nav_route,
        confidence=confidence,
        grid_resolution_m=grid_resolution_m,
    )
    hazard_metrics["planner_mean_cost"] = float(hazard_planner_metrics.get("mean_cost") or 0.0)
    nav_metrics["planner_mean_cost"] = float(nav_planner_metrics.get("mean_cost") or 0.0)

    hazard_route_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in hazard_route]
    nav_route_pixels = [grid_to_pixel(point, ortho.shape, args.grid_size) for point in nav_route]
    panel = render_panel(
        ortho=ortho,
        hazard=hazard,
        confidence=confidence,
        hazard_route_pixels=hazard_route_pixels,
        nav_route_pixels=nav_route_pixels,
        start_pixel=start_pixel,
        waypoint_pixel=waypoint_pixel,
        hazard_metrics=hazard_metrics,
        nav_metrics=nav_metrics,
        localizability_weight=args.localizability_weight,
        output_size=(1280, 720),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)).save(args.output)

    summary = {
        "output": str(args.output),
        "planner_app": str(planner_app),
        "grid_size": args.grid_size,
        "start_pixel": list(start_pixel),
        "waypoint_pixel": list(waypoint_pixel),
        "blocked_cost": blocked_cost,
        "hazard_threshold": args.hazard_threshold,
        "localizability_weight": args.localizability_weight,
        "confidence_model": "gradient + texture + feature_density + illumination_balance",
        "feature_detector": str(terms["feature_detector"]),
        "feature_count": int(terms["feature_count"]),
        "low_confidence_penalty_mean": float(np.mean(low_confidence_penalty)),
        "hazard_only_route": hazard_metrics,
        "localizability_aware_route": nav_metrics,
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
