#!/usr/bin/env python3
"""Render a navigation replay GIF: LOST -> DEGRADED -> OK.

This demo uses the C++ mission_navigation_demo app as the navigation source of
truth. It generates a small synthetic star-camera case, runs the app twice
(attitude-only and attitude+TRN), and visualizes the state transition with the
bundled Tycho terminal TRN fixture.
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


def run_capture(cmd: list[str]) -> str:
    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(f"$ {' '.join(cmd)}\n{result.stdout}\n{result.stderr}\n")
        raise SystemExit(result.returncode)
    return result.stdout


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


def read_observations(path: Path) -> list[tuple[str, float, float]]:
    rows: list[tuple[str, float, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((row["id"], float(row["u"]), float(row["v"])))
    return rows


def local_to_pixel(x_m: float, y_m: float, px_to_m: float, image_shape: tuple[int, int]) -> tuple[int, int]:
    height, width = image_shape
    x = int(round(x_m / px_to_m))
    y = int(round(y_m / px_to_m))
    return max(0, min(width - 1, x)), max(0, min(height - 1, y))


def trn_quality_terms(trn: dict) -> dict[str, float | str]:
    px_to_m = float(trn["wac"]["px_to_m"])
    inliers = max(int(trn["pnp"]["pnp_inliers"]), 1)
    reproj_px = float(trn["pnp"].get("inlier_median_reproj_px", 0.0))
    terms = {
        "map_resolution": 2.0 * px_to_m,
        "reprojection": reproj_px * px_to_m,
        "inlier_geometry": px_to_m * math.sqrt(12.0 / inliers),
    }
    dominant = max(terms.items(), key=lambda item: item[1])[0]
    return {**terms, "dominant": dominant}


def load_or_create_nav_states(args: argparse.Namespace, star_case: Path, summary_path: Path) -> tuple[dict, dict]:
    degraded_path = args.workdir / "nav_degraded.json"
    ok_path = args.workdir / "nav_ok.json"
    common = [
        str(args.nav_app),
        "--catalog",
        str(star_case / "catalog.csv"),
        "--observations",
        str(star_case / "observations.csv"),
        "--fx",
        "1000",
        "--fy",
        "1000",
        "--cx",
        "512",
        "--cy",
        "512",
    ]
    run_capture([*common, "--output-json", str(degraded_path)])
    run_capture([*common, "--trn-summary", str(summary_path), "--output-json", str(ok_path)])
    return (
        json.loads(degraded_path.read_text(encoding="utf-8")),
        json.loads(ok_path.read_text(encoding="utf-8")),
    )


def make_lost_state() -> dict:
    return {
        "timestamp": 0.0,
        "status": "LOST",
        "status_reason": "NO_LOCKS",
        "message": "navigation lost",
        "position_frame_id": "map",
        "attitude_reference_frame_id": "inertial",
        "position_m": [0.0, 0.0, 0.0],
        "velocity_mps": [0.0, 0.0, 0.0],
        "q_body_reference_xyzw": [0.0, 0.0, 0.0, 1.0],
        "quality": {
            "attitude_lock": False,
            "position_lock": False,
            "velocity_lock": False,
            "attitude_sigma_rad": 0.0,
            "position_sigma_m": 0.0,
            "attitude_correspondences": 0,
        },
    }


def render_star_panel(
    observations: list[tuple[str, float, float]],
    nav_state: dict,
    *,
    size: tuple[int, int],
) -> np.ndarray:
    width, height = size
    panel = np.full((height, width, 3), (14, 18, 26), dtype=np.uint8)
    text(panel, "STAR CAMERA", (24, 42), scale=0.85, color=(238, 240, 236), thickness=2)

    if not nav_state["quality"]["attitude_lock"]:
        center = (width // 2, height // 2)
        cv2.circle(panel, center, 66, (68, 78, 92), 2, cv2.LINE_AA)
        cv2.line(panel, (center[0] - 44, center[1]), (center[0] + 44, center[1]), (68, 78, 92), 1)
        cv2.line(panel, (center[0], center[1] - 44), (center[0], center[1] + 44), (68, 78, 92), 1)
        text(panel, "NO ATTITUDE LOCK", (52, height // 2 + 110), scale=0.65, color=(95, 180, 235), thickness=2)
        return panel

    sx = width / 1024.0
    sy = height / 1024.0
    for index, (_star_id, u, v) in enumerate(observations):
        x = int(round(u * sx))
        y = int(round(v * sy))
        radius = 3 + (index % 3)
        cv2.circle(panel, (x, y), radius, (238, 238, 220), -1, cv2.LINE_AA)
        cv2.circle(panel, (x, y), radius + 7, (105, 225, 140), 1, cv2.LINE_AA)

    qx, qy, qz, qw = nav_state["q_body_reference_xyzw"]
    text(panel, f"{nav_state['quality']['attitude_correspondences']} stars identified", (24, 82), scale=0.55, color=(170, 230, 185))
    text(panel, f"sigma {nav_state['quality']['attitude_sigma_rad']:.6f} rad", (24, 110), scale=0.5, color=(185, 198, 210))
    text(panel, "q body<-inertial", (24, height - 70), scale=0.48, color=(175, 190, 205))
    text(panel, f"[{qx:+.3f}, {qy:+.3f}, {qz:+.3f}, {qw:+.3f}]", (24, height - 42), scale=0.5, color=(238, 240, 236))
    return panel


def render_map_panel(
    ortho: np.ndarray,
    trn: dict,
    nav_state: dict,
    terms: dict[str, float | str],
    *,
    size: tuple[int, int],
) -> np.ndarray:
    width, height = size
    color = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    px_to_m = float(trn["wac"]["px_to_m"])
    truth = trn["rover_truth"]["xyz_m"]
    estimated = trn["rover_estimated_xyz_m"]
    truth_px = local_to_pixel(truth[0], truth[1], px_to_m, ortho.shape)
    estimate_px = local_to_pixel(estimated[0], estimated[1], px_to_m, ortho.shape)

    scale_x = width / ortho.shape[1]
    scale_y = height / ortho.shape[0]
    panel = cv2.resize(color, (width, height), interpolation=cv2.INTER_AREA)
    text(panel, "TERRAIN RELATIVE NAV", (20, 38), scale=0.78, color=(238, 240, 236), thickness=2)

    if not nav_state["quality"]["position_lock"]:
        text(panel, "NO MAP POSITION LOCK", (20, 72), scale=0.54, color=(100, 190, 245), thickness=1)
        return panel

    truth_scaled = (int(round(truth_px[0] * scale_x)), int(round(truth_px[1] * scale_y)))
    estimate_scaled = (int(round(estimate_px[0] * scale_x)), int(round(estimate_px[1] * scale_y)))
    sigma_px = int(round(float(nav_state["quality"]["position_sigma_m"]) / px_to_m * (scale_x + scale_y) * 0.5))

    overlay = panel.copy()
    cv2.circle(overlay, estimate_scaled, max(sigma_px, 1), (70, 180, 255), -1, cv2.LINE_AA)
    panel = cv2.addWeighted(overlay, 0.20, panel, 0.80, 0.0)
    cv2.circle(panel, estimate_scaled, max(sigma_px, 1), (80, 210, 255), 2, cv2.LINE_AA)
    cv2.circle(panel, truth_scaled, 11, (0, 230, 255), 3, cv2.LINE_AA)
    cv2.circle(panel, estimate_scaled, 11, (100, 245, 140), 3, cv2.LINE_AA)
    cv2.line(panel, truth_scaled, estimate_scaled, (40, 190, 255), 2, cv2.LINE_AA)

    dominant = str(terms["dominant"]).replace("_", " ").upper()
    text(panel, f"sigma {nav_state['quality']['position_sigma_m']:.1f} m", (20, 72), scale=0.54, color=(175, 230, 185))
    text(panel, f"dominant: {dominant}", (20, 100), scale=0.5, color=(80, 220, 255))
    return panel


def render_timeline_panel(nav_state: dict, frame_index: int, *, size: tuple[int, int]) -> np.ndarray:
    width, height = size
    panel = np.full((height, width, 3), (26, 29, 34), dtype=np.uint8)
    text(panel, "NAVIGATION STATE REPLAY", (24, 42), scale=0.78, color=(245, 245, 240), thickness=2)
    text(panel, f"{nav_state['status']} / {nav_state['status_reason']}", (24, 82), scale=0.66, color=(210, 255, 220), thickness=2)
    text(panel, nav_state["message"], (24, 114), scale=0.5, color=(185, 198, 210))

    stages = [
        ("LOST", "NO_LOCKS"),
        ("DEGRADED", "ATTITUDE_ONLY"),
        ("OK", "NONE"),
    ]
    y = 140
    for index, (status, reason) in enumerate(stages):
        active = index == frame_index
        done = index < frame_index
        color = (92, 215, 125) if active else ((105, 160, 210) if done else (70, 78, 88))
        x = 60 + index * 190
        cv2.circle(panel, (x, y), 22, color, -1, cv2.LINE_AA)
        text(panel, str(index + 1), (x - 8, y + 8), scale=0.52, color=(20, 24, 28), thickness=2)
        text(panel, status, (x - 42, y + 58), scale=0.46, color=(235, 238, 236))
        text(panel, reason, (x - 68, y + 84), scale=0.36, color=(175, 190, 205))
        if index < 2:
            cv2.line(panel, (x + 28, y), (x + 162, y), (80, 95, 110), 2, cv2.LINE_AA)

    quality = nav_state["quality"]
    y2 = 74
    rows = [
        ("attitude", "LOCK" if quality["attitude_lock"] else "NO LOCK"),
        ("position", "LOCK" if quality["position_lock"] else "NO LOCK"),
        ("velocity", "LOCK" if quality["velocity_lock"] else "NO LOCK"),
    ]
    for idx, (label, value) in enumerate(rows):
        x = 700 + idx * 165
        text(panel, label.upper(), (x, y2), scale=0.42, color=(125, 175, 220))
        text(panel, value, (x, y2 + 28), scale=0.48, color=(236, 238, 236))
    return panel


def composite(star: np.ndarray, nav_map: np.ndarray, timeline: np.ndarray, *, size: tuple[int, int]) -> np.ndarray:
    width, height = size
    frame = np.full((height, width, 3), (10, 12, 15), dtype=np.uint8)
    gutter = 12
    top_h = 430
    left_w = 420
    right_w = width - left_w - gutter * 3
    bottom_h = height - top_h - gutter * 3
    frame[gutter:gutter + top_h, gutter:gutter + left_w] = cv2.resize(star, (left_w, top_h), interpolation=cv2.INTER_AREA)
    frame[gutter:gutter + top_h, gutter * 2 + left_w:gutter * 2 + left_w + right_w] = cv2.resize(nav_map, (right_w, top_h), interpolation=cv2.INTER_AREA)
    frame[gutter * 2 + top_h:gutter * 2 + top_h + bottom_h, gutter:width - gutter] = cv2.resize(timeline, (width - gutter * 2, bottom_h), interpolation=cv2.INTER_AREA)
    return frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-app", type=Path, default=REPO_ROOT / "build" / "apps" / "mission_navigation_demo")
    parser.add_argument("--trn-fixture-dir", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_lro_tycho_terminal")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "navigation_replay_demo.gif")
    parser.add_argument("--workdir", type=Path, default=REPO_ROOT / "outputs" / "navigation_replay_demo")
    parser.add_argument("--stars", type=int, default=30)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=23)
    args = parser.parse_args()

    if not args.nav_app.exists():
        raise SystemExit(f"missing navigation app: {args.nav_app} (run cmake --build build --parallel)")

    args.workdir.mkdir(parents=True, exist_ok=True)
    star_case = args.workdir / "star_case"
    run_capture([
        sys.executable,
        str(REPO_ROOT / "scripts" / "generate_star_tracker_case.py"),
        "--output-dir",
        str(star_case),
        "--stars",
        str(args.stars),
        "--noise-px",
        str(args.noise_px),
        "--seed",
        str(args.seed),
    ])

    summary_path = args.trn_fixture_dir / "summary.json"
    ortho_path = args.trn_fixture_dir / "ortho.png"
    if not summary_path.exists() or not ortho_path.exists():
        raise SystemExit(f"missing TRN fixture assets under {args.trn_fixture_dir}")

    trn = json.loads(summary_path.read_text(encoding="utf-8"))
    ortho = cv2.imread(str(ortho_path), cv2.IMREAD_GRAYSCALE)
    if ortho is None:
        raise SystemExit(f"failed to read {ortho_path}")

    degraded_state, ok_state = load_or_create_nav_states(args, star_case, summary_path)
    lost_state = make_lost_state()
    observations = read_observations(star_case / "observations.csv")
    terms = trn_quality_terms(trn)

    states = [lost_state, degraded_state, ok_state]
    frames: list[Image.Image] = []
    for index, state in enumerate(states):
        star_panel = render_star_panel(observations, state, size=(420, 430))
        map_panel = render_map_panel(ortho, trn, state, terms, size=(834, 430))
        timeline_panel = render_timeline_panel(state, index, size=(1256, 254))
        frame_bgr = composite(star_panel, map_panel, timeline_panel, size=(1280, 720))
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        # Duplicate frames so the state changes are readable without needing
        # metadata-dependent per-frame durations.
        frames.extend([Image.fromarray(frame_rgb)] * 7)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=130,
        loop=0,
    )

    summary = {
        "output": str(args.output),
        "states": [
            {"status": state["status"], "status_reason": state["status_reason"]}
            for state in states
        ],
        "position_sigma_m": ok_state["quality"]["position_sigma_m"],
        "trn_quality_terms": terms,
        "position_error_m": trn["position_error_m"],
    }
    summary_path_out = args.output.with_suffix(".json")
    summary_path_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
