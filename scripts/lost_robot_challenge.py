#!/usr/bin/env python3
"""Render a one-shot "Lost Robot Challenge" mission card.

The challenge story:

  A lunar robot wakes up with no GNSS. It gets one star-camera frame and one
  nadir lunar camera frame. The card shows the two localization locks:

  - star tracker attitude from an identified synthetic star field;
  - terrain-relative position from a bundled real LRO/LOLA Tycho fixture;
  - a C++ `mission_navigation_demo` state that fuses those locks into
    navigation health.

The default path is deliberately offline-friendly: no HYG catalog, pair index,
or LRO download is required. It uses the same smoke-test star fixture as CI and
the already committed TRN fixture under docs/figures/trn_lro_tycho_terminal.
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


def star_estimate_from_nav_state(nav_state: dict) -> dict:
    quality = nav_state["quality"]
    return {
        "success": bool(quality["attitude_lock"]),
        "correspondences": int(quality["attitude_correspondences"]),
        "rms_direction_error_rad": float(quality["attitude_sigma_rad"]),
        "q_xyzw": [float(value) for value in nav_state["q_body_reference_xyzw"]],
        "status": nav_state["status"],
    }


def quat_angle_deg(a: list[float], b: list[float]) -> float:
    qa = np.asarray(a, dtype=float)
    qb = np.asarray(b, dtype=float)
    qa = qa / np.linalg.norm(qa)
    qb = qb / np.linalg.norm(qb)
    dot = float(abs(np.dot(qa, qb)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def render_star_panel(
    observations: list[tuple[str, float, float]],
    estimate: dict,
    truth: dict,
    *,
    size: tuple[int, int],
) -> np.ndarray:
    width, height = size
    panel = np.full((height, width, 3), (14, 18, 26), dtype=np.uint8)

    # Deterministic sparse star-field look: draw the solved observations and a
    # small camera reticle, but keep it abstract enough to read as telemetry.
    sx = width / 1024.0
    sy = height / 1024.0
    for index, (_star_id, u, v) in enumerate(observations):
        x = int(round(u * sx))
        y = int(round(v * sy))
        radius = 3 + (index % 3)
        cv2.circle(panel, (x, y), radius, (235, 235, 220), -1, cv2.LINE_AA)
        cv2.circle(panel, (x, y), radius + 6, (60, 190, 120), 1, cv2.LINE_AA)

    center = (width // 2, height // 2)
    cv2.line(panel, (center[0] - 34, center[1]), (center[0] + 34, center[1]), (90, 110, 130), 1)
    cv2.line(panel, (center[0], center[1] - 34), (center[0], center[1] + 34), (90, 110, 130), 1)
    cv2.circle(panel, center, 48, (90, 110, 130), 1, cv2.LINE_AA)

    angle_error = quat_angle_deg(estimate["q_xyzw"], truth["q_camera_inertial_xyzw"])
    text(panel, "STAR CAMERA LOCK", (24, 42), scale=0.82, color=(235, 238, 240), thickness=2)
    text(
        panel,
        f"{estimate['correspondences']} identified stars",
        (24, 78),
        scale=0.55,
        color=(170, 225, 180),
    )
    text(
        panel,
        f"attitude error {angle_error:.4f} deg",
        (24, 104),
        scale=0.55,
        color=(170, 225, 180),
    )
    qx, qy, qz, qw = estimate["q_xyzw"]
    text(panel, "q camera->inertial", (24, height - 74), scale=0.5, color=(175, 190, 205))
    text(panel, f"[{qx:+.3f}, {qy:+.3f}, {qz:+.3f}, {qw:+.3f}]", (24, height - 46), scale=0.5, color=(235, 238, 240))
    return panel


def local_to_pixel(x_m: float, y_m: float, px_to_m: float, image_shape: tuple[int, int]) -> tuple[int, int]:
    height, width = image_shape
    x = int(round(x_m / px_to_m))
    y = int(round(y_m / px_to_m))
    return max(0, min(width - 1, x)), max(0, min(height - 1, y))


def render_map_panel(ortho: np.ndarray, trn: dict, *, size: tuple[int, int]) -> np.ndarray:
    width, height = size
    color = cv2.cvtColor(ortho, cv2.COLOR_GRAY2BGR)
    px_to_m = float(trn["wac"]["px_to_m"])
    truth = trn["rover_truth"]["xyz_m"]
    estimated = trn["rover_estimated_xyz_m"]
    truth_px = local_to_pixel(truth[0], truth[1], px_to_m, ortho.shape)
    estimate_px = local_to_pixel(estimated[0], estimated[1], px_to_m, ortho.shape)

    cv2.circle(color, truth_px, 16, (0, 230, 255), 3, cv2.LINE_AA)
    cv2.circle(color, estimate_px, 16, (80, 240, 120), 3, cv2.LINE_AA)
    cv2.line(color, truth_px, estimate_px, (40, 190, 255), 2, cv2.LINE_AA)
    text(color, "truth", (truth_px[0] + 18, truth_px[1] - 10), scale=0.55, color=(0, 230, 255))
    text(color, "recovered", (estimate_px[0] + 18, estimate_px[1] + 22), scale=0.55, color=(120, 255, 150))

    panel = cv2.resize(color, (width, height), interpolation=cv2.INTER_AREA)
    text(panel, "LRO/LOLA MAP LOCK", (20, 38), scale=0.8, color=(235, 238, 240), thickness=2)
    text(
        panel,
        f"Tycho terminal descent - {trn['position_error_m']:.1f} m position error",
        (20, 70),
        scale=0.52,
        color=(175, 230, 185),
    )
    return panel


def render_rover_panel(rover: np.ndarray, trn: dict, *, size: tuple[int, int]) -> np.ndarray:
    width, height = size
    color = cv2.cvtColor(rover, cv2.COLOR_GRAY2BGR)
    panel = cv2.resize(color, (width, height), interpolation=cv2.INTER_AREA)
    text(panel, "NADIR CAMERA FRAME", (20, 38), scale=0.78, color=(235, 238, 240), thickness=2)
    text(
        panel,
        f"{trn['match_count']} SIFT matches, {trn['pnp']['pnp_inliers']} AP3P inliers",
        (20, 70),
        scale=0.52,
        color=(175, 230, 185),
    )
    return panel


def render_hud(
    *,
    star_estimate: dict,
    star_truth: dict,
    trn: dict,
    nav_state: dict,
    size: tuple[int, int],
) -> np.ndarray:
    width, height = size
    panel = np.full((height, width, 3), (28, 31, 35), dtype=np.uint8)
    angle_error = quat_angle_deg(star_estimate["q_xyzw"], star_truth["q_camera_inertial_xyzw"])

    text(panel, "LOST ROBOT CHALLENGE", (24, 46), scale=0.92, color=(245, 245, 240), thickness=2)
    text(panel, "One star frame + one lunar frame -> attitude and position", (24, 82), scale=0.5, color=(185, 198, 210))

    y = 130
    rows = [
        ("NAV STATE", f"{nav_state['status']} - {nav_state['message']}"),
        ("ATTITUDE", f"{star_estimate['correspondences']} stars, {angle_error:.4f} deg quaternion error"),
        ("SURFACE", "Tycho central peak, real LRO WAC + LOLA fixture"),
        ("ALTITUDE", f"{trn['rover_truth']['xyz_m'][2] / 1000.0:.1f} km"),
        ("POSITION", f"{trn['position_error_m']:.1f} m error"),
        ("MATCHING", f"{trn['match_count']} matches, {trn['pnp']['pnp_inliers']} PnP inliers"),
    ]
    for label, value in rows:
        text(panel, label, (24, y), scale=0.48, color=(125, 175, 220))
        text(panel, value, (150, y), scale=0.52, color=(236, 238, 236))
        y += 38

    localized = nav_state["status"] == "OK"
    status = "NAVIGATION LOCK" if localized else f"NAVIGATION {nav_state['status']}"
    fill = (42, 92, 62) if localized else (82, 72, 36)
    stroke = (105, 220, 135) if localized else (90, 190, 235)
    cv2.rectangle(panel, (24, height - 76), (width - 24, height - 24), fill, -1)
    cv2.rectangle(panel, (24, height - 76), (width - 24, height - 24), stroke, 2)
    text(panel, status, (44, height - 42), scale=0.75, color=(210, 255, 220), thickness=2)
    return panel


def composite(
    *,
    star_panel: np.ndarray,
    rover_panel: np.ndarray,
    map_panel: np.ndarray,
    hud_panel: np.ndarray,
    size: tuple[int, int],
) -> np.ndarray:
    width, height = size
    frame = np.full((height, width, 3), (10, 12, 15), dtype=np.uint8)
    gutter = 10
    left_w = 520
    right_w = width - left_w - gutter * 3
    top_h = 355
    bottom_h = height - top_h - gutter * 3

    frame[gutter:gutter + top_h, gutter:gutter + left_w] = cv2.resize(
        star_panel, (left_w, top_h), interpolation=cv2.INTER_AREA
    )
    frame[gutter:gutter + top_h, gutter * 2 + left_w:gutter * 2 + left_w + right_w] = cv2.resize(
        rover_panel, (right_w, top_h), interpolation=cv2.INTER_AREA
    )
    frame[gutter * 2 + top_h:gutter * 2 + top_h + bottom_h, gutter:gutter + left_w] = cv2.resize(
        hud_panel, (left_w, bottom_h), interpolation=cv2.INTER_AREA
    )
    frame[
        gutter * 2 + top_h:gutter * 2 + top_h + bottom_h,
        gutter * 2 + left_w:gutter * 2 + left_w + right_w,
    ] = cv2.resize(map_panel, (right_w, bottom_h), interpolation=cv2.INTER_AREA)
    return frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "docs" / "figures" / "lost_robot_challenge.png")
    parser.add_argument("--summary-output", type=Path, default=None)
    parser.add_argument("--workdir", type=Path, default=REPO_ROOT / "outputs" / "lost_robot_challenge")
    parser.add_argument("--nav-app", type=Path, default=REPO_ROOT / "build" / "apps" / "mission_navigation_demo")
    parser.add_argument("--trn-fixture-dir", type=Path, default=REPO_ROOT / "docs" / "figures" / "trn_lro_tycho_terminal")
    parser.add_argument("--stars", type=int, default=30)
    parser.add_argument("--noise-px", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=23)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    if not args.nav_app.exists():
        raise SystemExit(f"missing navigation app: {args.nav_app} (run cmake --build build --parallel)")

    args.workdir.mkdir(parents=True, exist_ok=True)
    run_capture([
        sys.executable,
        str(REPO_ROOT / "scripts" / "generate_star_tracker_case.py"),
        "--output-dir",
        str(args.workdir / "star_case"),
        "--stars",
        str(args.stars),
        "--noise-px",
        str(args.noise_px),
        "--seed",
        str(args.seed),
    ])

    star_case = args.workdir / "star_case"
    ortho_path = args.trn_fixture_dir / "ortho.png"
    rover_path = args.trn_fixture_dir / "rover.png"
    summary_path = args.trn_fixture_dir / "summary.json"
    ortho = cv2.imread(str(ortho_path), cv2.IMREAD_GRAYSCALE)
    rover = cv2.imread(str(rover_path), cv2.IMREAD_GRAYSCALE)
    if ortho is None or rover is None or not summary_path.exists():
        raise SystemExit(f"missing TRN fixture assets under {args.trn_fixture_dir}")
    trn = json.loads(summary_path.read_text(encoding="utf-8"))

    nav_state_path = args.workdir / "nav_state.json"
    run_capture([
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
        "--trn-summary",
        str(summary_path),
        "--output-json",
        str(nav_state_path),
    ])
    nav_state = json.loads(nav_state_path.read_text(encoding="utf-8"))
    star_estimate = star_estimate_from_nav_state(nav_state)
    star_truth = json.loads((star_case / "truth.json").read_text(encoding="utf-8"))
    observations = read_observations(star_case / "observations.csv")

    star_panel = render_star_panel(observations, star_estimate, star_truth, size=(560, 560))
    rover_panel = render_rover_panel(rover, trn, size=(720, 355))
    map_panel = render_map_panel(ortho, trn, size=(720, 335))
    hud_panel = render_hud(
        star_estimate=star_estimate,
        star_truth=star_truth,
        trn=trn,
        nav_state=nav_state,
        size=(560, 335),
    )
    frame = composite(
        star_panel=star_panel,
        rover_panel=rover_panel,
        map_panel=map_panel,
        hud_panel=hud_panel,
        size=(args.width, args.height),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), frame)

    summary = {
        "output": str(args.output),
        "star_tracker": {
            **star_estimate,
            "truth_q_xyzw": star_truth["q_camera_inertial_xyzw"],
            "attitude_error_deg": quat_angle_deg(
                star_estimate["q_xyzw"], star_truth["q_camera_inertial_xyzw"]
            ),
        },
        "navigation": {
            "nav_state_json": str(nav_state_path),
            "status": nav_state["status"],
            "status_reason": nav_state["status_reason"],
            "message": nav_state["message"],
            "position_m": nav_state["position_m"],
            "position_frame_id": nav_state["position_frame_id"],
            "q_body_reference_xyzw": nav_state["q_body_reference_xyzw"],
            "quality": nav_state["quality"],
        },
        "trn": {
            "fixture_dir": str(args.trn_fixture_dir),
            "target": trn["target"],
            "position_error_m": trn["position_error_m"],
            "match_count": trn["match_count"],
            "pnp_inliers": trn["pnp"]["pnp_inliers"],
            "altitude_m": trn["rover_truth"]["xyz_m"][2],
        },
    }
    summary_output = args.summary_output or args.output.with_suffix(".json")
    summary_output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
