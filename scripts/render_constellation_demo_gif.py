#!/usr/bin/env python3
"""Render a polished lost-in-space identification GIF with constellation overlays.

For each preset attitude — picked so the boresight lands on a recognisable
constellation (Orion, Big Dipper, Leo, Cygnus+Lyra, Cassiopeia, Scorpius) —
the script chains the existing pipeline:

    render_star_image.py  ->  centroid_stars_from_image.py  ->
        apps/lost_in_space_pair_id  (C++ identifier on the .bin index)

then composites:

    - cyan constellation stick-figure lines between identified stars belonging
      to the same well-known asterism;
    - gold proper-name labels on bright identified stars (Sirius, Vega, etc.);
    - small green/red rings on every centroid (correct/wrong) so the
      identification is honestly scored against the per-frame truth.

The frames are stitched into a looping GIF for the README. Per-frame cost is
dominated by `lost_in_space_pair_id` loading the .bin (~1-2 s for the 16k
mag<=6.5 catalog), so 6 famous-constellation frames is the sweet spot.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parent.parent


# Bright-star registry — catalog id (string) -> (proper name, IAU tag).
# scripts/convert_star_catalog.py uses `proper` as the catalog id when present
# (falling back to hip / hd / hr / numeric id), so famous stars appear here as
# their proper name — same string the C++ identifier emits in assignments.
NAMED_STARS: dict[str, tuple[str, str]] = {
    # Orion
    "Betelgeuse": ("Betelgeuse", "Ori"),
    "Bellatrix":  ("Bellatrix",  "Ori"),
    "Mintaka":    ("Mintaka",    "Ori"),
    "Alnilam":    ("Alnilam",    "Ori"),
    "Alnitak":    ("Alnitak",    "Ori"),
    "Saiph":      ("Saiph",      "Ori"),
    "Rigel":      ("Rigel",      "Ori"),
    # Big Dipper (UMa subset)
    "Dubhe":      ("Dubhe",      "UMa"),
    "Merak":      ("Merak",      "UMa"),
    "Phecda":     ("Phecda",     "UMa"),
    "Megrez":     ("Megrez",     "UMa"),
    "Alioth":     ("Alioth",     "UMa"),
    "Mizar":      ("Mizar",      "UMa"),
    "Alkaid":     ("Alkaid",     "UMa"),
    # Leo
    "Regulus":    ("Regulus",    "Leo"),
    "Algieba":    ("Algieba",    "Leo"),
    "Denebola":   ("Denebola",   "Leo"),
    # Cassiopeia W (Gamma Cas has no proper name in HYG, so the W is split
    # into two pieces — Caph-Schedar-Ruchbah, Ruchbah-Segin)
    "Caph":       ("Caph",       "Cas"),
    "Schedar":    ("Schedar",    "Cas"),
    "Ruchbah":    ("Ruchbah",    "Cas"),
    "Segin":      ("Segin",      "Cas"),
    # Cygnus Northern Cross (subset)
    "Deneb":      ("Deneb",      "Cyg"),
    "Sadr":       ("Sadr",       "Cyg"),
    "Albireo":    ("Albireo",    "Cyg"),
    # Lyra triangle
    "Vega":       ("Vega",       "Lyr"),
    "Sheliak":    ("Sheliak",    "Lyr"),
    "Sulafat":    ("Sulafat",    "Lyr"),
    # Aquila
    "Altair":     ("Altair",     "Aql"),
    "Tarazed":    ("Tarazed",    "Aql"),
    "Alshain":    ("Alshain",    "Aql"),
    # Scorpius head/tail
    "Antares":    ("Antares",    "Sco"),
    "Dschubba":   ("Dschubba",   "Sco"),
    "Acrab":      ("Acrab",      "Sco"),
    "Shaula":     ("Shaula",     "Sco"),
    "Lesath":     ("Lesath",     "Sco"),
    "Sargas":     ("Sargas",     "Sco"),
    # Other well-known landmarks (labelled if visible, no constellation lines)
    "Sirius":     ("Sirius",     "CMa"),
    "Procyon":    ("Procyon",    "CMi"),
    "Aldebaran":  ("Aldebaran",  "Tau"),
    "Capella":    ("Capella",    "Aur"),
    "Polaris":    ("Polaris",    "UMi"),
    "Spica":      ("Spica",      "Vir"),
    "Arcturus":   ("Arcturus",   "Boo"),
    "Castor":     ("Castor",     "Gem"),
    "Pollux":     ("Pollux",     "Gem"),
}


CONSTELLATION_LINES: dict[str, list[tuple[str, str]]] = {
    "Ori": [
        ("Betelgeuse", "Bellatrix"),  # shoulders
        ("Bellatrix",  "Mintaka"),
        ("Mintaka",    "Alnilam"),    # belt
        ("Alnilam",    "Alnitak"),    # belt
        ("Alnitak",    "Saiph"),
        ("Saiph",      "Rigel"),      # legs
        ("Rigel",      "Mintaka"),
        ("Betelgeuse", "Alnitak"),
    ],
    "UMa": [
        ("Dubhe",  "Merak"),    # front of bowl
        ("Merak",  "Phecda"),   # bottom
        ("Phecda", "Megrez"),   # back
        ("Megrez", "Dubhe"),    # top of bowl
        ("Megrez", "Alioth"),   # handle starts
        ("Alioth", "Mizar"),
        ("Mizar",  "Alkaid"),
    ],
    "Leo": [
        ("Regulus",  "Algieba"),
        ("Algieba",  "Denebola"),
        ("Regulus",  "Denebola"),
    ],
    "Cas": [
        ("Caph",    "Schedar"),
        ("Schedar", "Ruchbah"),
        ("Ruchbah", "Segin"),
    ],
    "Cyg": [
        ("Deneb",  "Sadr"),
        ("Sadr",   "Albireo"),
    ],
    "Lyr": [
        ("Vega",    "Sheliak"),
        ("Sheliak", "Sulafat"),
        ("Sulafat", "Vega"),
    ],
    "Aql": [
        ("Altair", "Tarazed"),
        ("Altair", "Alshain"),
    ],
    "Sco": [
        ("Antares",  "Dschubba"),
        ("Dschubba", "Acrab"),
        ("Antares",  "Shaula"),
        ("Shaula",   "Lesath"),
        ("Shaula",   "Sargas"),
    ],
}


# Six attitudes whose boresight lands on a recognisable target. Convention is
# the Tait-Bryan triple consumed by render_star_image.py: rotation_from_euler
# returns Rz(yaw) @ Ry(pitch) @ Rx(roll) and uses it as R_camera_inertial.
#
# Working out which (yaw, pitch, roll) lands on a desired (RA, Dec) is less
# obvious than it looks: with this Rz-Ry-Rx composition the camera +Z in
# inertial frame is the third row of R, namely
#   (-sin(pitch), cos(pitch) sin(roll), cos(pitch) cos(roll))
# which is INDEPENDENT of yaw — yaw only field-rotates the image plane around
# the optical axis. So for boresight at (RA, Dec):
#   pitch_deg = arcsin(-cos(Dec) * cos(RA))   in degrees
#   roll_deg  = atan2(cos(Dec) * sin(RA), sin(Dec))   in degrees
#   yaw_deg   = field rotation, free choice
# (Spent a debug pass converging from "yaw_deg = RA_deg" -> "yaw_deg = 180 - RA_deg"
# -> the formulas above. Both earlier guesses landed on the wrong side of the
# sky because they confused yaw / roll: with roll left at 0, yaw is degenerate.)
FAMOUS_ATTITUDES: list[tuple[str, float, float, float]] = [
    ("Orion",          0.0,  -5.9,  91.2),   # Alnilam: RA 84.05,  Dec -1.20
    ("Big Dipper",     0.0,  32.9,  -2.5),   # Megrez:  RA 183.86, Dec 57.03
    ("Cygnus + Lyra",  0.0, -18.3, -45.2),   # midpoint Vega/Deneb: RA 295, Dec 42
    ("Cassiopeia",     0.0, -32.9,   6.6),   # Schedar: RA 10.13,  Dec 56.54
    ("Leo",            0.0,  58.5,  49.5),   # Algieba: RA 154.99, Dec 19.84
    ("Scorpius",       0.0,  20.1, -118.3),  # Antares: RA 247.40, Dec -26.40
]


def run(cmd: list[str]) -> None:
    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(f"$ {' '.join(cmd)}\n{result.stdout}\n{result.stderr}\n")
        raise SystemExit(result.returncode)


def load_truth(truth_csv: Path) -> list[tuple[str, float, float]]:
    out: list[tuple[str, float, float]] = []
    with truth_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            out.append((row["id"], float(row["u"]), float(row["v"])))
    return out


def load_observations(obs_csv: Path) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    with obs_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            out.append((float(row["u"]), float(row["v"])))
    return out


def load_assignments(assignments_csv: Path) -> dict[int, str]:
    out: dict[int, str] = {}
    with assignments_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            out[int(row["observation_index"])] = row["id"]
    return out


def closest_truth(u: float, v: float, truth: list[tuple[str, float, float]],
                  match_px: float) -> str | None:
    best_id: str | None = None
    best_d2 = match_px * match_px
    for star_id, tu, tv in truth:
        d2 = (u - tu) ** 2 + (v - tv) ** 2
        if d2 <= best_d2:
            best_d2 = d2
            best_id = star_id
    return best_id


def text_with_shadow(image: np.ndarray, text: str, origin: tuple[int, int],
                     scale: float, color: tuple[int, int, int],
                     thickness: int = 2) -> None:
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)


def annotate_frame(
    image: np.ndarray,
    observations: list[tuple[float, float]],
    assignments: dict[int, str],
    truth: list[tuple[str, float, float]],
    match_px: float,
    target_label: str,
    yaw: float, pitch: float, roll: float,
    show_star_names: bool,
    show_constellations: bool,
) -> tuple[np.ndarray, dict[str, int]]:
    out = image.copy()

    # Build a (truth_id) -> (u, v) lookup so constellation-line drawing only
    # needs the catalog id, not the pixel position.
    truth_lookup_by_id: dict[str, tuple[float, float]] = {
        sid: (u, v) for sid, u, v in truth}

    # Identify which observations were correctly matched and what catalog id
    # they landed on. Only correct (non-wrong) matches feed the constellation
    # lines and labels — the demo's whole point is that the identification IS
    # what enables the overlay.
    correct_id_pixel: dict[str, tuple[float, float]] = {}
    counts = {"correct": 0, "wrong": 0, "unassigned": 0}

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

    # Draw constellation stick-figures (cyan) between correctly identified
    # stars in the same constellation. Only emit the line if BOTH endpoints
    # were identified — never use truth positions as a fallback, since the
    # demo's claim is "what you see is what the identifier produced."
    if show_constellations:
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

    # Centroid rings: green for correct, red for wrong, faint blue for
    # unassigned. Slightly smaller than the legacy demo's rings so the
    # constellation lines + labels read cleanly through them.
    for obs_index, (u, v) in enumerate(observations):
        truth_id = closest_truth(u, v, truth, match_px)
        assigned = assignments.get(obs_index)
        if assigned is None:
            color = (220, 180, 80)
            radius = 8
        elif truth_id is not None and assigned == truth_id:
            color = (90, 220, 120)
            radius = 9
        else:
            color = (60, 60, 240)
            radius = 9
        cv2.circle(out, (int(round(u)), int(round(v))), radius=radius,
                   color=color, thickness=2, lineType=cv2.LINE_AA)

    # Star-name labels on identified bright stars (mag prior already encoded
    # in NAMED_STARS membership — we only label landmarks worth naming).
    if show_star_names:
        for star_id, (uv) in correct_id_pixel.items():
            entry = NAMED_STARS.get(star_id)
            if entry is None:
                continue
            name, _con = entry
            u, v = uv
            text_with_shadow(
                out, name,
                (int(round(u)) + 14, int(round(v)) - 8),
                scale=0.55, color=(80, 220, 255), thickness=1,
            )

    # Header: target asterism name.
    text_with_shadow(out, f"Looking at: {target_label}",
                     (16, 36), scale=0.95, color=(240, 240, 240), thickness=2)
    # Sub-header: attitude triple.
    text_with_shadow(
        out,
        f"yaw {yaw:+.0f} deg   pitch {pitch:+.0f} deg   roll {roll:+.0f} deg",
        (16, 64), scale=0.55, color=(200, 200, 200), thickness=1,
    )
    # Footer: per-frame correctness scoreboard.
    legend = (
        f"correct {counts['correct']}   wrong {counts['wrong']}   "
        f"unassigned {counts['unassigned']}"
    )
    text_with_shadow(out, legend, (16, image.shape[0] - 18),
                     scale=0.65, color=(240, 240, 240), thickness=1)
    return out, counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--index-bin", type=Path, required=True)
    parser.add_argument("--identifier-bin", type=Path,
                        default=REPO_ROOT / "build" / "apps" / "lost_in_space_pair_id")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fx", type=float, default=900.0,
                        help="Slightly wider FOV than render_lost_in_space_gif's 1000 so an "
                        "entire asterism (e.g. Orion at ~25 deg) fits in one frame.")
    parser.add_argument("--fy", type=float, default=900.0)
    parser.add_argument("--cx", type=float, default=512.0)
    parser.add_argument("--cy", type=float, default=512.0)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--workdir", type=Path, default=Path("/tmp/constellation_demo_gif"))
    parser.add_argument("--match-px", type=float, default=4.0)
    parser.add_argument("--ms-per-frame", type=int, default=1500)
    parser.add_argument("--tolerance-arcsec", type=float, default=120.0)
    parser.add_argument("--neighbor-bins", type=int, default=1)
    parser.add_argument("--verification-tolerance-arcsec", type=float, default=600.0)
    parser.add_argument("--magnitude-prior-arcsec", type=float, default=15.0)
    parser.add_argument("--pyramid-size", type=int, default=6)
    parser.add_argument("--pyramid-restarts", type=int, default=3)
    parser.add_argument("--confidence-fraction", type=float, default=0.5)
    parser.add_argument("--no-star-names", dest="show_star_names",
                        action="store_false", default=True)
    parser.add_argument("--no-constellations", dest="show_constellations",
                        action="store_false", default=True)
    args = parser.parse_args()

    args.workdir.mkdir(parents=True, exist_ok=True)

    frames_pil: list[Image.Image] = []
    summary_rows: list[str] = []
    for frame_idx, (target, yaw, pitch, roll) in enumerate(FAMOUS_ATTITUDES):
        frame_dir = args.workdir / f"frame_{frame_idx:02d}_{target.replace(' ', '_').lower()}"
        frame_dir.mkdir(parents=True, exist_ok=True)
        image_path = frame_dir / "exposure.png"
        truth_path = frame_dir / "truth.csv"
        obs_path = frame_dir / "observations.csv"
        assignments_path = frame_dir / "assignments.csv"

        run([
            sys.executable, str(REPO_ROOT / "scripts" / "render_star_image.py"),
            "--catalog", str(args.catalog),
            "--output-image", str(image_path),
            "--output-truth", str(truth_path),
            "--fx", str(args.fx), "--fy", str(args.fy),
            "--cx", str(args.cx), "--cy", str(args.cy),
            "--width", str(args.width), "--height", str(args.height),
            "--yaw-deg", str(yaw), "--pitch-deg", str(pitch), "--roll-deg", str(roll),
            "--seed", str(args.seed + frame_idx),
        ])
        run([
            sys.executable, str(REPO_ROOT / "scripts" / "centroid_stars_from_image.py"),
            "--input-image", str(image_path),
            "--output-observations", str(obs_path),
        ])
        run([
            str(args.identifier_bin),
            "--observations", str(obs_path),
            "--index", str(args.index_bin),
            "--output", str(assignments_path),
            "--fx", str(args.fx), "--fy", str(args.fy),
            "--cx", str(args.cx), "--cy", str(args.cy),
            "--tolerance-arcsec", str(args.tolerance_arcsec),
            "--neighbor-bins", str(args.neighbor_bins),
            "--verification-tolerance-arcsec", str(args.verification_tolerance_arcsec),
            "--magnitude-prior-arcsec", str(args.magnitude_prior_arcsec),
            "--pyramid-size", str(args.pyramid_size),
            "--pyramid-restarts", str(args.pyramid_restarts),
            "--confidence-fraction", str(args.confidence_fraction),
        ])

        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f"could not read rendered image: {image_path}")
        truth = load_truth(truth_path)
        observations = load_observations(obs_path)
        assignments = load_assignments(assignments_path)
        annotated, counts = annotate_frame(
            image, observations, assignments, truth, args.match_px,
            target_label=target, yaw=yaw, pitch=pitch, roll=roll,
            show_star_names=args.show_star_names,
            show_constellations=args.show_constellations,
        )
        msg = (f"frame {frame_idx + 1}/{len(FAMOUS_ATTITUDES)} ({target}): "
               f"yaw={yaw:+.1f} pitch={pitch:+.1f} roll={roll:+.1f}  "
               f"correct={counts['correct']} wrong={counts['wrong']} "
               f"unassigned={counts['unassigned']}  "
               f"observations={len(observations)} truth={len(truth)}")
        print(msg)
        summary_rows.append(msg)
        frames_pil.append(Image.fromarray(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)))

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
