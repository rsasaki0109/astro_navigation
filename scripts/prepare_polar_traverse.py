#!/usr/bin/env python3
"""Prepare a NASA POLAR Traverse sequence for lunar_visual_odometry."""

from __future__ import annotations

import argparse
import json
import re
import shlex
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


IMAGE_RE = re.compile(
    r"loc(?P<locid>\d+)[\s_]cam(?P<camera>[LR])[\s_](?P<exposure>\d+)ms\.png$"
)


@dataclass(frozen=True)
class PolarImage:
    path: Path
    locid: int
    camera: str
    exposure_ms: int
    traverse_root: Path


def parse_polar_image(path: Path) -> PolarImage | None:
    match = IMAGE_RE.match(path.name)
    if not match:
        return None
    return PolarImage(
        path=path,
        locid=int(match.group("locid")),
        camera=match.group("camera"),
        exposure_ms=int(match.group("exposure")),
        traverse_root=path.parent.parent,
    )


def position_sort_key(path: Path) -> tuple[float, str]:
    match = re.search(r"(\d+(?:\.\d+)?)m", path.parent.name)
    if match:
        return float(match.group(1)), path.name
    return 0.0, path.name


def read_opencv_matrix(yaml_path: Path, matrix_name: str) -> list[float] | None:
    if not yaml_path.exists():
        return None
    text = yaml_path.read_text(encoding="utf-8", errors="replace")
    pattern = re.compile(rf"{re.escape(matrix_name)}:\s*!!opencv-matrix.*?data:\s*\[([^\]]+)\]", re.S)
    match = pattern.search(text)
    if not match:
        return None
    return [float(value.strip()) for value in match.group(1).replace("\n", " ").split(",") if value.strip()]


def read_scalar(yaml_path: Path, name: str) -> float | None:
    text = yaml_path.read_text(encoding="utf-8", errors="replace")
    match = re.search(rf"^{re.escape(name)}:\s*([0-9.eE+-]+)\s*$", text, re.M)
    return float(match.group(1)) if match else None


def find_intrinsics(root: Path, camera: str) -> dict[str, float | str | list[float]]:
    stem = "left" if camera == "L" else "right"
    candidates = list(root.rglob(f"{stem} intrinsics.yml")) + list(root.rglob(f"{stem}_intrinsics.yml"))
    if not candidates:
        return {}

    path = candidates[0]
    data = read_opencv_matrix(path, "cameraMatrix") or read_opencv_matrix(path, "camera_matrix")
    if not data or len(data) < 9:
        return {"path": str(path)}
    result: dict[str, float | str | list[float]] = {
        "path": str(path),
        "fx": data[0],
        "fy": data[4],
        "cx": data[2],
        "cy": data[5],
        "camera_matrix": data[:9],
    }
    distortion = read_opencv_matrix(path, "distortion_coefficients")
    if distortion:
        result["distortion_coefficients"] = distortion
    width = read_scalar(path, "image_width")
    height = read_scalar(path, "image_height")
    if width is not None:
        result["image_width"] = width
    if height is not None:
        result["image_height"] = height
    return result


def find_extrinsics(root: Path) -> dict[str, float | str | list[float]]:
    candidates = list(root.rglob("extrinsics.yml"))
    if not candidates:
        return {}
    path = candidates[0]
    rotation = read_opencv_matrix(path, "rotation_matrix")
    translation = read_opencv_matrix(path, "translation_vector")
    result: dict[str, float | str | list[float]] = {"path": str(path)}
    if rotation and len(rotation) >= 9:
        result["R_right_left"] = rotation[:9]
    if translation and len(translation) >= 3:
        result["t_right_left"] = translation[:3]
        result["baseline_m"] = abs(translation[0])
    return result


def find_stereo_partner(left_image: PolarImage) -> Path | None:
    pattern = f"loc*_camR_{left_image.exposure_ms:03d}ms.png"
    candidates = sorted(left_image.path.parent.glob(pattern))
    if not candidates:
        pattern = f"loc*_camR_{left_image.exposure_ms}ms.png"
        candidates = sorted(left_image.path.parent.glob(pattern))
    return candidates[0] if candidates else None


def matrix3(values: list[float]):
    import numpy as np

    return np.asarray(values, dtype=float).reshape(3, 3)


def distortion(values: list[float] | None):
    import numpy as np

    if not values:
        return np.zeros((5, 1), dtype=float)
    return np.asarray(values, dtype=float).reshape(-1, 1)


def write_rectified_pairs(
    output: Path,
    pairs: list[tuple[Path, Path]],
    left_intrinsics: dict[str, float | str | list[float]],
    right_intrinsics: dict[str, float | str | list[float]],
    extrinsics: dict[str, float | str | list[float]],
) -> dict[str, float | str | list[float]]:
    import cv2
    import numpy as np

    if not pairs:
        return {}
    if "camera_matrix" not in left_intrinsics or "camera_matrix" not in right_intrinsics:
        return {}
    if "R_right_left" not in extrinsics or "t_right_left" not in extrinsics:
        return {}

    first_left = cv2.imread(str(pairs[0][0]), cv2.IMREAD_GRAYSCALE)
    if first_left is None:
        return {}
    image_size = (first_left.shape[1], first_left.shape[0])

    k_left = matrix3(left_intrinsics["camera_matrix"])  # type: ignore[arg-type]
    k_right = matrix3(right_intrinsics["camera_matrix"])  # type: ignore[arg-type]
    d_left = distortion(left_intrinsics.get("distortion_coefficients"))  # type: ignore[arg-type]
    d_right = distortion(right_intrinsics.get("distortion_coefficients"))  # type: ignore[arg-type]
    r_right_left = matrix3(extrinsics["R_right_left"])  # type: ignore[arg-type]
    t_right_left = np.asarray(extrinsics["t_right_left"], dtype=float).reshape(3, 1)  # type: ignore[arg-type]

    r1, r2, p1, p2, _q, _roi1, _roi2 = cv2.stereoRectify(
        k_left,
        d_left,
        k_right,
        d_right,
        image_size,
        r_right_left,
        t_right_left,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.0,
    )
    map1_left, map2_left = cv2.initUndistortRectifyMap(
        k_left, d_left, r1, p1, image_size, cv2.CV_16SC2
    )
    map1_right, map2_right = cv2.initUndistortRectifyMap(
        k_right, d_right, r2, p2, image_size, cv2.CV_16SC2
    )

    rectified_dir = output / "rectified"
    rectified_dir.mkdir(parents=True, exist_ok=True)
    pairs_csv = output / "stereo_pairs_rectified.csv"
    with pairs_csv.open("w", encoding="utf-8") as handle:
        handle.write("left,right\n")
        for index, (left_path, right_path) in enumerate(pairs):
            left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
            right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
            if left is None or right is None:
                continue
            rect_left = cv2.remap(left, map1_left, map2_left, cv2.INTER_LINEAR)
            rect_right = cv2.remap(right, map1_right, map2_right, cv2.INTER_LINEAR)
            out_left = rectified_dir / f"{index:04d}_left.png"
            out_right = rectified_dir / f"{index:04d}_right.png"
            cv2.imwrite(str(out_left), rect_left)
            cv2.imwrite(str(out_right), rect_right)
            handle.write(f"{out_left.resolve()},{out_right.resolve()}\n")

    baseline = abs(float(p2[0, 3] / p2[0, 0]))
    return {
        "stereo_pairs_rectified": str(pairs_csv),
        "rectified_dir": str(rectified_dir),
        "baseline_m": baseline,
        "left": {
            "fx": float(p1[0, 0]),
            "fy": float(p1[1, 1]),
            "cx": float(p1[0, 2]),
            "cy": float(p1[1, 2]),
            "camera_matrix": p1[:3, :3].reshape(-1).tolist(),
        },
        "right": {
            "fx": float(p2[0, 0]),
            "fy": float(p2[1, 1]),
            "cx": float(p2[0, 2]),
            "cy": float(p2[1, 2]),
            "camera_matrix": p2[:3, :3].reshape(-1).tolist(),
        },
    }


def read_pose_file(path: Path) -> dict[int, list[str]]:
    poses: dict[int, list[str]] = {}
    if not path.exists():
        return poses
    rows = [
        line.strip()
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
        if line.strip() and not line.startswith("#")
    ]
    for row in rows[1:]:
        fields = re.split(r"\s+", row)
        if len(fields) >= 8:
            try:
                locid = int(float(fields[0]))
            except ValueError:
                continue
            poses[locid] = fields
    return poses


def write_pose_subset(path: Path, images: list[PolarImage], pose_kind: str) -> Path | None:
    pose_dir = images[0].traverse_root / "poses"
    pose_file = pose_dir / f"{pose_kind} pose.txt"
    if not pose_file.exists():
        pose_file = pose_dir / f"{pose_kind}_pose.txt"
    poses = read_pose_file(pose_file)
    if not poses:
        return None

    output = path / f"{pose_kind}_poses.tsv"
    with output.open("w", encoding="utf-8") as handle:
        handle.write("LOCID\tX\tY\tZ\tQW\tQX\tQY\tQZ\n")
        for image in images:
            fields = poses.get(image.locid)
            if fields:
                handle.write("\t".join(fields[:8]) + "\n")
    return output


def choose_sequence(images: list[PolarImage], traverse_filter: str | None) -> list[PolarImage]:
    if traverse_filter:
        images = [image for image in images if traverse_filter in str(image.traverse_root)]
        if not images:
            raise RuntimeError(f"no images matched traverse filter: {traverse_filter}")

    groups: dict[Path, list[PolarImage]] = defaultdict(list)
    for image in images:
        groups[image.traverse_root].append(image)
    traverse_root, selected = max(groups.items(), key=lambda item: len(item[1]))
    selected.sort(key=lambda image: (position_sort_key(image.path), image.locid))
    print(f"selected traverse: {traverse_root} ({len(selected)} images)")
    return selected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="extracted POLAR Traverse root")
    parser.add_argument("--output", type=Path, default=Path("outputs/polar_sequence"))
    parser.add_argument("--camera", choices=["L", "R"], default="L")
    parser.add_argument("--exposure-ms", type=int, default=50)
    parser.add_argument("--traverse-filter", help="substring used to select one traverse")
    parser.add_argument("--pose-kind", choices=["refined", "approx"], default="refined")
    parser.add_argument("--rectify-stereo", action="store_true", help="write rectified stereo images")
    args = parser.parse_args()

    parsed_images = [parsed for path in args.root.rglob("*.png") if (parsed := parse_polar_image(path))]
    filtered = [
        image
        for image in parsed_images
        if image.camera == args.camera and image.exposure_ms == args.exposure_ms
    ]
    if not filtered:
        raise RuntimeError(
            f"no POLAR images found under {args.root} for camera {args.camera}, "
            f"exposure {args.exposure_ms} ms"
        )

    images = choose_sequence(filtered, args.traverse_filter)
    args.output.mkdir(parents=True, exist_ok=True)

    image_list = args.output / "images.txt"
    with image_list.open("w", encoding="utf-8") as handle:
        for image in images:
            handle.write(str(image.path.resolve()) + "\n")

    intrinsics = find_intrinsics(args.root, args.camera)
    right_intrinsics = find_intrinsics(args.root, "R")
    extrinsics = find_extrinsics(args.root)
    stereo_pairs = args.output / "stereo_pairs.csv"
    stereo_pair_count = 0
    stereo_pair_paths: list[tuple[Path, Path]] = []
    with stereo_pairs.open("w", encoding="utf-8") as handle:
        handle.write("left,right\n")
        for image in images:
            right = find_stereo_partner(image)
            if right:
                stereo_pair_count += 1
                stereo_pair_paths.append((image.path, right))
                handle.write(f"{image.path.resolve()},{right.resolve()}\n")

    pose_subset = write_pose_subset(args.output, images, args.pose_kind)
    rectified = {}
    if args.rectify_stereo:
        rectified = write_rectified_pairs(
            args.output, stereo_pair_paths, intrinsics, right_intrinsics, extrinsics
        )
    metadata = {
        "root": str(args.root),
        "camera": args.camera,
        "exposure_ms": args.exposure_ms,
        "image_count": len(images),
        "stereo_pair_count": stereo_pair_count,
        "traverse_root": str(images[0].traverse_root),
        "image_list": str(image_list),
        "stereo_pairs": str(stereo_pairs),
        "intrinsics": intrinsics,
        "right_intrinsics": right_intrinsics,
        "extrinsics": extrinsics,
        "rectified": rectified,
        "pose_subset": str(pose_subset) if pose_subset else None,
    }
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    if {"fx", "fy", "cx", "cy"}.issubset(intrinsics):
        command = [
            "build/apps/lunar_visual_odometry",
            "--images",
            str(image_list),
            "--fx",
            str(intrinsics["fx"]),
            "--fy",
            str(intrinsics["fy"]),
            "--cx",
            str(intrinsics["cx"]),
            "--cy",
            str(intrinsics["cy"]),
            "--feature",
            "orb",
            "--trajectory",
            str(args.output / "trajectory_orb.tum"),
        ]
        (args.output / "run_orb.sh").write_text(
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            + " ".join(shlex.quote(part) for part in command)
            + "\n",
            encoding="utf-8",
        )

    if {"fx", "fy", "cx", "cy"}.issubset(intrinsics) and {
        "fx",
        "fy",
        "cx",
        "cy",
    }.issubset(right_intrinsics) and "baseline_m" in extrinsics:
        command = [
            "build/apps/stereo_visual_odometry",
            "--pairs",
            str(stereo_pairs),
            "--fx",
            str(intrinsics["fx"]),
            "--fy",
            str(intrinsics["fy"]),
            "--cx",
            str(intrinsics["cx"]),
            "--cy",
            str(intrinsics["cy"]),
            "--right-fx",
            str(right_intrinsics["fx"]),
            "--right-fy",
            str(right_intrinsics["fy"]),
            "--right-cx",
            str(right_intrinsics["cx"]),
            "--right-cy",
            str(right_intrinsics["cy"]),
            "--baseline",
            str(extrinsics["baseline_m"]),
            "--trajectory",
            str(args.output / "trajectory_stereo_pnp.tum"),
        ]
        (args.output / "run_stereo_pnp.sh").write_text(
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            + " ".join(shlex.quote(part) for part in command)
            + "\n",
            encoding="utf-8",
        )

    if rectified and isinstance(rectified.get("left"), dict) and isinstance(rectified.get("right"), dict):
        left = rectified["left"]
        right = rectified["right"]
        command = [
            "build/apps/stereo_visual_odometry",
            "--pairs",
            str(rectified["stereo_pairs_rectified"]),
            "--fx",
            str(left["fx"]),
            "--fy",
            str(left["fy"]),
            "--cx",
            str(left["cx"]),
            "--cy",
            str(left["cy"]),
            "--right-fx",
            str(right["fx"]),
            "--right-fy",
            str(right["fy"]),
            "--right-cx",
            str(right["cx"]),
            "--right-cy",
            str(right["cy"]),
            "--baseline",
            str(rectified["baseline_m"]),
            "--max-stereo-y-diff",
            "10",
            "--min-disparity",
            "2",
            "--trajectory",
            str(args.output / "trajectory_stereo_pnp_rectified.tum"),
        ]
        (args.output / "run_stereo_pnp_rectified.sh").write_text(
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            + " ".join(shlex.quote(part) for part in command)
            + "\n",
            encoding="utf-8",
        )

    print(f"wrote {image_list}")
    print(f"wrote {stereo_pairs}")
    if rectified:
        print(f"wrote {rectified['stereo_pairs_rectified']}")
    print(f"wrote {args.output / 'metadata.json'}")
    if pose_subset:
        print(f"wrote {pose_subset}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
