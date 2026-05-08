# Interfaces

## Image Sequence Input

`lunar_visual_odometry` accepts:

- A directory containing image files
- A `.txt` or `.csv` file where the first comma-separated field is an image path
- A single image file for future single-frame tools

Images are sorted lexicographically. Dataset adapters should create explicit image-list files when a
dataset's native folder layout is not lexicographically ordered.

## POLAR Traverse Adapter

`scripts/prepare_polar_traverse.py` scans extracted NASA POLAR Traverse folders for files named:

```text
loc<LOCID> cam<L|R> <EXPOSURE>ms.png
```

It selects one traverse, writes an ordered `images.txt`, extracts OpenCV camera intrinsics when present,
copies the matching pose rows from `poses/refined pose.txt` or `poses/approx pose.txt`, and writes
`stereo_pairs.csv` when matching right-camera images are present.

## Stereo Pair Input

`stereo_visual_odometry` accepts a CSV file:

```text
left,right
/abs/path/left_000.png,/abs/path/right_000.png
```

The first stereo baseline uses ORB left-right matching, disparity/row filtering, triangulation, and PnP
RANSAC. `scripts/prepare_polar_traverse.py --rectify-stereo` writes rectified image pairs and a generated
`run_stereo_pnp_rectified.sh`. The C++ CLI currently accepts intrinsics plus baseline; full extrinsic YAML
loading is a planned adapter improvement.

## Camera Model

The MVP uses a pinhole camera model:

```text
fx fy cx cy
```

Distortion correction is expected to happen before images enter the VO executable. A later dataset
adapter can load calibration files and rectify images directly.

## Image Preprocessing

`lunar_visual_odometry` and `stereo_visual_odometry` support optional CLAHE preprocessing:

```text
--clahe --clahe-clip-limit 2.0 --clahe-tile-grid-size 8
```

Use this for high-shadow lunar analogue imagery where local contrast is poor. Keep it off for baseline
runs unless an experiment explicitly enables it.

## Trajectory Output

TUM format:

```text
timestamp tx ty tz qx qy qz qw
```

CSV format:

```text
timestamp,tx,ty,tz,qx,qy,qz,qw
```

`T_world_camera` is the camera pose in the initial camera frame. Monocular translation is relative
scale.

## Trajectory Evaluation

`scripts/evaluate_trajectory.py` compares TUM/CSV VO output against a POLAR pose subset TSV. Use
`--alignment sim3` for monocular trajectories and `--alignment se3` for metric stereo/depth trajectories
before reporting ATE and relative translation metrics.

## Module Boundaries

- `core`: shared types and image sequence loading
- `localization`: visual odometry and future pose estimation
- `terrain`: terrain landmarks and descriptors
- `crater`: crater candidate detection/matching
- `visualization`: trajectory and future map visualization
- `datasets`: downloaded public data and dataset-specific manifests
- `benchmarks`: reproducible experiment runners
