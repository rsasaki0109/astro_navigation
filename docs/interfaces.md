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

## Navigation State Output

`mission_navigation_demo --output-json nav_state.json` writes a mission-facing navigation state.
The JSON contract is documented in [`docs/schemas/nav_state.schema.json`](schemas/nav_state.schema.json).

Required top-level fields:

- `timestamp`: seconds
- `status`: one of `UNKNOWN`, `OK`, `DEGRADED`, `LOST`, `RELOCALIZING`
- `status_reason`: one of `NONE`, `NO_LOCKS`, `ATTITUDE_ONLY`, `POSITION_ONLY`, `VELOCITY_MISSING`,
  `HIGH_ATTITUDE_UNCERTAINTY`, `HIGH_POSITION_UNCERTAINTY`
- `message`: human-readable status message
- `position_frame_id`: frame for `position_m` and `velocity_mps`
- `attitude_reference_frame_id`: frame for `q_body_reference_xyzw`
- `position_m`: `[x, y, z]`
- `velocity_mps`: `[vx, vy, vz]`
- `q_body_reference_xyzw`: quaternion `[x, y, z, w]`
- `quality`: lock flags, sigma values, and correspondence counts
- `covariance_6x6`: pose covariance ordered as `x, y, z, roll, pitch, yaw`

For TRN-derived position locks, `position_sigma_m` is a truth-free conservative estimate. It is
currently the maximum of:

- `2.0 * wac.px_to_m`
- `pnp.inlier_median_reproj_px * wac.px_to_m`
- `wac.px_to_m * sqrt(12 / max(pnp.pnp_inliers, 1))`

The Tycho terminal fixture has terms `143.956 m`, `6.911 m`, and `75.179 m`, so the navigation
sigma is `143.956 m`. The fixture's `position_error_m` remains an evaluation metric and is not used
as the navigation sigma.

Example:

```json
{
  "timestamp": 0.0,
  "status": "OK",
  "status_reason": "NONE",
  "message": "navigation lock",
  "position_frame_id": "map",
  "attitude_reference_frame_id": "inertial",
  "position_m": [46069.113087615, 46097.730733616, 30000.596809296],
  "velocity_mps": [0.0, 0.0, 0.0],
  "q_body_reference_xyzw": [0.041021085, -0.057040183, 0.106367638, 0.991841527],
  "quality": {
    "attitude_lock": true,
    "position_lock": true,
    "velocity_lock": false,
    "attitude_sigma_rad": 0.00025164,
    "position_sigma_m": 143.956140950,
    "attitude_correspondences": 30
  },
  "covariance_6x6": [
    [20723.372490099, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 20723.372490099, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 20723.372490099, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.000000063, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.000000063, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.000000063]
  ]
}
```

`mission_navigation_demo --output-csv nav_state.csv` writes one flat row:

```text
timestamp,status,status_reason,attitude_lock,position_lock,attitude_correspondences,attitude_sigma_rad,position_sigma_m,frame,x,y,z,qx,qy,qz,qw,message
```

## Hazard-Aware Guidance API

`astro_navigation/navigation/hazard_guidance.hpp` exposes a planner-facing interface that accepts a
2D cost grid and returns a waypoint route. It is deliberately independent from OpenCV and terrain-image
loading; callers can build the cost grid from shadows, slopes, crater masks, operator annotations, or a
mission map.

Core types:

- `HazardCostMap`: row-major grid with `width`, `height`, `resolution_m`, `origin_xy_m`, and per-cell costs
- `HazardPlannerOptions`: blocked-cost threshold, heuristic weight, start/goal snap radius, and diagonal moves
- `HazardRoute`: route cells, metric waypoint centers, total traversal cost, route quality metrics, and message
- `HazardRouteMetrics`: route length, straight-line length, detour ratio, mean/max route cost, and minimum
  clearance from blocked cells

Example:

```cpp
#include "astro_navigation/navigation/hazard_guidance.hpp"

astro::navigation::HazardCostMap map;
map.width = 100;
map.height = 100;
map.resolution_m = 2.0;
map.costs.assign(100U * 100U, 1.0);

astro::navigation::HazardPlannerOptions options;
options.blocked_cost = 1000.0;

const auto route = astro::navigation::planHazardAwareRoute(map, {4, 12}, {80, 60}, options);
const double detour = route.metrics.detour_ratio;
```

The current implementation uses grid A* with optional diagonal moves. Start and goal cells are snapped
to nearby traversable cells so a rover can recover when a TRN estimate lands just inside a masked hazard
cell.

`hazard_route_demo` exposes the same planner through a small CLI. The input cost map is a numeric CSV
matrix, one row per grid row. High or infinite values can be blocked with `--blocked-cost`.

```bash
build/apps/hazard_route_demo \
  --cost-map hazard_cost.csv \
  --start-cell-x 4 --start-cell-y 12 \
  --goal-cell-x 80 --goal-cell-y 60 \
  --resolution-m 2.0 \
  --blocked-cost 1000 \
  --output-csv hazard_route.csv \
  --output-json hazard_route.json
```

The route CSV schema is:

```text
index,cell_x,cell_y,x_m,y_m,cost,cumulative_distance_m,clearance_cells,clearance_m
```

The stdout summary and JSON output include:

```text
route_length_m,straight_line_length_m,detour_ratio,mean_cost,max_cost,min_clearance_cells,min_clearance_m
```

The hazard-aware GIF renderer can use the CLI directly:

```bash
python3 scripts/render_hazard_aware_navigation_demo.py \
  --planner-app build/apps/hazard_route_demo \
  --output docs/figures/hazard_aware_navigation_demo.gif
```

## Trajectory Evaluation

`scripts/evaluate_trajectory.py` compares TUM/CSV VO output against a POLAR pose subset TSV. Use
`--alignment sim3` for monocular trajectories and `--alignment se3` for metric stereo/depth trajectories
before reporting ATE and relative translation metrics.

## Module Boundaries

- `core`: shared types and image sequence loading
- `localization`: visual odometry and future pose estimation
- `navigation`: mission-facing navigation state, quality, status, pipeline glue, and hazard-aware guidance
- `terrain`: terrain landmarks and descriptors
- `crater`: crater candidate detection/matching
- `visualization`: trajectory and future map visualization
- `datasets`: downloaded public data and dataset-specific manifests
- `benchmarks`: reproducible experiment runners
