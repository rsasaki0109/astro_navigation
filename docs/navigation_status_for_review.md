# astro_navigation Development Status

This is a handoff summary for external design review. The repository was originally named
`astro_localization`, but the project direction has been broadened toward space navigation.

## Current Goal

Build `astro_navigation`, a C++20 + Python toolkit for GNSS-denied space robotics:

- star tracker attitude estimation
- lost-in-space star identification
- lunar visual odometry
- terrain-relative navigation using real LRO/LOLA fixtures
- navigation state / health that combines attitude and position locks

The intended distinction is:

- `localization`: estimators that produce attitude, pose, or position observations
- `navigation`: mission-facing state, quality, covariance, status, and future fusion / guidance

## Rename Status

The external project name and include path have been changed from `astro_localization` to
`astro_navigation`.

Examples:

- CMake project: `astro_navigation`
- CMake target namespace: `astro_navigation::...`
- include path: `#include "astro_navigation/..."`
- README/docs/GitHub About text updated to navigation-facing wording

The existing `localization/` module remains as a functional submodule because it still contains
star tracker, VO, stereo VO, and lost-in-space estimators.

The GitHub repository has been renamed to `astro_navigation`. The local checkout directory may still
be named `astro_localization` until it is recloned or moved locally.

## New Navigation Module

Added `navigation/`:

- `navigation/include/astro_navigation/navigation/state.hpp`
- `navigation/src/state.cpp`
- `navigation/include/astro_navigation/navigation/trn_summary.hpp`
- `navigation/src/trn_summary.cpp`
- `navigation/include/astro_navigation/navigation/state_io.hpp`
- `navigation/src/state_io.cpp`
- `navigation/include/astro_navigation/navigation/pipeline.hpp`
- `navigation/src/pipeline.cpp`
- `navigation/include/astro_navigation/navigation/hazard_guidance.hpp`
- `navigation/src/hazard_guidance.cpp`

Main types:

- `NavStatus`: `UNKNOWN`, `OK`, `DEGRADED`, `LOST`, `RELOCALIZING`
- `NavStatusReason`: `NONE`, `NO_LOCKS`, `ATTITUDE_ONLY`, `POSITION_ONLY`, `VELOCITY_MISSING`,
  `HIGH_ATTITUDE_UNCERTAINTY`, `HIGH_POSITION_UNCERTAINTY`, `ROUTE_RISK_HIGH`
- `NavQuality`: lock flags, sigma values, route confidence, risk score, correspondence counts
- `NavState`: timestamp, frame IDs, position, velocity, quaternion, 6x6 covariance, quality, status,
  status reason, message
- `PositionLockMeasurement`: TRN position, estimated sigma, evaluation error, match count, inlier
  count, source
- `MissionNavigationInput` / `MissionNavigationResult`: reusable pipeline boundary for mission demos
- `HazardCostMap` / `HazardPlannerOptions` / `HazardRoute`: reusable waypoint-guidance boundary for
  2D hazard-cost grids

Current status logic:

- no attitude or position lock -> `LOST`, reason `NO_LOCKS`
- attitude only -> `DEGRADED`, reason `ATTITUDE_ONLY`
- position only -> `DEGRADED`, reason `POSITION_ONLY`
- attitude + position -> `OK`, reason `NONE`
- attitude + position + `navigation_risk_score >= 0.60` -> `DEGRADED`, reason `ROUTE_RISK_HIGH`

## Hazard-Aware Guidance

The previous hazard-aware lunar navigation GIF now has a reusable C++ planner behind the mission idea.

Added:

- `planHazardAwareRoute()`: grid-cell A* over a row-major cost map
- `planHazardAwareRouteMeters()`: metric start/goal adapter using map origin and resolution
- `resampleRoute()`: route downsampling for visualization and control-loop handoff
- `HazardRouteMetrics`: route length, straight-line length, detour ratio, mean/max cost, and minimum
  clearance from blocked hazard cells
- blocked-cell filtering with a configurable cost threshold
- start/goal snapping to nearby traversable cells for noisy TRN position locks

The Python renderer still owns image-derived cost-map creation from the Tycho TRN ortho fixture. The C++
navigation library now owns the reusable planning contract, and `hazard_route_demo` exposes it as a
CSV/JSON CLI.

Covariance is currently simple diagonal covariance:

- position variance = `position_sigma_m^2`
- attitude variance = `attitude_sigma_rad^2`
- fallback large variances for missing/invalid sigma

## Navigation Pipeline And CLI

Added:

- `runMissionNavigation()` in `navigation/pipeline.hpp`

It is the reusable library API around the mission navigation flow:

1. load star observations and catalog
2. estimate star tracker attitude
3. optionally load TRN summary position or manual position
4. emit `MissionNavigationResult` with `NavState`, TRN matches, and TRN inliers

The CLI now calls this API instead of owning the navigation logic.

## New CLI

Added:

- `apps/mission_navigation_demo.cpp`

It is now a thin command-line wrapper:

1. reads identified star observations and catalog
2. runs existing C++ star tracker attitude estimator
3. optionally reads TRN summary JSON from `scripts/lro_trn_demo.py`
4. prints one CSV row to stdout
5. optionally writes JSON/CSV files

Example:

```bash
build/apps/mission_navigation_demo \
  --catalog outputs/quick_star_case/catalog.csv \
  --observations outputs/quick_star_case/observations.csv \
  --fx 1000 --fy 1000 --cx 512 --cy 512 \
  --trn-summary docs/figures/trn_lro_tycho_terminal/summary.json \
  --output-json outputs/quick_star_case/nav_state.json \
  --output-csv outputs/quick_star_case/nav_state.csv
```

Expected status:

```text
OK
```

## TRN Summary Loader

`loadTrnSummaryPositionLock()` reads existing TRN summary JSON files, currently using the fields:

- `rover_estimated_xyz_m`
- `position_error_m` as `evaluation_error_m`, not as the navigation sigma
- `wac.px_to_m`
- `pnp.inlier_median_reproj_px`
- `match_count`
- `pnp.pnp_inliers`

The estimated navigation sigma is now truth-free:

```text
map_term = 2.0 * px_to_m
reproj_term = inlier_median_reproj_px * px_to_m
inlier_term = px_to_m * sqrt(12 / max(pnp_inliers, 1))
position_sigma_m = max(map_term, reproj_term, inlier_term)
```

`PositionLockMeasurement::quality_terms` preserves each component:

- `map_resolution_sigma_m`
- `reprojection_sigma_m`
- `inlier_geometry_sigma_m`

`MissionNavigationResult::position_lock` exposes the full `PositionLockMeasurement` for renderers,
benchmarks, or future debug output.

The first implementation uses a minimal regex-based extractor rather than a full JSON library,
consistent with some existing C++ code in the repo. This is probably acceptable short-term, but it is
worth reviewing whether a proper JSON dependency should be introduced.

## Navigation State Output

`writeNavStateJson()` emits:

- `timestamp`
- `status`
- `message`
- `position_frame_id`
- `attitude_reference_frame_id`
- `position_m`
- `velocity_mps`
- `q_body_reference_xyzw`
- `quality`
- `covariance_6x6`

The JSON contract is now documented in:

- `docs/schemas/nav_state.schema.json`
- `docs/interfaces.md`

`writeNavStateCsv()` emits one flat row for quick benchmark ingestion.

## Lost Robot Challenge Integration

Updated:

- `scripts/lost_robot_challenge.py`

Previously it called `star_tracker_attitude` directly and manually combined that result with the TRN
fixture summary.

Now it calls `mission_navigation_demo`, reads the generated `nav_state.json`, and uses that as the
mission-level source for:

- navigation status
- attitude quaternion
- attitude/position lock quality
- final summary JSON
- HUD text in the generated mission card

The resulting summary now includes a `navigation` section:

```json
{
  "navigation": {
    "status": "DEGRADED",
    "status_reason": "ROUTE_RISK_HIGH",
    "message": "route risk high",
    "position_m": [46069.113087615, 46097.730733616, 30000.596809296],
    "position_frame_id": "map",
    "quality": {
      "attitude_lock": true,
      "position_lock": true,
      "velocity_lock": false,
      "attitude_sigma_rad": 0.000141226,
      "position_sigma_m": 143.956140950,
      "localizability_score": 0.630000000,
      "route_trn_confidence": 0.380000000,
      "navigation_risk_score": 0.620000000,
      "attitude_correspondences": 30
    }
  }
}
```

## Tests

CTest is now enabled.

Added:

- `tests/navigation_state_smoke.cpp`
- `tests/hazard_guidance_smoke.cpp`
- `tests/mission_navigation_cli_smoke.cpp`
- `tests/hazard_route_cli_smoke.cpp`
- `tests/CMakeLists.txt`

The state/pipeline smoke test verifies:

- empty state -> `LOST`
- attitude-only state -> `DEGRADED`
- Tycho TRN summary fixture parses to expected position / sigma / match count
- attitude + TRN position -> `OK`
- JSON/CSV writers include expected fields
- `runMissionNavigation()` produces `OK` and preserves TRN quality counts from generated test input

The CLI smoke test verifies:

- `mission_navigation_demo` exits successfully with generated star CSV fixtures and the Tycho TRN summary
- stdout contains the expected `OK` navigation row
- `--output-json` and `--output-csv` files contain expected navigation fields
- `hazard_route_demo` plans from a CSV cost map and writes route CSV/JSON outputs with route-quality metrics

The hazard-guidance smoke test verifies:

- A* routes through the only traversable gap in a synthetic hazard wall
- blocked cells are excluded from the route
- route length, detour ratio, mean/max cost, and minimum clearance are reported
- metric start/goal planning matches grid-cell planning
- blocked starts snap to nearby traversable cells
- route resampling preserves endpoints

Verified:

```text
1/4 Test #1: navigation_state_smoke ........... Passed
2/4 Test #2: hazard_guidance_smoke ............ Passed
3/4 Test #3: mission_navigation_cli_smoke ..... Passed
4/4 Test #4: hazard_route_cli_smoke ........... Passed
100% tests passed, 0 tests failed out of 4
```

## Verified Commands

The latest working verification used an out-of-tree temporary build:

```bash
cmake -S . -B /tmp/astro_navigation-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/astro_navigation-build --parallel
ctest --test-dir /tmp/astro_navigation-build --output-on-failure
```

The Lost Robot Challenge path was also checked:

```bash
python3 scripts/lost_robot_challenge.py \
  --nav-app /tmp/astro_navigation-build/apps/mission_navigation_demo \
  --workdir /tmp/lost_robot_nav/work \
  --output /tmp/lost_robot_nav/lost_robot_challenge.png \
  --summary-output /tmp/lost_robot_nav/lost_robot_challenge.json
```

Result:

- PNG generated successfully, 1280 x 720
- navigation status `OK`
- navigation message `navigation lock`

The navigation visual demos were regenerated:

```bash
python3 scripts/render_navigation_replay_demo.py \
  --output docs/figures/navigation_replay_demo.gif
python3 scripts/render_hazard_aware_navigation_demo.py \
  --planner-app /tmp/astro_navigation-build/apps/hazard_route_demo \
  --output docs/figures/hazard_aware_navigation_demo.gif
```

The hazard-aware demo derives a cost map from the Tycho terminal TRN ortho image
using shadow/darkness and image-gradient cost, calls the C++ `hazard_route_demo`
planner, and animates a rover through `LOST`, `DEGRADED`, `OK`, relocalizing,
and arrived phases.

## Open Design Questions

1. Should `NavState` live under `astro::navigation`, while CMake uses `astro_navigation::navigation`, or should
   the C++ namespace also become `astro_navigation` for consistency?
2. Should uncertainty thresholds activate `HIGH_ATTITUDE_UNCERTAINTY` and `HIGH_POSITION_UNCERTAINTY`,
   or should those remain reserved until the covariance model is more defensible?
3. Should covariance be a fixed 6x6 pose covariance, or should navigation state reserve 9x9 / 15x15 for
   velocity, IMU bias, and future EKF/factor graph integration?
4. Should TRN summary parsing remain dependency-free regex parsing, or should the project add a JSON library?
5. Is the first truth-free TRN sigma model conservative enough, or should it include altitude,
   feature geometry, DEM resolution, and map-registration uncertainty?
6. Should `runMissionNavigation()` remain a functional API, or should it become a stateful `NavigationPipeline`
   class once temporal fusion, configuration, and health transitions are added?
7. Should the existing `docs/space_localization.md` be renamed to `docs/space_navigation.md`, or keep the filename
   to avoid churn?
8. Should the hazard-aware planner remain a grid-cost A* utility, or grow toward a guidance component with
   route-following state, receding-horizon replanning, and velocity constraints?

## Suggested Next Steps

Short-term:

1. Improve TRN quality modeling beyond the first px-to-m / reprojection / inlier-count heuristic.
2. Add uncertainty thresholds for the reserved high-uncertainty status reasons.
3. Improve CLI coverage for failure paths such as invalid intrinsics and missing TRN summary files.
4. Add route-cost and clearance overlays to the hazard-aware GIF using the new route metrics.
5. Decide whether to rename `docs/space_localization.md` to `docs/space_navigation.md`.

Medium-term:

1. Add EKF/factor graph interface for star tracker + TRN + VO.
2. Add `NavHealth` transitions: `OK`, `DEGRADED`, `LOST`, `RELOCALIZING`.
3. Add route-following state and receding-horizon replanning around `HazardRoute`.
4. Connect navigation output into mission demo renderers and benchmark scripts more broadly.
