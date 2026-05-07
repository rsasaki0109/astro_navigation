# astro_localization Handoff Plan

Last updated: 2026-05-08 (realism flags added — found ~17% catastrophic-failure rate at 37.5% false detection under realistic camera effects; algorithmic robustness work is now the next priority)

This file is a handoff note for the next coding agent. The project direction has shifted from generic
Earth-style rover visual odometry toward space-native localization, especially star tracker / lost-in-space
identification, with lunar terrain-relative navigation kept as a later parallel track.

## Operating Notes

- Workspace root used in this session:
  `astro_loc_ws/astro_localization`
- Shell commands in this environment should be prefixed with `rtk`.
- The repository currently appears as untracked files in `git status --short`; do not assume a clean git
  baseline.
- Avoid destructive git commands.
- Network is available, but public dataset/catalog artifacts already exist locally for the current star
  tracker work.
- Primary current benchmark catalog:
  `datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv`
- Raw HYG catalog:
  `datasets/star_catalogs/hyg-v42/raw/hyg_v42.csv.gz`

## Project Goal

Build `astro_localization`, a practical OSS localization/navigation stack for GNSS-denied space robotics:

- lunar rovers
- Mars rovers
- orbital robots
- planetary exploration robots
- Terrain Relative Navigation
- star tracker and celestial attitude estimation
- future Visual/LiDAR/Inertial fusion

The current strongest thread is:

1. public star catalog adapter
2. identified-star attitude estimation
3. lost-in-space star identification
4. scalable catalog indexing
5. robustness under missing and false detections

The lunar visual odometry / POLAR work exists and builds, but the user explicitly wants space-specific
localization such as star trackers rather than generic Earth rover VO as the main direction.

## Current Build Status

The following passed after the latest changes:

```bash
rtk python3 -m py_compile \
  scripts/download_dataset.py \
  scripts/download_star_catalog.py \
  scripts/convert_star_catalog.py \
  scripts/filter_star_catalog.py \
  scripts/generate_star_tracker_case.py \
  scripts/generate_star_tracker_observations_from_catalog.py \
  scripts/drop_star_ids.py \
  scripts/identify_stars_by_triangles.py \
  scripts/build_star_triangle_index.py \
  scripts/build_star_pair_index.py \
  scripts/identify_stars_with_index.py \
  scripts/identify_stars_with_pair_index.py \
  scripts/apply_star_identifications.py \
  scripts/prepare_polar_traverse.py \
  scripts/evaluate_trajectory.py \
  scripts/plot_trajectory.py \
  benchmarks/run_hyg_ambiguity_benchmark.py \
  benchmarks/run_hyg_pair_index_benchmark.py \
  benchmarks/run_hyg_pair_index_robustness.py \
  benchmarks/run_hyg_pair_index_observation_scaling.py \
  benchmarks/run_hyg_pair_index_false_scaling.py \
  benchmarks/run_lost_in_space_benchmark.py \
  benchmarks/run_lost_in_space_robustness.py \
  benchmarks/run_star_tracker_benchmark.py \
  benchmarks/run_visual_odometry_benchmark.py \
  benchmarks/run_polar_traverse_suite.py \
  benchmarks/summarize_exposure_sweep.py
```

```bash
rtk cmake --build build --parallel
```

The CI workflow `.github/workflows/ci.yml` includes these Python scripts in its compile smoke test.

## Implemented Star Tracker Components

### C++ App

- `apps/star_tracker_attitude`
- Estimates camera-inertial attitude from identified star observations.
- Uses catalog unit vectors and image observations.
- Solves Wahba/Kabsch style rotation.
- Outputs attitude quaternion and residuals.

### Catalog Tooling

- `scripts/download_star_catalog.py`
  - Downloads HYG v4.2 from the public Astronexus Codeberg source.
- `scripts/convert_star_catalog.py`
  - Converts RA/Dec/magnitude rows into `id,x,y,z,mag,ra_deg,dec_deg`.
- `scripts/filter_star_catalog.py`
  - Sorts by magnitude and removes close unresolved stars below `--min-separation-arcsec`.
  - This is important for benchmarks because the synthetic camera model cannot resolve close visual
    binaries or near-neighbor stars.

### Synthetic Observation Generation

- `scripts/generate_star_tracker_case.py`
  - Fully synthetic known-ID star tracker case.
- `scripts/generate_star_tracker_observations_from_catalog.py`
  - Projects stars from a converted public catalog into the camera.
  - Supports yaw/pitch/roll, noise, intrinsics.
- `scripts/drop_star_ids.py`
  - Removes IDs from observation CSVs.
  - Can inject missing detections with `--drop-count`.
  - Can inject false detections with `--false-count`.
  - Can write shuffled truth map with `--truth-output`.

## Lost-In-Space Identification Implementations

### Brute Force Triangle Prototype

- `scripts/identify_stars_by_triangles.py`
- Good for local synthetic validation.
- Not suitable for public full-catalog scaling.

### Triangle Index Prototype

- `scripts/build_star_triangle_index.py`
- `scripts/identify_stars_with_index.py`
- Precomputes sorted angular triangle side bins.
- Accurate through small/medium HYG subsets, but build explodes:
  - 500 stars: 2,524,698 triangles
  - 55 MB pickle
  - about 373 seconds end-to-end in Python
- Decision: do not scale full triangle enumeration further.

### Pair-Angle Index Prototype

- `scripts/build_star_pair_index.py`
- `scripts/identify_stars_with_pair_index.py`
- Stores star pairs by angular separation instead of precomputing all triangles.
- Query reconstructs candidate triangles from three observed pair angles.
- Reuses Wahba/Kabsch verification.
- Candidate pruning:
  - score by edge residual + weak magnitude prior
  - `--max-candidates-per-observation-triangle`
  - `--max-verified-hypotheses`
- Verification is vectorized in `identify_stars_with_index.py::verify_rotation`, and pair-index imports it.
- Observation triangle selection now uniformly samples across all combinations instead of taking the first
  N combinations. This fixed false-star cases where shuffled observation order made prefix selection biased.

## Key Benchmark Results

### Star Tracker Attitude

Synthetic identified-star attitude benchmark, 30 identified stars:

| Pixel noise | Mean attitude error |
| ---: | ---: |
| 0 px | 0.000015 deg |
| 0.1 px | 0.00459 deg |
| 0.5 px | 0.0229 deg |
| 1.0 px | 0.0458 deg |

### Pair Index Scaling

Current preferred scalable prototype: pair-angle index with resolved HYG catalog.

Command pattern:

```bash
rtk python3 benchmarks/run_hyg_pair_index_benchmark.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_benchmark_pruned \
  --index-sizes 500 1000 2000 \
  --trials 3 \
  --stars 8 \
  --noise-px 0.1 \
  --max-edge-deg 80 \
  --min-star-separation-arcsec 120
```

Results:

| Indexed stars | Indexed pairs | Index size | Query avg | Correct IDs |
| ---: | ---: | ---: | ---: | ---: |
| 500 | 52,553 | 0.4 MB | 0.298 s | 24/24 |
| 1000 | 208,563 | 1.6 MB | 0.392 s | 24/24 |
| 2000 | 832,038 | 6.3 MB | 0.727 s | 24/24 |

### Pair Index Robustness

Command pattern:

```bash
rtk python3 benchmarks/run_hyg_pair_index_robustness.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_robustness \
  --index-size 2000 \
  --stars 12 \
  --trials 2 \
  --drop-counts 0 2 4 \
  --false-counts 0 2 4 \
  --noise-px 0.1 \
  --min-star-separation-arcsec 120
```

Results:

| Dropped | False | Correct true IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | 24/24 | 0 | 1.281 s |
| 0 | 4 | 24/24 | 0 | 2.393 s |
| 2 | 4 | 20/20 | 0 | 2.299 s |
| 4 | 4 | 16/16 | 0 | 1.560 s |

Full table is in:

- `outputs/hyg_pair_index_robustness/summary.md`

### Observed Star Count Scaling

Command pattern:

```bash
rtk python3 benchmarks/run_hyg_pair_index_observation_scaling.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_observation_scaling \
  --index-size 2000 \
  --star-counts 16 24 32 \
  --trials 2 \
  --noise-px 0.1 \
  --min-star-separation-arcsec 120
```

Results:

| Observed stars | Correct IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: |
| 16 | 32/32 | 0 | 2.873 s |
| 24 | 48/48 | 0 | 2.381 s |
| 32 | 64/64 | 0 | 2.454 s |

Full table is in:

- `outputs/hyg_pair_index_observation_scaling/summary.md`

### False Detection Scaling With 32 True Stars

This is the latest major result before this handoff.

Command:

```bash
rtk python3 benchmarks/run_hyg_pair_index_false_scaling.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_false_scaling_uniform \
  --index-size 2000 \
  --stars 32 \
  --false-counts 0 4 8 12 \
  --trials 2 \
  --noise-px 0.1 \
  --min-star-separation-arcsec 120
```

Results after uniform observation-triangle sampling:

| False detections | Correct true IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 2.426 s |
| 4 | 64/64 | 0 | 2.686 s |
| 8 | 64/64 | 0 | 2.664 s |
| 12 | 64/64 | 0 | 2.694 s |

Full table is in:

- `outputs/hyg_pair_index_false_scaling_uniform/summary.md`

### 4000-Star Density Smoke

The user asked for the next step, and we scaled HYG pair-index density from 2000 to 4000.

Initial smoke (false 0 and 12 only):

```bash
rtk python3 benchmarks/run_hyg_pair_index_false_scaling.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_false_scaling_4000 \
  --index-size 4000 \
  --stars 32 \
  --false-counts 0 12 \
  --trials 2 \
  --noise-px 0.1 \
  --min-star-separation-arcsec 120
```

Index details:

- indexed stars: 4000
- indexed pairs: 3,329,812
- index size: 25.2 MB
- index build: 51-65 s (between runs)
- minimum star separation: 120 arcsec

Full false sweep (latest, post-profiling):

```bash
rtk python3 benchmarks/run_hyg_pair_index_false_scaling.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_false_scaling_4000_full \
  --index-size 4000 \
  --stars 32 \
  --false-counts 0 4 8 12 \
  --trials 2 \
  --noise-px 0.1 \
  --min-star-separation-arcsec 120
```

Result with per-stage timing:

| False | Correct | Wrong | Query avg | Cand gen avg | Verify avg |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 8.796 s | 7.413 s | 0.425 s |
| 4 | 64/64 | 0 | 8.689 s | 7.346 s | 0.382 s |
| 8 | 64/64 | 0 | 8.889 s | 7.419 s | 0.472 s |
| 12 | 64/64 | 0 | 9.311 s | 7.932 s | 0.410 s |

Full table: `outputs/hyg_pair_index_false_scaling_4000_full/summary.md`

Already documented in README, docs/experiments.md, docs/space_localization.md, docs/decisions.md.

### Candidate-Generation Profiling (added 2026-05-07)

`scripts/identify_stars_with_pair_index.py` now records per-stage timings in the assignment JSON:

- `candidate_generation_seconds`: time spent inside the per-observation-triangle
  `candidate_mappings()` call (pair-list lookup, adjacency build, intersection, edge filter, scoring,
  per-triangle pruning).
- `pruning_seconds`: time spent sorting and slicing the global hypothesis list.
- `verification_seconds`: time spent in vectorized Wahba/Kabsch verification.

`benchmarks/run_hyg_pair_index_false_scaling.py` aggregates these into `summary.csv` and `summary.md`.

Confirmed bottleneck at 4000 indexed stars: candidate generation is ~85% of query time
(7.3-7.9 s per query) while verification is only ~0.4 s and pruning is ~1 ms.

## Important Design Decisions

### Keep Star Tracker Work Python Until Data Structure Stabilizes

The current star identification logic is still in Python because algorithm ambiguity, catalog indexing,
and benchmark design are the main risks. Do not port to C++ until:

- pair-index data structure is stable
- false/missing detections are well characterized
- larger HYG density results are understood
- index file format decision is made

### Pair Index Is Preferred Over Full Triangle Index

Full triangle indexing is accurate but does not scale. Pair indexing reaches 2000+ stars with small
indices and reasonable query time.

### Filter Unresolved Close Stars

`scripts/filter_star_catalog.py` is part of the benchmark pipeline. It is not merely cosmetic.

Reason:

- HYG contains close visual binaries / near-neighbor stars.
- The current synthetic camera has no PSF, centroid blending, intensity model, or resolving power model.
- Treating unresolved close stars as distinct truth targets causes artificial failures.

Default benchmark threshold:

- `--min-star-separation-arcsec 120`

### Uniform Observation-Triangle Sampling Is Required

Taking the first N observation triangles failed in false-star cases because observation rows are shuffled.
This biased candidate generation based on arbitrary row order.

Current behavior:

- `select_observation_triangles()` samples uniformly over all combinations.
- This fixed the `32 true + false 8/12` failures at `max_observation_triangles=400`.

## Files Added Recently

Benchmarks:

- `benchmarks/run_hyg_pair_index_benchmark.py`
- `benchmarks/run_hyg_pair_index_robustness.py`
- `benchmarks/run_hyg_pair_index_observation_scaling.py`
- `benchmarks/run_hyg_pair_index_false_scaling.py`
- `benchmarks/run_hyg_ambiguity_benchmark.py`

Scripts:

- `scripts/build_star_pair_index.py`
- `scripts/identify_stars_with_pair_index.py`
- `scripts/filter_star_catalog.py`

Modified core scripts:

- `scripts/identify_stars_with_index.py`
  - vectorized `verify_rotation`
  - uniform observation-triangle sampling
- `scripts/identify_stars_with_pair_index.py`
  - candidate pruning
  - uniform observation-triangle sampling
- `scripts/build_star_triangle_index.py`
  - stores magnitudes
- `scripts/drop_star_ids.py`
  - can write shuffled truth mapping

## Recommended Next Tasks

### 1. Update README/docs With 4000-Star Result

DONE (2026-05-07). README.md, docs/experiments.md, docs/space_localization.md, docs/decisions.md
all now contain the 4000-star false sweep result with per-stage timings.

### 2. Run Full 4000 False Scaling

DONE (2026-05-07). Full `false_counts 0 4 8 12` sweep at 4000 indexed stars all returned 64/64
correct with 0 wrong assignments. Output at `outputs/hyg_pair_index_false_scaling_4000_full/`.

### 3. Optimize Candidate Generation For 4000+

History so far at 4000 indexed stars:

- Baseline (`outputs/hyg_pair_index_false_scaling_4000_full/`): cand_gen 7.35-7.93 s per query.
- `QueryCache` edge-bin adjacency cache (2026-05-07): no measurable improvement, reverted.
- Predicted-edge hoist (2026-05-07): regressed false=12 by ~4x, reverted.
- NumPy vectorization of inner predictions (2026-05-08): bit-exact, standalone improvement -2%
  to -15%, benchmark too noisy to confirm.
- Pandas 3-way merge phase-1 + NumPy phase-2 (2026-05-08,
  `outputs/hyg_pair_index_false_scaling_4000_pandas/`): bit-exact, sweep-avg cand_gen drops
  6.09-6.56 s (-10.7% to -19.6%) per false count.
- Pyramid mode `--pyramid-size 8` (2026-05-08,
  `outputs/hyg_pair_index_false_scaling_4000_pyramid8/`): 64/64 correct, 0 wrong; cand_gen drops
  6.09-6.56 s (pandas) → 0.90-1.08 s (-82% to -86%); query avg 2.48-2.83 s;
  candidate_hypotheses ~6.5-8.0k.
- `.npz` pair-index format with ndarray-backed buckets (2026-05-08,
  `outputs/hyg_pair_index_false_scaling_4000_pyramid8_npz/`): 64/64 correct, 0 wrong; cand_gen
  drops 0.90-1.08 s → 0.53-0.63 s (a further -40%); query avg 1.47-1.65 s; index size halves
  (26 MB → 13 MB).
- 8000 / 16000 sweeps on `.npz` (2026-05-08,
  `outputs/hyg_pair_index_false_scaling_{8000,16000}_pyramid_npz/`): 64/64 correct everywhere;
  8000 cand_gen 3.5-4.1 s / query 5.3-5.8 s (was 4.4-5.6 / 8.8-10.2); 16000 cand_gen 3.6-4.4 s /
  query 5.4-6.1 s (was 5.1-7.4 / 9.9-12.6). Both are catalog-saturated on mag&le;6.5 (see below).
- Catalog-saturation finding (2026-05-08): the converted HYG mag&le;6.5 file has 8920 stars, so
  any "16000-indexed-star" run on it is really an 8920-star run at the larger nominal slot.

Status at 4000/8000/16000 with pyramid + npz on mag&le;6.5: cand_gen 0.5-4.4 s, query 1.5-6.1 s.

Status at 8000 with pyramid + npz (`outputs/hyg_pair_index_false_scaling_8000_pyramid_npz/`):
64/64 correct, 0 wrong; cand_gen 3.5-4.1 s, query 5.3-5.8 s. Build 225 s, pickle 101 MB,
pairs 13.3M.

Status at 16000 with pyramid + npz (mag&le;6.5, catalog-saturated,
`outputs/hyg_pair_index_false_scaling_16000_pyramid_npz/`): 64/64 correct, 0 wrong; cand_gen
3.6-4.4 s, query 5.4-6.1 s. Build 240 s, pickle 124 MB, pairs 16.2M.

Status at 16000 with pyramid + npz on the deeper mag&le;8 catalog with the historical defaults
(`outputs/hyg_pair_index_false_scaling_16000_mag8_smoke/`, `--neighbor-bins 2 --tolerance-arcsec 300`):
32/32 correct, 0 wrong; cand_gen 47.1 s, query 51.5 s. Build 843 s, pickle 406 MB / npz 182 MB,
pairs 53.3M, candidate_hypotheses 451,789.

Status at 16000 with pyramid + npz on mag&le;8 with tight params
(`outputs/hyg_pair_index_false_scaling_16000_mag8_tight/`, `--neighbor-bins 1 --tolerance-arcsec 120`):
64/64 correct, 0 wrong across all four false counts. Cand_gen 9.97-11.96 s, query 13.99-16.58 s.
candidate_hypotheses ~33-41k (vs 451,789 with the historical defaults). Same index footprint.
Tightening the tolerance/neighbor-bin window closes ~75% of the cand_gen gap between catalog-
saturated mag&le;6.5 16000 and honest mag&le;8 16000.

Status at 24000 with pyramid + npz on mag&le;8 with tight params + ps=8
(`outputs/hyg_pair_index_false_scaling_24000_mag8_tight/`):
64/64 correct, 0 wrong across all four false counts. Build 226.8 s,
pickle 913.6 MB / npz 407 MB, pairs 119,879,891. Cand_gen 34.85-45.41 s, query 41.76-52.57 s.

Status at 24000 with pyramid + npz on mag&le;8 with tight params + ps=6
(`outputs/hyg_pair_index_false_scaling_24000_mag8_ps6/`):
64/64 correct, 0 wrong across all four false counts. Cand_gen 13.84-17.95 s,
query 20.66-25.19 s. About -60% cand_gen vs ps=8 at the same density.

Status at 32000 with pyramid + npz on mag&le;8 with tight params + ps=8 (smoke,
`outputs/hyg_pair_index_false_scaling_32000_mag8_tight_smoke/`):
32/32 correct, 0 wrong (single trial). Build 430.0 s, pickle 1624 MB / npz 667 MB,
pairs 213,021,414. Cand_gen 155.69 s, query 167.64 s. Single-pass ps=8 is past the practical
envelope.

Status at 32000 with pyramid + npz on mag&le;8 with tight params + ps=6 (FULL SWEEP,
`outputs/hyg_pair_index_false_scaling_32000_mag8_ps6/`):
64/64 correct, 0 wrong across all four false counts. Build 472.8 s, pickle 1624 MB / npz 667 MB,
pairs 213,021,414. Cand_gen 35.92-43.65 s, query 49.16-55.19 s. **New operational ceiling.**

Pyramid-size tuning at 16000 mag&le;8 (false=12 trial=1):

- size 4 fails (only 6/32 stars assigned, RMS 333 arcsec) at 27% false rate.
- size 6 succeeds at 33/32, RMS 90.47 arcsec, cand_gen 4.76-5.25 s.
- size 8 also succeeds at 33/32, RMS 90.47, cand_gen 12.08-13.43 s (~58% slower).
- size 10/12 same correctness, much slower.

Operational recommendation: `--pyramid-size 6` for honest-density work.

Pre-pyramid 8000 single-pass smoke (`outputs/hyg_pair_index_false_scaling_8000_smoke/`):
32/32 correct, 0 wrong, cand_gen 30.86 s. Pyramid mode reduces this 5-6x without losing
correctness.

Concrete next experiments, in order of expected payoff:

1. **40000-indexed-star sweep on mag&le;8 with ps=6**. The mag&le;8 catalog has 41,487 stars, so
   40000 is roughly the absolute density ceiling without re-conversion. Build ~12 min, query
   ~75-90 s extrapolated. Acceptance: 64/64 correct, 0 wrong, query under 2 minutes.
2. **`--pyramid-size 6` regression at higher false rates** (50%+ false detections). Confirm when
   size 6 starts failing and at what false-rate size 8 becomes necessary. Re-run mag&le;8 16000
   with `--false-counts 16 24 32` (= 33% / 43% / 50% false rate at 32 true stars).
3. **Catalog-magnitude sweep** at fixed 16000 indexed stars (mag&le;6.5, 7, 7.5, 8) to map
   pair-count vs query-latency growth.
4. **Sky-cell / bitset adjacency optimization** for 48000+ density. Deprioritized: the ps=6 +
   tight-params combination took the operational ceiling from 16000 to 32000 without any new
   algorithm. Algorithmic work re-enters only when query latency at 40000+ exceeds two minutes.
5. **C++ port of build_star_pair_index** if mag&le;9 catalog (~120k stars) is desired. Python
   build at 32000 stars already takes 7 minutes; 80000+ would push past 30 minutes even
   vectorized. The `.npz` format is the right serialization for a C++ reader.

Acceptance signal: `candidate_generation_seconds` in the assignment JSON and the `Cand gen avg`
column in the false-scaling summary should drop while `correct == total` and `wrong == 0`. The
current best baselines to compare against are
`outputs/hyg_pair_index_false_scaling_4000_pyramid8/summary.csv`,
`outputs/hyg_pair_index_false_scaling_8000_pyramid/summary.csv`, and
`outputs/hyg_pair_index_false_scaling_16000_pyramid/summary.csv`.

### 4. 8000 Stars: Full Sweep With Pyramid

Pyramid mode flipped 8000 from "barely smokeable" to "comfortably benchmarkable":

```bash
rtk python3 benchmarks/run_hyg_pair_index_false_scaling.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/hyg_pair_index_false_scaling_8000_pyramid \
  --index-size 8000 \
  --stars 32 \
  --false-counts 0 4 8 12 \
  --trials 2 \
  --noise-px 0.1 \
  --min-star-separation-arcsec 120 \
  --pyramid-size 8
```

Result: 64/64 correct, 0 wrong at every false count. Cand_gen 4.4-5.6 s, query 8.8-10.2 s, build
198 s, index 101 MB, 13.3M pairs. Output at `outputs/hyg_pair_index_false_scaling_8000_pyramid/`.

The single-pass smoke from earlier in 2026-05-08
(`outputs/hyg_pair_index_false_scaling_8000_smoke/`) is preserved as a regression baseline:
32/32 correct, cand_gen 30.86 s, query 34.9 s. Pyramid mode is the operational configuration; the
single-pass path is still in the code as the default for backwards compatibility.

### 5. Add A Stable Index Format

Current index format is Python pickle. It is okay for prototype benchmarks, not for OSS/public API.

Options:

- `.npz` arrays:
  - star vectors
  - magnitudes
  - pair bin offsets
  - pair endpoint arrays
- flat binary format
- C++ readable binary once C++ port begins

Do this before C++ port.

### 6. Add Realistic Star Camera Effects

Current synthetic observations are clean geometric projections plus Gaussian pixel noise.

Missing realism:

- limiting magnitude / exposure model
- measured spot intensity
- PSF / centroid noise
- unresolved binary blending
- optical distortion
- catalog proper motion
- aberration/time effects
- sensor false positives not uniformly random

Near-term useful additions:

- generate false stars biased near real stars
- generate false stars with image-edge or hot-pixel patterns
- use catalog magnitude to choose observed stars probabilistically, not just brightest visible subset

## Caution Points

### Do Not Overclaim Full Flight-Grade Star Tracker

This is still a prototype. It is practical and benchmarkable, but not flight software.

Known gaps:

- no PSF/intensity model
- no dynamic exposure
- no full camera calibration/distortion model
- no proper motion/time handling
- no stable index format
- Python implementation only

### Do Not Delete Triangle Index

Triangle index is still useful as a baseline and regression check, even if pair-index is preferred.

### Watch Query Metrics

For every scaling change, record:

- indexed stars
- indexed pairs or triangles
- index MB
- build seconds
- candidate hypotheses
- pruned hypotheses / triangle matches
- verified hypotheses
- query seconds
- correct / wrong / assigned IDs

## Quick Commands For Claude

### Compile Smoke

```bash
rtk python3 -m py_compile \
  scripts/download_dataset.py \
  scripts/download_star_catalog.py \
  scripts/convert_star_catalog.py \
  scripts/filter_star_catalog.py \
  scripts/generate_star_tracker_case.py \
  scripts/generate_star_tracker_observations_from_catalog.py \
  scripts/drop_star_ids.py \
  scripts/identify_stars_by_triangles.py \
  scripts/build_star_triangle_index.py \
  scripts/build_star_pair_index.py \
  scripts/identify_stars_with_index.py \
  scripts/identify_stars_with_pair_index.py \
  scripts/apply_star_identifications.py \
  scripts/prepare_polar_traverse.py \
  scripts/evaluate_trajectory.py \
  scripts/plot_trajectory.py \
  benchmarks/run_hyg_ambiguity_benchmark.py \
  benchmarks/run_hyg_pair_index_benchmark.py \
  benchmarks/run_hyg_pair_index_robustness.py \
  benchmarks/run_hyg_pair_index_observation_scaling.py \
  benchmarks/run_hyg_pair_index_false_scaling.py \
  benchmarks/run_lost_in_space_benchmark.py \
  benchmarks/run_lost_in_space_robustness.py \
  benchmarks/run_star_tracker_benchmark.py \
  benchmarks/run_visual_odometry_benchmark.py \
  benchmarks/run_polar_traverse_suite.py \
  benchmarks/summarize_exposure_sweep.py
```

### C++ Build

```bash
rtk cmake --build build --parallel
```

### Latest High-Value Benchmark

```bash
rtk cat outputs/hyg_pair_index_false_scaling_4000/summary.md
```

### Pair Index 2000 False Scaling

```bash
rtk cat outputs/hyg_pair_index_false_scaling_uniform/summary.md
```

### Pair Index 2000 Observation Scaling

```bash
rtk cat outputs/hyg_pair_index_observation_scaling/summary.md
```

## Suggested Immediate Claude Plan

Both PLAN steps from the prior handoff (40000 full sweep + 16000 high-false-rate stress) are now
done — see results in `docs/experiments.md` (entries dated 2026-05-08) and the consequence
discussion in `docs/decisions.md`. Operational ceiling is now 40000 indexed stars on HYG mag&le;8,
which is essentially the absolute density limit (mag&le;8 catalog caps at 41487 stars).

Realism flags landed (`--limiting-magnitude`, `--mag-softness` on the catalog observation
generator; `--false-near-fraction`, `--false-near-sigma-px` on `drop_star_ids.py`). A 16000
mag&le;8 ps=6 trials=6 sweep with realism enabled (`limiting=7.0 softness=0.5 near-frac=0.5
near-sigma=20`) found a ~17% per-trial catastrophic-failure rate at false=12 (37.5% false rate);
ps=8 mitigation does not help. See `docs/experiments.md` and `docs/decisions.md` for the data and
the failure analysis.

The next handoff should pick from the following, in roughly decreasing value:

1. **Algorithmic robustness against confusion-attitude lock-in (highest priority — this is now the
   main correctness bottleneck under realism).** The pyramid identifier accepts the first
   verified hypothesis without checking whether the predicted vs observed star count is
   consistent. Concrete options:
   - Keep top-K candidate attitudes by verified-match-count, run full verification on each, pick
     the highest-match attitude (reject low-confidence ties).
   - Track expected vs predicted observation counts under the recovered attitude and a
     limiting-magnitude prior; reject attitudes that predict far more or far fewer matches than
     observed.
   - Re-pyramid with a different observation subset when verified-count falls below a threshold.

2. **Push past the catalog density ceiling.** Convert HYG mag&le;9 (~120k stars), filter to a
   resolved subset, and benchmark. Pair-count growth from 332M (40k) to ~3G (80k) plus
   `np.argsort` cost makes peak memory the next risk even with `--skip-pkl`. Worth doing only if
   real-camera mag-limit testing shows mag&le;8 is too restrictive (and the realism work above
   suggests the bottleneck is currently algorithmic, not catalog depth).

3. **Sky-cell / HEALPix partitioning of the pair index.** Currently deferred — at 40000 mag&le;8
   we are still at ~70 s worst-case query, comfortable for cold-start LIS. Becomes important if
   step 2 happens or if a sub-10 s online identifier is needed.

4. **Begin the C++ port of the pair-index identifier.** The Python prototype is fixed enough to
   port, but step 1 should land first because the C++ port should not freeze a known-broken
   matcher.

5. **More realism axes.** Centroid noise scaled by intensity, optical distortion, catalog proper
   motion, image-edge / hot-pixel false-positive distributions. Lower priority than step 1 since
   the matching algorithm itself is the current weak link.

6. **POLAR multi-traverse robustness.** The Traverse4-6 baseline is still 11/33 to 15/33 frames
   OK even with CLAHE. Exposure sweeps and SIFT stereo PnP haven't been tried as a unified suite.

For the immediate `tugi` cycle, step 1 is the right continuation: it's the only path to making
the realism numbers match the idealized numbers.

## Current Best Technical Summary

The star tracker path has moved from a simple identified-star attitude estimator to a scalable lost-in-space
prototype backed by the public HYG catalog. Full triangle indexing hit a clear scaling wall, so pair-angle
indexing is now the preferred prototype. With resolved HYG catalog filtering, uniform observation-triangle
sampling, candidate pruning, and vectorized attitude verification, the system currently handles:

- 2000 indexed stars
- 32 true observed stars
- 12 false detections
- 0 wrong assignments
- around 2.7 s query time

It also completed a 4000-star smoke:

- 4000 indexed stars
- 32 true observed stars
- 12 false detections
- 0 wrong assignments
- around 9.8 s query time

It now operates at honest mag&le;8 density up to the catalog ceiling:

- 40000 indexed stars (mag&le;8 catalog has 41487 effective stars)
- 332 M indexed pairs, 1016 MB `.npz`, 277 s build (vectorized)
- 32 true observed stars + up to 12 false detections
- 64/64 correct, 0 wrong across the full sweep
- 61-94 s query (`--pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120 --skip-pkl`)
- separately confirmed at 16000 mag&le;8 to be robust through 50% false rate at ~6 s query

The next engineering problem is no longer scaling within the mag&le;8 catalog. It is either
reaching past the catalog density (mag&le;9 conversion) or porting the prototype to C++ for
real-camera deployment.
