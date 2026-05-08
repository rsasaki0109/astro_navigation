# astro_localization

`astro_localization` is an early-stage C++20 OSS for localization and navigation in GNSS-denied space
robotics: lunar/Mars rovers, orbital robots, planetary explorers, and terrain-relative navigation.

The focus is space-specific localization. First-class directions are **star tracker attitude**,
**lost-in-space star identification** against public catalogs, **lunar visual odometry**, and
**terrain-relative navigation** — not generic Earth robotics VO. The implementation is deliberately
small so that experiments converge quickly, and the Python prototypes live alongside the C++ apps.

## Headline Results

Numbers below are the current best on the corresponding benchmark. Full per-iteration history is in
[`docs/experiments.md`](docs/experiments.md).

| Module | Benchmark | Result |
| --- | --- | --- |
| Star tracker attitude | 30 stars synthetic, 0.1 px noise | mean attitude error **0.00459 deg** |
| Lost-in-space (HYG mag≤8, 40k indexed stars — catalog density ceiling) | 32 true + up to 12 false detections, 0.1 px noise, `--pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120 --skip-pkl` | **64/64 correct, 0 wrong**, query 61-94 s, build 277 s, .npz 1016 MB, 332 M pairs |
| Lost-in-space high-false-rate (HYG mag≤8, 16k indexed stars) | 32 true + 16/24/32 false detections (33-50% false rate), ps=6 | **64/64 correct, 0 wrong** at every level, query ~6 s |
| Lunar VO (POLAR Traverse1, L 50 ms, monocular SIFT) | 11 frames, Sim(3) alignment | ATE RMSE **0.0186 m**, 11/11 frames OK |
| Lunar VO (POLAR Traverse1, L 50 ms, rectified stereo + PnP) | 11 frames, SE(3) | ATE RMSE **0.0650 m**, path 10.18 m vs 9.98 m GT |

`--pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120` is the operational default for honest-density
HYG mag≤8 lost-in-space work.

## Build

Dependencies: CMake 3.20+, C++20 compiler, OpenCV 4 (`features2d`, `calib3d`, `imgcodecs`, `imgproc`),
Eigen3.

```bash
cd astro_localization
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

## Quick Start

Run lunar visual odometry on a POLAR Traverse left-camera 50 ms sequence:

```bash
python3 scripts/download_dataset.py --dataset polar-traverse-view1 --output datasets --confirm-large
python3 scripts/prepare_polar_traverse.py \
  --root datasets/polar-traverse-view1/extracted --camera L --exposure-ms 50 \
  --output outputs/polar_view1_left_50ms

build/apps/lunar_visual_odometry \
  --images outputs/polar_view1_left_50ms/images.txt \
  --fx 1452.71 --fy 1452.88 --cx 999.53 --cy 1035.4 \
  --feature sift --clahe \
  --trajectory outputs/trajectory_sift.tum
```

Run lost-in-space star identification against a public HYG catalog subset:

```bash
python3 scripts/download_star_catalog.py --catalog hyg-v42 --output datasets/star_catalogs
python3 scripts/convert_star_catalog.py \
  --input datasets/star_catalogs/hyg-v42/raw/hyg_v42.csv.gz \
  --output datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv \
  --format hyg --max-magnitude 8.0

python3 scripts/build_star_pair_index.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv \
  --output outputs/hyg_pair_index_40000.pkl --limit 40000 --skip-pkl

python3 scripts/identify_stars_with_pair_index.py \
  --index outputs/hyg_pair_index_40000.npz \
  --observations <observations_unlabeled.csv> \
  --output <assignments.csv> \
  --pyramid-size 6 --neighbor-bins 1 --tolerance-arcsec 120 \
  --fx 1000 --fy 1000 --cx 512 --cy 512
```

More detailed example commands (synthetic generators, ambiguity / robustness benchmarks, multi-traverse
suites) are in [`docs/space_localization.md`](docs/space_localization.md) and the benchmark scripts
under `benchmarks/`.

## Public Datasets

```bash
python3 scripts/download_dataset.py --list
```

- **NASA POLAR Traverse**: stereo traverses with poses and calibration —
  https://ti.arc.nasa.gov/dataset/PolarTrav/
- **NASA POLAR Stereo**: HDR stereo terrain with LiDAR ground truth —
  https://ti.arc.nasa.gov/dataset/IRG_PolarDB/
- **LunarLoc**: simulator traverses + `.lac` playback —
  https://github.com/mit-acl/lunarloc-data
- **Synthetic Lunar Terrain**: multimodal RGB/event/laser terrain —
  https://zenodo.org/records/13218780
- **Apollo Surface Panoramas** — https://catalog.data.gov/dataset/apollo-surface-panoramas
- **HYG Database v4.2** star catalog — https://codeberg.org/astronexus/hyg

Dataset licenses stay with the upstream providers; `manifest.json` records source URL, citation, and
checksum.

## Documentation

- [`docs/space_localization.md`](docs/space_localization.md) — primary modes, star tracker / TRN
  interfaces, near-term priorities.
- [`docs/experiments.md`](docs/experiments.md) — full experiment log: ORB vs SIFT, essential vs PnP,
  CLAHE, every HYG pair-index density iteration.
- [`docs/decisions.md`](docs/decisions.md) — design decisions and rationale.
- [`docs/interfaces.md`](docs/interfaces.md) — CSV/binary interface contracts.
- [`PLAN.md`](PLAN.md) — current and upcoming work.

## Roadmap

Star tracker catalog adapters; star tracker + visual TRN fusion; stereo VO with metric scale and
PnP; crater descriptor matching against orbital maps; visual-inertial fusion; LiDAR scan matching;
factor graph optimization (GTSAM/Ceres); orbital localization with star tracker fusion; ROS 2
integration; repeatable simulation benchmarks.

## References

- Hansen, M., Wong, U., and Fong, T. POLAR Traverse Dataset. NASA Ames Research Center, 2023.
- Wong, U., Nefian, A., Edwards, L., Buoyssounouse, X., Furlong, P. M., Deans, M., and Fong, T.
  POLAR Stereo Dataset. NASA Ames Research Center, 2017.
- LunarLoc: Segment-Based Global Localization on the Moon. https://arxiv.org/abs/2506.16940
- Synthetic Lunar Terrain: A Multimodal Open Dataset. https://arxiv.org/abs/2408.16971
