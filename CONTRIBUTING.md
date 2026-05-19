# Contributing

`astro_localization` is early-stage research software. The best contributions are small,
reproducible improvements that keep the space-localization focus sharp.

## Good First Contributions

- Reproduce one README command on a fresh machine and report missing dependencies or unclear output.
- Add a small fixture or smoke test around an existing script.
- Improve a plot, GIF, or summary table without changing the underlying benchmark.
- Add documentation for one dataset adapter, including source URL, size, license, and expected cache path.
- Port a narrow Python helper into C++ when there is already a matching script and benchmark.

## Larger Project Areas

- Lost-in-space star identification: catalog partitioning, index-size reduction, hot-pixel and distortion
  robustness, and faster candidate generation.
- Terrain-relative navigation: ASIFT or affine-aware matching, render-time orthorectification, crater
  descriptors, and lower-altitude LRO/LOLA terminal-descent cases.
- Lunar visual odometry: metric stereo robustness, POLAR traversal coverage, illumination handling, and
  eventual visual-inertial fusion.
- Integration: ROS 2 nodes, CMake packaging, clean library APIs, and repeatable benchmark runners.

## Development Loop

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
python3 scripts/generate_star_tracker_case.py --output-dir outputs/star_tracker_smoke --noise-px 0.1 --stars 12
build/apps/star_tracker_attitude \
  --catalog outputs/star_tracker_smoke/catalog.csv \
  --observations outputs/star_tracker_smoke/observations.csv \
  --fx 1000 --fy 1000 --cx 512 --cy 512
```

Before opening a PR, run the narrow command that covers your change. For C++ edits, also run:

```bash
find . \( -name '*.cpp' -o -name '*.hpp' \) | xargs clang-format --dry-run --Werror
```

## Reporting Results

When reporting a benchmark or reproduction result, include:

- exact command line
- commit SHA
- dataset/catalog version
- machine summary if runtime or memory is relevant
- output table or artifact path

Negative results are useful when they define the operating envelope clearly. Keep the failure mode and
the next testable hypothesis explicit.
