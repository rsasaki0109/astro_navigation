# Space Localization Focus

`astro_localization` should not become a generic Earth robotics VO package. The main direction is
space-specific localization and navigation where GNSS is unavailable and planetary/orbital sensors matter.

## Primary Modes

| Mode | State estimated | Main sensors | Why it is space-specific |
| --- | --- | --- | --- |
| Star tracker attitude | Inertial attitude | Star camera, catalog | Absolute attitude from celestial references |
| Terrain Relative Navigation | Position relative to orbital map | Camera/LiDAR + DEM/orthophoto | Uses crater/terrain landmarks instead of roads/buildings |
| Crater localization | Surface pose constraints | Lunar/Mars surface camera | Sparse landmarks tied to planetary maps |
| Visual-inertial rover localization | Local rover trajectory | Camera + IMU + wheel odometry | GNSS-denied surface motion under harsh illumination |
| Orbital optical navigation | Spacecraft relative/absolute pose | Camera, limb, stars, landmarks | Uses bodies, horizons, and stars instead of Earth GNSS |

## Initial Star Tracker Interface

`star_tracker_attitude` estimates camera attitude from identified star observations.

Observation CSV:

```text
id,u,v
star_a,512.0,512.0
```

Catalog CSV:

```text
id,x,y,z
star_a,0.0,0.0,1.0
```

The output quaternion `q_camera_inertial` rotates inertial catalog directions into the camera frame. This is
a minimal Wahba/Kabsch baseline. It assumes star IDs are already matched; lost-in-space identification is a
future module.

## Near-Term Priorities

1. Star tracker simulation and public star catalog adapter.
2. Crater/TRN map matching against public lunar imagery or DEM products.
3. Fuse star tracker attitude with visual terrain-relative position.
4. Keep rover VO as a supporting module, not the project identity.

## Synthetic Star Tracker Benchmark

`scripts/generate_star_tracker_case.py` creates deterministic identified-star observations from a known
camera attitude. `benchmarks/run_star_tracker_benchmark.py` runs the C++ estimator and compares the
estimated quaternion against the truth.

Current 30-star result:

| Pixel noise | Mean attitude error | Max attitude error |
| ---: | ---: | ---: |
| 0 px | 0.000015 deg | 0.000015 deg |
| 0.1 px | 0.00459 deg | 0.00728 deg |
| 0.5 px | 0.0229 deg | 0.0364 deg |
| 1.0 px | 0.0458 deg | 0.0729 deg |

This still assumes identified stars. The next star tracker step is a public catalog adapter and a simple
lost-in-space matcher.

## Public Star Catalog Adapter

`scripts/download_star_catalog.py` can download HYG v4.2 from the Astronexus Codeberg repository.
`scripts/convert_star_catalog.py` converts RA/Dec/magnitude rows to the `id,x,y,z` unit-vector format used
by `star_tracker_attitude`.

The converted `mag <= 6.5` HYG subset currently contains 8,920 stars. It is suitable for star tracker
simulation and catalog-backed identified-star tests. The adapter does not yet model proper motion,
aberration, time, optical distortion, or lost-in-space matching.

Catalog-backed synthetic observation generation:

```bash
python3 scripts/generate_star_tracker_observations_from_catalog.py \
  --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv \
  --output-dir outputs/star_tracker_hyg_case \
  --stars 20 \
  --noise-px 0.2
```

## Lost-In-Space Prototype

`scripts/identify_stars_by_triangles.py` is the first lost-in-space prototype. It accepts unlabeled pixel
observations, converts them to bearing vectors, compares angular triangle side lengths against a candidate
catalog, and votes for one-to-one star assignments.

Supporting scripts:

- `scripts/drop_star_ids.py`: remove IDs from synthetic observations and optionally write the shuffled
  `observation_index -> id` truth map used by benchmarks.
- `scripts/apply_star_identifications.py`: convert assignment output back to `id,u,v`.
- `benchmarks/run_lost_in_space_benchmark.py`: run synthetic ID recovery trials.

Current synthetic result, 12 stars and a candidate catalog containing those stars:

| Pixel noise | Correct IDs |
| ---: | ---: |
| 0 px | 36/36 |
| 0.05 px | 36/36 |
| 0.1 px | 36/36 |
| 0.2 px | 36/36 |

Important limitation: this is not yet a full-sky lost-in-space solver. It assumes the candidate catalog has
already been narrowed to a small local set. The next step is an angular-pair/triangle index over a public
catalog subset.

## Triangle Index Prototype

`scripts/build_star_triangle_index.py` builds a binned angular triangle index over a converted star catalog.
`scripts/identify_stars_with_index.py` uses that index to recover IDs from unlabeled observations without
brute-forcing every catalog triangle at query time.

Current public-catalog ambiguity benchmark:

| Catalog subset | Indexed stars | Indexed triangles | Observations | Correct IDs |
| --- | ---: | ---: | ---: | ---: |
| HYG v4.2 brightest | 120 | 34,858 | 24 | 24/24 |
| HYG v4.2 brightest | 180 | 121,705 | 24 | 24/24 |
| HYG v4.2 brightest | 240 | 293,642 | 24 | 24/24 |
| HYG v4.2 brightest | 360 | 958,157 | 24 | 24/24 |
| HYG v4.2 brightest | 500 | 2,524,698 | 16 | 16/16 |

Triangle candidates are verified by estimating a Wahba/Kabsch camera-inertial attitude hypothesis and
checking all observations against the indexed catalog. The verifier also uses a weak HYG magnitude prior
to break close binary or near-neighbor ties when angular error is nearly equal. This is still a prototype.
A production lost-in-space path needs a larger catalog partitioning strategy, faster index construction,
proper field-of-view selection, explicit ambiguity margins, and a stable non-pickle index format. The
500-star smoke already produces a 55 MB pickle and took about 373 s end-to-end in Python.

## Pair-Angle Index Prototype

`scripts/build_star_pair_index.py` stores catalog star pairs by angular separation instead of precomputing
every catalog triangle. `scripts/identify_stars_with_pair_index.py` reconstructs candidate triangles at
query time from the three observed pair angles, then reuses the same attitude verification gate.

`scripts/filter_star_catalog.py` removes stars closer than a configurable angular separation after sorting
by magnitude. This keeps unresolved close pairs such as bright visual binaries from dominating a benchmark
whose synthetic camera cannot resolve them.

Current HYG brightest resolved benchmark, 120 arcsec minimum catalog star separation:

| Indexed stars | Indexed pairs | Index size | Query avg | Correct IDs |
| ---: | ---: | ---: | ---: | ---: |
| 500 | 52,553 | 0.4 MB | 0.298 s | 24/24 |
| 1000 | 208,563 | 1.6 MB | 0.392 s | 24/24 |
| 2000 | 832,038 | 6.3 MB | 0.727 s | 24/24 |

Candidate pruning and vectorized catalog verification keep the query path fast at 2000 stars.

Robustness with 2000 indexed stars, 12 generated stars, 0.1 px noise, 2 trials per setting:

| Dropped | False | Correct true IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | 24/24 | 0 | 1.281 s |
| 0 | 4 | 24/24 | 0 | 2.393 s |
| 2 | 4 | 20/20 | 0 | 2.299 s |
| 4 | 4 | 16/16 | 0 | 1.560 s |

Observation scaling with 2000 indexed stars, 0.1 px noise, 2 trials per setting:

| Observed stars | Correct IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: |
| 16 | 32/32 | 0 | 2.873 s |
| 24 | 48/48 | 0 | 2.381 s |
| 32 | 64/64 | 0 | 2.454 s |

False detection scaling with 2000 indexed stars, 32 true stars, 0.1 px noise, 2 trials per setting:

| False detections | Correct true IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 2.426 s |
| 4 | 64/64 | 0 | 2.686 s |
| 8 | 64/64 | 0 | 2.664 s |
| 12 | 64/64 | 0 | 2.694 s |

Full false sweep at 4000 indexed stars, 32 true stars, 0.1 px noise, 2 trials per setting (3,329,812
indexed pairs, 25.2 MB index, 52.435 s build, 120 arcsec minimum catalog star separation). Phase-1
collection uses a pandas 3-way merge; phase-2 evaluates predicted angular distances with vectorized
`np.einsum` + `np.arccos`:

| False detections | Correct true IDs | Wrong IDs | Query avg | Candidate gen avg | Verify avg |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 8.059 s | 6.258 s | 0.364 s |
| 4 | 64/64 | 0 | 8.291 s | 6.557 s | 0.406 s |
| 8 | 64/64 | 0 | 7.836 s | 6.088 s | 0.335 s |
| 12 | 64/64 | 0 | 8.069 s | 6.374 s | 0.387 s |

Correctness holds at 4000 indexed stars across all tested false counts. Pandas-merge phase-1 plus
NumPy phase-2 reduces candidate-generation time by 10-25% per trial relative to the pre-optimization
baseline while preserving bit-exact `candidate_hypotheses` and `best_rms_error_arcsec`.

8000-star smoke (single-pass), 32 true stars, 0.1 px noise, 1 trial (13,320,198 indexed pairs,
101.2 MB index, 218.978 s build, 120 arcsec minimum catalog star separation):

| False detections | Correct true IDs | Wrong IDs | Query | Candidate gen | Verify |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 32/32 | 0 | 34.905 s | 30.861 s | 0.553 s |

Correctness still holds. The 4x pair-count growth (3.3M → 13.3M) drives a 7x candidate-hypothesis
growth (~50k → 366k) and a 5x query-time growth. The remaining engineering problem is the inner
candidate-generation algorithm; vectorized verification stays under one second.

Pyramid mode (`--pyramid-size 8`) builds observation triangles from only the first eight
observations and verifies the recovered attitude against all observations. At 4000 indexed stars
this collapses C(N, 3) sampled triangles down to C(8, 3) = 56:

| Indexed | False | Correct | Wrong | Cand gen avg | Verify avg | Query avg |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4000 | 0 | 64/64 | 0 | 0.611 s | 0.292 s | 1.653 s |
| 4000 | 4 | 64/64 | 0 | 0.539 s | 0.249 s | 1.466 s |
| 4000 | 8 | 64/64 | 0 | 0.534 s | 0.266 s | 1.484 s |
| 4000 | 12 | 64/64 | 0 | 0.629 s | 0.288 s | 1.578 s |
| 8000 | 0 | 64/64 | 0 | 4.067 s | 0.392 s | 5.817 s |
| 8000 | 4 | 64/64 | 0 | 3.470 s | 0.440 s | 5.333 s |
| 8000 | 8 | 64/64 | 0 | 3.844 s | 0.495 s | 5.691 s |
| 8000 | 12 | 64/64 | 0 | 3.900 s | 0.490 s | 5.589 s |
| 16000 | 0 | 64/64 | 0 | 4.367 s | 0.364 s | 6.128 s |
| 16000 | 4 | 64/64 | 0 | 3.574 s | 0.436 s | 5.406 s |
| 16000 | 8 | 64/64 | 0 | 4.075 s | 0.394 s | 5.786 s |
| 16000 | 12 | 64/64 | 0 | 4.335 s | 0.412 s | 6.068 s |

Pyramid mode preserves zero-wrong correctness across all tested false-detection counts and pushes
the working point from 4000 (single-pass wall) up through 8000 and 16000 indexed stars. The
16000-star benchmark above builds 16,249,850 pairs and a 123.5 MB pickle in 240.1 s; pair count
grows only ~1.2x from 8000 to 16000 because the `--max-edge-deg 80` and
`--min-star-separation-arcsec 120` filters saturate the HYG mag&le;6.5 catalog (8920 stars) well
before reaching 16000 indexed entries.

A new converted catalog `datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv`
(41,487 stars) lifts the saturation cap. A 16000-star pyramid smoke on this deeper catalog with
the historical defaults `--neighbor-bins 2 --tolerance-arcsec 300` indexed 53,279,484 pairs
(3.3x of mag&le;6.5 at the same nominal index size), wrote a 406 MB pickle / 182 MB `.npz` in
843 s, and reported 32/32 correct, 0 wrong assignments, candidate_hypotheses 451,789, cand_gen
47.1 s, query 51.5 s. Output at `outputs/hyg_pair_index_false_scaling_16000_mag8_smoke/`.

A follow-up full sweep with tightened parameters
(`--neighbor-bins 1 --tolerance-arcsec 120 --pyramid-size 8`,
`outputs/hyg_pair_index_false_scaling_16000_mag8_tight/`) recovered 64/64 correct with 0 wrong
across all four false counts, dropped candidate_hypotheses to ~33-41k (12-13x reduction),
candidate_generation to 9.97-11.96 s (about -75%), and total query to 13.99-16.58 s. The
0.1 px noise budget produces ~30 arcsec inter-pair edge error, well within the 120 arcsec
tolerance, so the tighter window is correctness-safe at this noise level.

Pyramid size 6 was found to be the operational sweet spot at honest density via a tuning sweep
on the 16000 mag&le;8 false=12 fixture: same 33/32 assigned and 90.47 arcsec RMS as size 8 but
~58% faster cand_gen. Size 4 fails (only 6 stars assigned, 333 arcsec RMS) because at 27% false
rate ~30% of pyramid attempts contain ≤2 true stars and admit no all-true triangle.

The 24000-indexed-star pyramid sweep on mag&le;8 with the same tight params and ps=8
(`outputs/hyg_pair_index_false_scaling_24000_mag8_tight/`) preserved 64/64 correct, 0 wrong:

| False detections | Correct | Wrong | Cand gen | Verify | Query |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 45.10 s | 0.661 s | 51.97 s |
| 4 | 64/64 | 0 | 34.85 s | 0.824 s | 41.76 s |
| 8 | 64/64 | 0 | 41.06 s | 0.916 s | 47.97 s |
| 12 | 64/64 | 0 | 45.41 s | 0.885 s | 52.57 s |

Re-running the same sweep with `--pyramid-size 6`
(`outputs/hyg_pair_index_false_scaling_24000_mag8_ps6/`) keeps the 64/64 / 0 wrong correctness
envelope and drops candidate generation by ~60% and total query by ~50%:

| False detections | Correct | Wrong | Cand gen | Verify | Query |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 13.84 s | 0.288 s | 20.66 s |
| 4 | 64/64 | 0 | 14.86 s | 0.356 s | 21.90 s |
| 8 | 64/64 | 0 | 15.30 s | 0.403 s | 22.50 s |
| 12 | 64/64 | 0 | 17.95 s | 0.417 s | 25.19 s |

The 32000-star pyramid full sweep with the same tight params and `--pyramid-size 6`
(`outputs/hyg_pair_index_false_scaling_32000_mag8_ps6/`) doubled the prior 16000 ceiling:
64/64 correct at every false count, query 49-55 s, build 472.8 s, 213 M pairs / 667 MB `.npz`.

The 40000-star sweep with the same params plus `--skip-pkl`
(`outputs/hyg_pair_index_false_scaling_40000_mag8_ps6/`) is the current correctness-clean
operating ceiling — essentially the absolute density limit, since the mag&le;8 catalog itself
caps at 41487 stars:

| False detections | Correct | Wrong | Cand gen | Verify | Query |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 0 | 49.56 s | 0.300 s | 61.05 s |
| 4 | 64/64 | 0 | 50.56 s | 0.326 s | 61.77 s |
| 8 | 64/64 | 0 | 58.97 s | 0.447 s | 70.42 s |
| 12 | 64/64 | 0 | 78.47 s | 0.538 s | 94.10 s |

Index 332,760,587 pairs / 1016 MB `.npz`, build 277.4 s vectorized. The `--skip-pkl` flag
(2026-05-08) bypasses the dict-of-tuples + pickle representation that peaks at ~30 GB for 332 M
pairs, and is required for any sweep at 40000+ on a typical workstation.

A separate ps=6 stress test at 16000 mag&le;8 with `--false-counts 16 24 32` (33%, 43%, 50%
false rate; `outputs/hyg_pair_index_false_scaling_16000_mag8_ps6_highfalse/`) recovered 64/64
true IDs with 0 wrong assignments at every level, query ~6 s in all cases. Query time is
essentially flat in false rate because the pyramid takes only the first 6 observations and
constraint is dominated by catalog density, not total observation count.

Filter and pair-index build are NumPy-vectorized as of 2026-05-08:

- `scripts/filter_star_catalog.py` checks the dot product against the kept-star matrix in one
  numpy operation. 24000-star filter dropped from an estimated ~30 min to 1.3 s.
- `scripts/build_star_pair_index.py` enumerates pairs in numpy batches per row of the catalog
  vector matrix, then groups into bins via `np.argsort` + `np.unique`. 4000-star build went from
  ~52 s to ~6 s with bit-exact bucketing.

Takeaway: parameter tightening + vectorized index construction + `--skip-pkl` take the working
point all the way to 40000 indexed stars on mag&le;8 within the same correctness envelope —
essentially the absolute density ceiling without re-converting HYG to mag&le;9 (~120 k stars).
ps=6 also tolerates 50% false rates without falling back to ps=8. Pushing past 40000 requires a
deeper-magnitude catalog AND algorithmic candidate-density reduction (sky-cell partitioning is
the most plausible next step).

`scripts/build_star_pair_index.py` now writes a parallel `.npz` archive next to the existing
`.pkl`. `scripts/identify_stars_with_pair_index.py` prefers the `.npz` when both are present and
falls back to `.pkl` otherwise (so existing benchmark fixtures keep working). The `.npz` format
stores parallel `int32` / `float64` arrays plus bin offsets, halves on-disk size (4000 stars:
26 MB pickle vs 13 MB npz), and is consumed via 2D ndarray slices with no per-pair Python tuple
construction. The 4000-star pyramid sweep above is the first benchmark recorded against the new
format; all four false counts retain bit-exact correctness vs the pickle baseline while shaving
another ~40% off candidate-generation time.

## Missing And False Star Robustness

`benchmarks/run_lost_in_space_robustness.py` injects missing detections and false detections into synthetic
unlabeled observations, then evaluates ID recovery.

Current local-catalog benchmark, 16 generated stars, 0.1 px noise, 3 trials per setting:

| Dropped true stars | False stars | Correct true IDs | Assigned IDs |
| ---: | ---: | ---: | ---: |
| 0 | 0 | 48/48 | 48/48 |
| 0 | 2 | 48/48 | 48/48 |
| 0 | 4 | 48/48 | 48/48 |
| 2 | 0 | 42/42 | 42/42 |
| 2 | 2 | 42/42 | 42/42 |
| 2 | 4 | 42/42 | 42/42 |
| 4 | 0 | 36/36 | 36/36 |
| 4 | 2 | 36/36 | 36/36 |
| 4 | 4 | 36/36 | 36/36 |

The prototype can reject false points in these controlled local-catalog cases. Public-catalog ambiguity is
now measurable through `benchmarks/run_hyg_ambiguity_benchmark.py` and
`benchmarks/run_hyg_pair_index_benchmark.py`; the current pair-index path reaches 2000 HYG brightest
resolved stars with all benchmark observations recovered, including tested missing and false detections.
