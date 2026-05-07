# Decisions

## 2026-05-07: Start with Simple Monocular VO

Decision: implement ORB/SIFT matching plus essential matrix pose recovery before adding SLAM,
factor graphs, or heavy terrain-relative navigation abstractions.

Reasoning: public lunar datasets can validate feature matching and frame-to-frame odometry quickly.
Scale, loop closure, and absolute localization need more dataset-specific work, so they should not
shape the initial architecture too early.

Consequences:

- Current trajectory is relative and unit-scale.
- Stereo, PnP, IMU, and LiDAR can be added as separate modules.
- Benchmark logs are prioritized over complex APIs.

## 2026-05-07: Keep Dataset Access Script Explicit

Decision: maintain a small hard-coded public dataset registry in `scripts/download_dataset.py`.

Reasoning: early users need reproducible commands and source manifests more than a generic data
platform. Explicit entries make URLs, citations, and size warnings visible.

Consequences:

- New datasets require a small registry edit.
- Large downloads require `--confirm-large`.
- Downloaded raw/extracted data is ignored by Git.

## 2026-05-07: Add Stereo PnP Before Full Stereo SLAM

Decision: add a small stereo triangulation plus PnP RANSAC baseline before implementing full stereo
odometry, bundle adjustment, or factor graph optimization.

Reasoning: POLAR Traverse provides calibrated stereo pairs, so metric scale can be tested immediately
without introducing SLAM state management.

Consequences:

- Metric path length can be compared against POLAR refined poses.
- Current baseline depends on raw-image stereo feature matches and needs rectification improvements.
- Failed PnP frames currently hold the previous pose rather than interpolating or dead reckoning.

## 2026-05-07: Rectify POLAR Stereo in the Dataset Adapter

Decision: perform OpenCV stereo rectification in `prepare_polar_traverse.py` before adding calibration
file parsing to the C++ VO app.

Reasoning: rectification is dataset-specific preparation work, and doing it in the adapter keeps the C++
MVP focused on VO. It also gives immediate metric benchmark improvement on public POLAR data.

Consequences:

- Prepared outputs include `rectified/`, `stereo_pairs_rectified.csv`, and `run_stereo_pnp_rectified.sh`.
- Rectified stereo is now the preferred metric baseline.
- Future C++ calibration loading can replace the generated-image workflow when broader dataset support is needed.

## 2026-05-07: Keep CLAHE Optional

Decision: add CLAHE as a CLI preprocessing option instead of enabling it by default.

Reasoning: CLAHE improves some difficult POLAR traverses, especially SIFT on Traverse4-5, but it does
not solve Traverse6 and can change feature distributions. Benchmark comparability is clearer when raw
images remain the default.

Consequences:

- Suite runs can compare raw and CLAHE results explicitly with `--clahe`.
- Future exposure sweeps should be evaluated both with and without CLAHE.

## 2026-05-07: Prototype Lost-In-Space in Python First

Decision: implement triangle-based lost-in-space star identification as a Python prototype before C++.

Reasoning: the main risk is algorithmic ambiguity and catalog indexing, not runtime integration. Python
lets us test angular tolerances, voting, and failure cases quickly.

Consequences:

- Current matcher assumes a small candidate catalog, not full-sky search.
- C++ implementation should wait until the indexed full-catalog approach is chosen.

## 2026-05-07: Use Binned Triangle Index for Next Lost-In-Space Step

Decision: use a binned angular triangle index as the next lost-in-space prototype.

Reasoning: brute-force triangle comparison is useful for validation but does not scale. Binning sorted
angular side lengths gives a simple path to larger public catalog subsets while keeping matching logic
inspectable.

Consequences:

- Current index is serialized as Python pickle and is not a stable file format.
- Index parameters such as bin size and max edge angle must be benchmarked before C++ implementation.
- False stars and ambiguous matches are still unhandled.

## 2026-05-07: Add False/Missing Detection Tests Before C++

Decision: keep lost-in-space in Python until false detections, missing detections, and larger-catalog
ambiguity are measured.

Reasoning: a C++ port would be premature if the matcher only works on perfect synthetic observations.
The Python robustness benchmark now gives a measurable gate for future implementations.

Consequences:

- Local-catalog false/missing tests pass; public-catalog ambiguity is now measured separately.
- The next implementation step is scoring/ambiguity handling, not C++ conversion.

## 2026-05-07: Verify Triangle Matches With Attitude Hypotheses

Decision: score indexed lost-in-space triangle candidates by estimating a Wahba/Kabsch attitude hypothesis
and rechecking all observations against the catalog.

Reasoning: triangle side lengths alone produce plausible but wrong permutations on public HYG subsets.
Pose verification gives a simple, inspectable second gate before committing to an ID assignment.

Consequences:

- `identify_stars_with_index.py` now reports verified hypotheses and best RMS bearing error.
- HYG benchmark subsets are selected by brightest stars, not raw catalog row order.
- `drop_star_ids.py` can write a shuffled truth map so benchmarks evaluate the unlabeled observation
  order correctly.
- HYG brightest 120/180/240-star subsets pass the current benchmark.
- Magnitude is only a weak tie-breaker; future camera models should use measured spot intensity and
  exposure metadata instead of catalog magnitude alone.

## 2026-05-07: Do Not Scale Full Triangle Enumeration Further

Decision: stop treating Python full-catalog triangle enumeration as the path to full-sky lost-in-space.

Reasoning: the matcher remains accurate at 360 and 500 brightest HYG stars, but index construction is
already expensive. The 500-star smoke generated 2,524,698 indexed triangles, a 55 MB pickle, and took
about 373 s end-to-end.

Consequences:

- The next star tracker task is index construction, not more tuning of query scoring.
- Candidate approaches: sky-cell partitioning, pair-angle indexes, limiting triangles by FOV density, or
  moving the index builder to C++ after the data structure is chosen.
- `benchmarks/run_hyg_ambiguity_benchmark.py` records index build time, query time, and index size for
  future scaling runs.

## 2026-05-07: Prefer Pair-Angle Index For Scaling

Decision: use a pair-angle index as the next scalable lost-in-space prototype.

Reasoning: full triangle indexing took about 373 s and 55 MB at 500 stars. Pair indexing reached 2000
stars with a 6.3 MB index and 24/24 correct IDs after filtering unresolved close stars. Candidate pruning
and vectorized verification reduced the 2000-star query average to 0.727 s.

Consequences:

- The next bottleneck is larger observed star sets and more realistic false-star distributions, not index build.
- `filter_star_catalog.py` is part of the benchmark pipeline because unresolved close pairs are not
  meaningful truth targets for the current synthetic camera.
- Current query pruning limits per-observation-triangle candidates and total verified hypotheses before
  Wahba/Kabsch verification.
- HYG pair-index robustness at 2000 indexed stars recovered all remaining true IDs with zero wrong
  assignments for drop/false counts up to 4 in the current benchmark.
- Observed-star scaling at 2000 indexed stars remains stable through 32 observed stars with zero wrong
  assignments in the current benchmark.
- False-star scaling at 2000 indexed stars remains stable through 32 true stars plus 12 false detections.
- Observation triangles are sampled uniformly across all combinations instead of taking the first N
  combinations; prefix selection was biased by shuffled observation order and failed in false-star cases.
- A 4000-indexed-star density smoke recovered 64/64 true IDs with 0 wrong assignments at false 0 and
  false 12, with a 25.2 MB index built in 65.443 s and a query average of 9.838 s. Correctness holds,
  but query latency now grows quickly with index size.
- The full 4000-indexed-star false sweep (`false_counts 0 4 8 12`) recovered 64/64 true IDs with 0
  wrong assignments at every setting; query avg 8.689-9.311 s. Per-stage timing inside
  `identify_stars_with_pair_index.py` reports candidate generation 7.346-7.932 s and verification
  0.382-0.472 s per query, confirming candidate generation as the dominant cost at this density.

## 2026-05-07: Profile Candidate Generation Before Scaling Past 4000

Decision: do not jump from 4000 to 8000 indexed stars yet. Profile candidate generation in
`identify_stars_with_pair_index.py` and reduce candidate-set size first.

Reasoning: at 4000 stars the pair index already contains 3,329,812 pairs and the per-query candidate
hypotheses sit in the 49k-57k range before pruning. Vectorized Wahba/Kabsch verification is no longer
the dominant cost; candidate generation and pair-list intersections are. Doubling indexed stars without
addressing this would push query latency well past the current ~10 s and would not produce a useful
scaling signal.

Consequences:

- Per-stage timings (`candidate_generation_seconds`, `pruning_seconds`, `verification_seconds`) are
  now emitted by `identify_stars_with_pair_index.py` and aggregated by
  `benchmarks/run_hyg_pair_index_false_scaling.py`.
- The 4000-indexed-star full false sweep confirms candidate generation dominates: 7.346-7.932 s per
  query versus 0.382-0.472 s for verification. Pruning is negligible.
- Candidate-list adjacency caching by `(edge_bin, neighbor_bins)` is the first optimization to try.
- An 8000-star smoke is deferred until candidate generation is improved or until a deliberate worst-case
  measurement is needed.

## 2026-05-08: Vectorize `filter_star_catalog.py` and `build_star_pair_index.py`

Decision: rewrite both scripts to use NumPy batched operations instead of per-pair Python
`math.acos` loops. Keep input/output schemas unchanged.

Reasoning: at higher catalog density both scripts hit Python loop walls. The filter at 24000
stars from a 41487-star catalog is O(N*K) with `K` growing each iteration — when first attempted
it ran for 25+ minutes and was killed. The pair-index builder at 24000 stars enumerates
C(24000, 2) ≈ 288M pairs in pure Python. A naive estimate gives 32+ minutes; in practice the
real wall is similar.

Empirical confirmation:

- `filter_star_catalog.py` 24000-star run: from estimated 30 min (killed) to 1.3 s with the
  vectorized dot-product check (`kept_array @ current_direction` vs `cos(min_separation_rad)`).
- `build_star_pair_index.py` 4000-star run: 52 s → 5.9 s with batched per-row `arccos` plus
  `np.argsort` / `np.unique` bucketing. Bit-exact bucketing verified against the old build
  (`bin_mismatches: 0/2395` on the 4000-star fixture).
- 24000-star pair index now builds in 227 s. 32000-star builds in 430 s. Without the
  vectorization, neither was tractable in this session.

Consequences:

- The vectorized builds enable 24000-star full sweeps and 32000-star smokes within minutes
  instead of an hour.
- Memory cost is modest: per-row temp arrays of size `(N - i - 1, 3)`. At 32000 stars the
  largest allocation is ~768 KB.
- The scripts still write the same pickle / npz / json outputs. Existing fixtures remain valid.

## 2026-05-08: Pyramid Size 6 Confirmed at 24000 / 32000 Honest Density

Decision: `--pyramid-size 6` is the operational default for honest-density (mag&le;8) sweeps.
The ~58% cand_gen reduction observed in the 16000 tuning sweep scales to higher densities.

Reasoning: re-running the 24000 mag&le;8 full sweep with ps=6 (vs the previous ps=8 baseline)
preserved 64/64 correctness across all four false counts and dropped cand_gen from 35-45 s to
14-18 s (~60% lower) and query from 42-53 s to 21-25 s (~50% lower). Re-running the 32000 smoke
as a full sweep with ps=6 turned a 167 s "past practical" single-trial smoke into a 49-55 s
full sweep with 64/64 correct. Output at
`outputs/hyg_pair_index_false_scaling_{24000,32000}_mag8_ps6/`.

Consequences:

- 32000 indexed stars on mag&le;8 is now correctness-clean at operational latency. This is the
  new largest workable density without any new algorithm.
- The single-pass ps=8 path is still in the code as a regression baseline. ps=6 is the
  recommended setting for benchmarks and production-style runs at honest density.
- For workloads with > 40% false-detection rate, ps=8 may still be needed; the 27% rate in our
  benchmark fixtures fits comfortably in the ps=6 envelope.
- Sky-cell / bitset-adjacency optimizations (which were on the table after the ps=8 32000 wall)
  are deprioritized further. They re-enter the picture only at 48000+ density or for false rates
  above 40%.

## 2026-05-08: Pyramid Size 6 Is the Operational Sweet Spot at Honest Density

Decision: when running on honest-density catalogs (mag&le;8 or deeper) use `--pyramid-size 6` as
the operational default. Keep the script CLI default at 0 (single-pass) for backwards
compatibility, but switch documentation and recommended invocations to size 6.

Reasoning: a controlled tuning sweep at 16000 mag&le;8 (false=12 trial=1, tight params, repeated
twice per setting) gave:

| pyramid_size | candidate_hypotheses | cand_gen | RMS arcsec | assigned |
| ---: | ---: | ---: | ---: | ---: |
| 4 | 2,349 | 0.57-0.74 s | 333.9 | 6/32 (FAIL) |
| 6 | 17,998 | 4.76-5.25 s | 90.47 | 33/32 |
| 8 | 44,080 | 12.08-13.43 s | 90.47 | 33/32 |
| 10 | 82,816 | 24.55-29.56 s | 90.47 | 33/32 |
| 12 | 159,637 | 45.35-47.70 s | 89.99 | 33/32 |

Size 4 fails because at 27% false rate the probability of getting `<3` true stars in the picked
subset is ~30% — too many trials lack a valid all-true triangle. Size 6 already has
`P(>=3 true) >= 99%` and is ~58% faster than size 8.

Consequences:

- The 16000 mag&le;8 full sweep at size 8 (10-12 s cand_gen) likely runs in 4-5 s with size 6.
- The 24000 mag&le;8 sweep currently in flight uses size 8 for safety; rerun with size 6 once
  the 24000 baseline is recorded.
- `--pyramid-size 4` should not ship as a default; it needs an explicit "low false rate" guard.
- The decision is workload-conditioned: at higher false rates (>40%), size 8 or larger may be
  required again. Document the false-rate vs pyramid-size relationship in
  `docs/space_localization.md`.

## 2026-05-08: Tighten Default Pair-Index Tolerance to 120 arcsec / Neighbor-Bins 1

Decision: when running `identify_stars_with_pair_index.py` against an honest-density catalog
(mag&le;8 or deeper), pass `--tolerance-arcsec 120 --neighbor-bins 1` rather than the historical
defaults `300 / 2`. Keep the script-level defaults at `300 / 2` for backwards compatibility with
existing benchmark output dirs.

Reasoning: the inter-pair edge error budget at 0.1 px noise on a 1000 px focal length is roughly
`2 * 0.1/1000 rad = 2e-4 rad = ~40 arcsec`. The historical 300 arcsec tolerance was set with a
generous safety margin. At catalog-saturated density (mag&le;6.5 at 16000 indexed stars) this
generosity costs little because pair density per bin is small. At honest density (mag&le;8 at
16000 stars, 53M pairs) it triples the per-query candidate set and turns query latency into a
50 s wall.

Empirical confirmation: a same-fixture A/B at 16000 mag&le;8 (false=0 trial=0) gave
`(neighbor_bins=2, tol=300)` 451,789 candidates / 38.4 s cand_gen vs `(neighbor_bins=1, tol=120)`
35,988 candidates / 10.4 s cand_gen, with bit-exact `assigned_observations`,
`verified_hypotheses`, and `best_rms_error_arcsec`. The full sweep at 16000 mag&le;8 with the
tighter parameters preserved 64/64 correctness across false counts 0/4/8/12 and dropped
candidate_generation to 9.97-11.96 s.

Consequences:

- The next benchmarks at honest density should use the tighter parameters by default.
- The script defaults remain `300 / 2` so existing fixtures and benchmark output dirs continue to
  reproduce.
- Sky-cell / HEALPix partitioning of the pair index is deprioritized: it would have been the next
  algorithmic step at 47 s cand_gen but is unnecessary at the new 10 s baseline.
- If observation noise rises (e.g., 1 px) the tolerance must be relaxed proportionally.
- `count_correct` reports `assigned > total` when a denser catalog has stars near false-detection
  observation indices (the recovered attitude pulls them in via `verify_rotation`). This is not a
  correctness regression: `correct` and `wrong` columns track the truth-vs-assignment metric.

## 2026-05-08: HYG mag&le;6.5 Catalog Saturates Around 9000 Indexed Stars

Decision: when comparing scaling sweeps, treat the mag&le;6.5 HYG catalog as a fixed 8920-star
ceiling. Results that nominally use 16000 indexed stars on this catalog are actually re-indexed
8920-star catalogs after the 120 arcsec separation filter. To honestly probe higher densities,
use `datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv` (41,487 stars).

Reasoning: the 16000-star sweep on mag&le;6.5 produced 16,249,850 pairs only ~1.2x the 8000-star
sweep's 13,320,198 pairs, and the filtered catalog file at `--index-size 16000` had 8840 rows. A
deeper mag conversion confirmed: at true 16000-star density on mag&le;8 the index has 53,279,484
pairs (3.3x), candidate_hypotheses jumps from ~85k to ~452k, and cand_gen jumps from ~4 s to
~47 s.

Consequences:

- Past benchmarks that report 16000 indexed stars on the default catalog should be read as
  "8920 effective stars, replicated through the 16000 index slot" rather than as honest density
  scaling.
- Future scaling work should specify which catalog was used. Two clean operating points are now
  recorded: 16000 on mag&le;6.5 (saturated) and 16000 on mag&le;8 (honest).
- The next algorithmic effort (sky-cell partitioning, candidate-density reduction) should be
  measured against the mag&le;8 16000 baseline since that is where the true bottleneck appears.
- The `.npz` format size advantage holds at higher densities: 16000 on mag&le;8 is 406 MB pickle
  vs 182 MB npz (55% smaller).

## 2026-05-08: `.npz` Pair-Index Format Alongside Pickle

Decision: `scripts/build_star_pair_index.py` now writes both the existing `.pkl` and a new `.npz`
archive containing parallel `int32` / `float64` arrays. `scripts/identify_stars_with_pair_index.py`
prefers `.npz` when present and falls back to `.pkl`, so existing fixtures keep working without
regeneration.

Reasoning: pickle indices reached 124 MB at 16000 indexed stars and serialize a Python
`dict[int, list[tuple[int, int]]]` whose loader iterates Python tuples per pair. At 16000 the
loader rebuild dominated query latency (~5 s of dict reconstruction on top of every query). A
flat numpy layout (`bin_keys[]`, `bin_offsets[]`, `pair_endpoints[][2]`, plus catalog vectors and
magnitudes) keeps the same logical schema, halves on-disk size, and is consumable directly via
2D ndarray slices.

Consequences:

- 4000-star index: 26 MB `.pkl` vs 13 MB `.npz` (~50% smaller).
- Loader rebuild on `.npz` returns ndarray buckets without per-pair tuple construction. Profiling
  showed dict rebuild dropped from ~5.3 s to ~0.003 s at 4000 stars.
- `load_candidate_pairs()` and `adjacency()` now accept both ndarray and list-of-tuple buckets so
  the .pkl path still works.
- The 4000-star pyramid full sweep on `.npz` is bit-exact vs the `.pkl` pyramid baseline:
  identical `candidate_hypotheses` per fixture, identical `best_rms_error_arcsec`, 64/64 correct
  with 0 wrong at every false count. Output at
  `outputs/hyg_pair_index_false_scaling_4000_pyramid8_npz/`.
- Side benefit: pyramid + `.npz` cand_gen drops 0.90-1.08 s → 0.53-0.63 s (a further ~40%
  reduction on top of pyramid mode).
- The format is also a prerequisite for the eventual C++ port: `.npz` is a documented zip-of-arrays
  format and can be read from C without depending on Python pickle.
- Backwards compatibility: any user still on a `.pkl`-only index continues to work; the new
  `index_format` metadata key in the assignment JSON makes which path was used inspectable.

## 2026-05-08: Pyramid Mode (`--pyramid-size`) for Faster Triangle Generation

Decision: add an opt-in pyramid mode to `scripts/identify_stars_with_pair_index.py` that builds
observation triangles only from the first N observations, while still running attitude verification
against all observations. CLI flag: `--pyramid-size N` (default 0 = disabled). Plumb the flag
through `benchmarks/run_hyg_pair_index_false_scaling.py` as well.

Reasoning: per-query work is dominated by `len(observation_triangles)` (default 400 sampled from
C(N, 3)). With 32 true plus 12 false observations there are C(44, 3) = 13,284 raw triangles, each
calling `candidate_mappings()`. The pair-index matcher already verifies hypotheses against every
observation in `verify_rotation`, so the only role of the rest of the triangles is to find an
attitude. A small high-quality subset of observations (e.g., 8) is enough: with 32/44 = 73% true
rate, P(≥3 true in 8) ≈ 99%, so at least one valid all-true triangle is virtually guaranteed.

Consequences:

- 4000-indexed-star sweep with `--pyramid-size 8` recovers 64/64 with 0 wrong assignments at every
  tested false count. Cand_gen drops 6.09-6.56 s (pandas baseline) → 0.90-1.08 s, an 82-86%
  reduction. Query avg 2.48-2.83 s. Output at
  `outputs/hyg_pair_index_false_scaling_4000_pyramid8/`.
- 8000-indexed-star full sweep with the same setting recovers 64/64 with 0 wrong at every false
  count. Cand_gen 4.42-5.63 s (vs 30.86 s for the single-pass 8000 smoke). Query avg 8.77-10.24 s.
  Output at `outputs/hyg_pair_index_false_scaling_8000_pyramid/`.
- 16000-indexed-star full sweep (post-pyramid follow-up) recovers 64/64 with 0 wrong at every
  false count. Cand_gen 5.07-7.44 s, query 9.88-12.60 s, build 248.8 s, index 123.5 MB,
  16,249,850 pairs. Output at `outputs/hyg_pair_index_false_scaling_16000_pyramid/`.
- Pair count from 8000 to 16000 grows only 1.2x because `--max-edge-deg 80` and
  `--min-star-separation-arcsec 120` saturate at this density. Cand_gen scales sublinearly with
  catalog density once the filter regime kicks in.
- candidate_hypotheses count at 4000 drops from ~50k to ~6-8k per query — fewer hypotheses also
  means lower memory pressure during phase-2 vectorization.
- Pyramid mode's main risk is the first-N selection: observations are shuffled randomly, so the
  pyramid is a random subset. With 27% false rate and pyramid_size 8, expected true count in
  pyramid is ~5.8 (well above the 3 needed). For workloads with higher false rates (e.g., 50%+),
  raise `--pyramid-size`.
- The metadata JSON now records `pyramid_size` and `triangle_pool_size` so reruns are reproducible.
- 16000 indexed stars is the new largest correctness-preserving operating point. The next
  engineering problem moves from query latency to index serialization (pickle is awkward at >100
  MB) and build time. Defer the next density step until `Recommended Next Tasks #5` (stable `.npz`
  index format) is addressed.

## 2026-05-08: Pandas 3-Way Merge for Phase-1 Triple Collection

Decision: replace the Python `for` loop in `candidate_mappings()` phase 1 with a pandas 3-way merge
over the symmetrized `pairs_ab`, `pairs_ac`, and `pairs_bc` DataFrames.

Reasoning: NumPy vectorization of the inner predictions (the same-day decision below) only sped up
the second half of the function. Phase-1 collection of `(oriented_a, oriented_b, cat_c)` triples was
still a Python loop with ~14,000 set intersections per observation triangle and 400 triangles per
query, totalling ~5.6M Python set operations per query. A symmetric SQL-style 3-way join is the
natural shape of the work and pandas implements it in C.

Consequences:

- Bit-exact correctness across the full `false_counts 0 4 8 12` sweep at 4000 indexed stars:
  identical `candidate_hypotheses` (46,645-57,122) and `best_rms_error_arcsec` to the baseline.
- Sweep-average candidate-generation drops 10.7-19.6% per false count, with consistent per-trial
  improvement (no per-trial regression). Output at
  `outputs/hyg_pair_index_false_scaling_4000_pandas/`.
- The 8000-star smoke now succeeds on this implementation: 32/32 correct, 0 wrong,
  `candidate_generation_seconds = 30.861 s`, `verification_seconds = 0.553 s`,
  `index_pairs = 13,320,198`, `index size = 101.2 MB`, `build = 218.978 s`. Output at
  `outputs/hyg_pair_index_false_scaling_8000_smoke/`.
- Adds a `pandas` import to `scripts/identify_stars_with_pair_index.py`. The dependency is already
  available in the project Python environment.
- `candidate_hypotheses` and `verified_hypotheses` semantics are unchanged because the merge
  produces the same set of triples that the Python loop produced (after dedup against `seen`).

## 2026-05-08: NumPy Vectorize Candidate-Generation Inner Predictions

Decision: replace the scalar inner loop in `candidate_mappings()` with a two-phase implementation.
Phase 1 collects unique `(oriented_a, oriented_b, cat_c)` triples through Python iteration over
`pairs_ab` and the adjacency intersection. Phase 2 bulk-evaluates the three predicted angular
distances with `np.einsum("ij,ij->i", ...)` plus `np.arccos`, applies the tolerance filter, and
computes scores in vectorized form.

Reasoning: profiling at 4000 indexed stars showed candidate generation at ~7.4-7.9 s per query and
~85% of total query latency. The dominant cost was the three `angular_distance(catalog_vectors[i],
catalog_vectors[j])` scalar calls per accepted candidate (`np.dot + math.acos`). With ~50,000
candidates per query, a single bulk arccos is much cheaper than 150,000 scalar arccos calls.

Consequences:

- Bit-exact correctness preserved across the false 0/4/8/12 sweep at 4000 indexed stars: identical
  `candidate_hypotheses`, `verified_hypotheses`, and `best_rms_error_arcsec` to the pre-vectorization
  run.
- Standalone three-run smokes on the same fixture show median candidate-generation drops of -2% to
  -15% depending on input density. Benchmark numbers are dominated by system noise and not
  individually conclusive, but the standalone reproducibility is clean.
- `seen` is now updated at collection time rather than after the tolerance check. No observable
  semantic change because `pairs_ab` does not contain duplicate pairs in HYG v4.2 indices.
- `catalog_magnitudes` is now passed as a NumPy array to enable fancy indexing inside
  `candidate_mappings()`. `verify_rotation` already normalizes via `np.asarray`, so no callsite
  change was needed there.
- The PLAN's 8000-star smoke condition (`candidate_generation_seconds < 3 s`) is not met by this
  optimization alone, so 8000 is deferred to the next algorithmic step (sky-cell / HEALPix
  partitioning).

## 2026-05-07: Edge-Bin Adjacency Cache Did Not Improve Candidate Generation

Decision: do not ship a per-query edge-bin adjacency / pair-list cache for `candidate_mappings()`.
Move the next optimization attempt to NumPy vectorization of the inner loop instead.

Reasoning: a `QueryCache` that memoized angular distances per `(obs_i, obs_j)` and pair-list /
adjacency maps per `edge_bin` was implemented and benchmarked at index size 4000 with the full
`false_counts 0 4 8 12` sweep. Correctness was preserved (64/64, 0 wrong), but per-query
`candidate_generation_seconds` was statistically indistinguishable from baseline (7.15-7.91 s with
cache vs 7.35-7.93 s without; standalone reruns showed 10-15% variance from system load alone).
Cache hit rate is low because 44 observations with C(44,2)=946 unique edges spread across
roughly 2400 distinct edge bins, so most bin lookups are unique and per-call dict-lookup overhead
cancels the savings. A separate predicted-edge hoist optimization actively regressed false=12 by
~4x because it moved `angular_distance(cat_a, cat_b)` out of the `for cat_c in shared` loop where
it was already gated by an empty-shared early-exit.

Consequences:

- `scripts/identify_stars_with_pair_index.py` retains only the per-stage timing instrumentation;
  the inner candidate-generation loop is unchanged from the pre-experiment baseline.
- Archived benchmark output for the failed experiments lives at
  `outputs/hyg_pair_index_false_scaling_4000_optimized/` and
  `outputs/hyg_pair_index_false_scaling_4000_cached/`.
- The next candidate-generation optimization should target NumPy vectorization of the inner loop:
  bulk-compute `predicted_ab/ac/bc` for a batched `(cat_a, cat_b, cat_c)` array instead of
  per-tuple scalar `np.dot + math.acos` calls. The angular distance call is the per-iteration cost
  most amenable to vectorization.
- Sky-cell / HEALPix partitioning of the catalog remains the second-line option if vectorization
  alone is insufficient at 8000+ indexed stars.

## 2026-05-08: 40000 mag&le;8 Confirms ps=6 At The Catalog Density Ceiling

Decision: 40000 indexed stars on HYG mag&le;8 with `--pyramid-size 6 --neighbor-bins 1
--tolerance-arcsec 120` is the new operational ceiling for cold-start lost-in-space identification,
and ps=6 is correctness-safe through 50% false detection rates at 16000 honest density. Do not
fall back to ps=8 for high-false-rate workloads.

Reasoning: the 40000 full sweep (false 0/4/8/12, 2 trials) recovered 64/64 true IDs with 0 wrong
assignments at every false count, with worst-case query 94 s — under PLAN's 2-minute acceptance.
The mag&le;8 catalog itself caps at 41487 stars, so 40000 is essentially the absolute density
ceiling without re-converting HYG to mag&le;9 (~120k stars), which would push build time well past
2 hours even with the vectorized path. Separately, the 16000 mag&le;8 high-false-rate stress
(`--false-counts 16 24 32`, i.e. 33%/43%/50% false rate) returned 64/64 correct, 0 wrong at every
setting with ~6 s query — query time is essentially flat in false rate because the pyramid only
takes the first 6 observations and constraint is dominated by catalog density.

Consequences:

- The README headline result is now 40000 (mag&le;8) / 64 correct / 332 M pairs / 1016 MB `.npz` /
  61-94 s query, replacing the prior 32000 entry.
- `--pyramid-size 6` is the operational default for honest-density mag&le;8 work at all densities
  from 16000 through 40000. ps=8 stays in the codebase as a regression baseline only.
- Pushing past 40000 requires a deeper-magnitude catalog conversion (mag&le;9 ~ 120k stars) AND
  an algorithmic candidate-density reduction (sky-cell partitioning is still the most plausible
  next step). Both are deferred until a concrete need arises (e.g. real-camera mag-limit testing
  shows we need stars below mag 8).
- The 40000-star build needs `--skip-pkl` because the dict-of-tuples + pickle path peaks at ~30 GB
  RAM for 332 M pairs. Documented separately in the next decision below.

## 2026-05-08: Add `--skip-pkl` To `build_star_pair_index.py` For 40k+ Builds

Decision: gate the dict-of-tuples + `pickle.dump` step in `build_star_pair_index.py` behind a new
`--skip-pkl` flag, derive the `.npz` arrays directly from the sorted numpy buffers, and forward the
flag from `benchmarks/run_hyg_pair_index_false_scaling.py`. Keep dual-write as the default for
backwards compatibility with archived `.pkl` fixtures and the loader's `.pkl` fallback.

Reasoning: the first 40000 sweep died with SIGKILL during `build_star_pair_index.py`. The vectorized
pair-enumeration produces compact int64 numpy arrays (~8 GB at 332 M pairs), but the subsequent
`index[bin_key] = list(zip(sorted_lhs.tolist(), sorted_rhs.tolist()))` materializes 332 M Python
tuples of Python ints, which costs ~30 GB at peak — the OOM killer fires on a 62 GB box once the
buffer cache and other processes are accounted for. The `.npz` writer downstream rebuilds the same
arrays from the dict, so removing the dict step is purely a memory optimization with no functional
effect. Bit-exact equivalence verified on a 500-star fixture: `bin_keys`, `bin_offsets`,
`pair_endpoints`, `vectors`, and `magnitudes` all `np.array_equal=True` between `--skip-pkl` and
the dual-write build.

Consequences:

- 40000 build runs in 277 s with bounded memory (sorted int64 buffers, ~7 GB peak).
- `metadata.pkl_path` is `None` when `--skip-pkl` is set; downstream tooling that wants the index
  size should stat the `.npz` instead. The benchmark switches to `index_path.with_suffix(".npz")`
  for `index_size_mb` when `--skip-pkl` is on.
- Loader (`identify_stars_with_pair_index.py`) needs no change: it already prefers `.npz` and falls
  back to `.pkl` only when `.npz` is absent.
- Future bigger builds (mag&le;9, sky-cell partitioned, etc.) should default to `--skip-pkl`.

## 2026-05-08: Realism Reveals A ~17% Per-Trial Catastrophic Failure At High False Rates

Decision: the `64/64 correct, 0 wrong` numbers on idealized synthetic data were too optimistic at
high false detection rates. Under a realistic camera-effects configuration
(`--limiting-magnitude 7.0 --mag-softness 0.5 --false-near-fraction 0.5 --false-near-sigma-px 20.0`)
at 16000 mag&le;8 ps=6, the pyramid identifier hits a roughly 17% per-trial catastrophic-failure
rate at 37.5% false detection rate. The failing trial assigns 0 observations correctly and produces
4-6 confidently-wrong assignments. Increasing `--pyramid-size` from 6 to 8 does not help — the
larger pyramid produces 3.2x more candidate hypotheses but locks onto the same confusion attitude.

Reasoning: trials=6 sweeps with the realism config show 5/6 success at false 0/4/8/12 except false=12
where 5/6 trials succeed (192/192 → 160/192). Two ablations confirm the failure is not isolated to
one axis: mag-only realism fails at false=4 in trials=2 runs, false-near-only fails at false=12,
combined fails at false=12. The pattern is fixture-driven — random attitudes occasionally land near
catalog regions that admit more than one self-consistent triangle pattern under the algorithm's
matching tolerance, and the pyramid identifier locks onto the wrong one. The algorithm has no
mechanism to detect "I picked the wrong attitude": it returns the first verified hypothesis without
checking whether the predicted vs observed star count is consistent under the limiting magnitude.

Consequences:

- Update README and PLAN to caveat the headline 40000 / 64-correct number as idealized.
- Future LIS work should add multi-hypothesis verification or attitude-rejection signals before
  claiming flight-grade robustness. Concrete options:
  - Keep top-K candidate attitudes by verified-match-count, run full verification on each, pick
    the one with the most matches (and reject ties / low confidence).
  - Track expected vs predicted observation counts under the recovered attitude and the limiting
    magnitude prior; reject attitudes that predict far more or far fewer matches than observed.
  - Re-pyramid with a different observation subset when verification falls below a threshold
    rather than accepting the first match.
- The realism flags themselves (`--limiting-magnitude`, `--false-near-fraction`, etc.) stay as
  optional flags. Default behavior is unchanged so prior fixtures and `count_correct` semantics
  continue to work.
- Pure idealized correctness benchmarks (Gaussian-noise-only synthetic) remain valid as
  algorithmic regression baselines but no longer count as the headline robustness statement.
