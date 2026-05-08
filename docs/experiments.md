# Experiments

This project follows `experiment -> converge`: add small baselines, compare them on public data, keep
the winning implementation readable, and avoid premature abstraction.

## ORB vs SIFT

| Method | Expected strengths | Expected risks | Metrics |
| --- | --- | --- | --- |
| ORB | Fast, binary descriptors, simple CPU baseline | Less robust under extreme lighting and low texture | matches/frame, inliers/frame, runtime, trajectory drift |
| SIFT | Stronger invariance and matching quality | Slower, floating descriptors | matches/frame, inliers/frame, runtime, trajectory drift |

Initial protocol:

1. Run both methods on the same POLAR Traverse view.
2. Save logs with `benchmarks/run_visual_odometry_benchmark.py`.
3. Compare inlier ratio, failed-frame count, and relative trajectory smoothness.

## Essential Matrix vs PnP

| Pose method | Input requirement | MVP status | Notes |
| --- | --- | --- | --- |
| Essential matrix | Calibrated monocular image pair | Implemented | Relative scale only |
| PnP | 2D-3D correspondences from stereo/depth/map landmarks | Planned | Needed for metric localization |

Convergence target: keep essential matrix as the monocular fallback, add PnP once stereo depth or
terrain landmark maps are represented in a dataset adapter.

## Crater Detection Methods

| Method | Status | Notes |
| --- | --- | --- |
| Hough circles | Implemented baseline | Works only for near-circular image-space crater rims |
| Edge + ellipse fitting | Planned | Better for oblique views |
| Learned segmentation | Future | Requires public labels or synthetic training data |
| Orbital-map crater matching | Future | Needed for absolute TRN |

## Lunar Dataset Comparison

| Dataset | Best use | Strength | Limitation |
| --- | --- | --- | --- |
| NASA POLAR Traverse | Visual odometry | Sequential stereo rover-like imagery with pose/calibration | Laboratory analogue, large downloads |
| NASA POLAR Stereo | Stereo and crater/terrain perception | HDR scenes, rocks/craters, LiDAR ground truth | Mostly independent scenes, not long traverses |
| LunarLoc | Segment/global localization | Simulator traverses with playback utilities | Format-specific adapter needed |
| Apollo imagery | Terrain appearance and crater context | Real lunar surface imagery | Not rover odometry sequences |
| Synthetic Lunar Terrain | Multimodal perception | RGB/event/laser scan in analogue terrain | Adapter and file selection still needed |

## Result Log

Add new rows here when running benchmarks:

| Date | Dataset | Command | Result summary | Decision |
| --- | --- | --- | --- | --- |
| 2026-05-07 | Scaffold | N/A | Initial baselines added | Start with ORB essential matrix |
| 2026-05-07 | POLAR Traverse View1 Traverse1 L 50ms | `benchmarks/run_visual_odometry_benchmark.py --images outputs/polar_view1_traverse1_left_50ms/images.txt --ground-truth outputs/polar_view1_traverse1_left_50ms/refined_poses.tsv` | ORB: 10/11 initialized-or-ok frames, 1 failed motion, avg 133.0 matches, avg 87.8 inliers, Sim(3) ATE RMSE 0.309 m, RPE translation RMSE 0.364 m. SIFT: 11/11 initialized-or-ok frames, 0 failed motions, avg 205.5 matches, avg 165.1 inliers, Sim(3) ATE RMSE 0.0186 m, RPE translation RMSE 0.0283 m. | Use SIFT as robustness reference; keep ORB as fast baseline |
| 2026-05-07 | POLAR Traverse View1 Traverse1 stereo 50ms | `build/apps/stereo_visual_odometry --pairs outputs/polar_view1_traverse1_left_50ms/stereo_pairs.csv ...` | ORB stereo + PnP: 11/11 initialized-or-ok frames, SE3 ATE RMSE 0.243 m, RPE translation RMSE 0.0835 m, path length 10.72 m vs 9.98 m GT. | Metric scale is now available; next improve stereo matching/rectification |
| 2026-05-07 | POLAR Traverse View1 Traverse1 rectified stereo 50ms | `outputs/polar_view1_traverse1_left_50ms/run_stereo_pnp_rectified.sh` | ORB rectified stereo + PnP: 11/11 initialized-or-ok frames, SE3 ATE RMSE 0.0650 m, RPE translation RMSE 0.0251 m, path length 10.18 m vs 9.98 m GT. | Use rectified stereo as metric VO baseline |
| 2026-05-07 | HYG v4.2 brightest subsets | `benchmarks/run_hyg_ambiguity_benchmark.py --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv --index-sizes 120 180 240 --trials 3 --stars 8 --noise-px 0.1 --max-edge-deg 80` | Lost-in-space triangle index with pose verification and magnitude prior: 120 stars 24/24 correct, 180 stars 24/24 correct, 240 stars 24/24 correct. | Continue scaling catalog size and measuring ambiguity before C++ port |
| 2026-05-07 | HYG v4.2 brightest scaling | `benchmarks/run_hyg_ambiguity_benchmark.py --index-sizes 360` and `--index-sizes 500` | 360 stars: 958,157 triangles, 24/24 correct. 500 stars: 2,524,698 triangles, 16/16 correct, 55 MB pickle, 373 s end-to-end run. | Accuracy remains good; all-triangle Python index build is now the bottleneck |
| 2026-05-07 | HYG v4.2 pair-index scaling | `benchmarks/run_hyg_pair_index_benchmark.py --index-sizes 500 1000 2000 --min-star-separation-arcsec 120` | Pair index with candidate pruning and vectorized verification: 500 stars 24/24 correct, 0.298 s avg query; 1000 stars 24/24, 0.392 s; 2000 stars 24/24, 0.727 s. | Pair-angle index is the preferred scalable prototype |
| 2026-05-07 | HYG v4.2 pair-index robustness | `benchmarks/run_hyg_pair_index_robustness.py --index-size 2000 --stars 12 --drop-counts 0 2 4 --false-counts 0 2 4 --trials 2` | All drop/false settings recovered every remaining true ID with 0 wrong assignments. Worst listed setting drop 0 / false 4: 24/24 correct, 0 wrong, 2.393 s avg query. | Pair-index remains stable under missing and false detections in this benchmark |
| 2026-05-07 | HYG v4.2 pair-index observed-star scaling | `benchmarks/run_hyg_pair_index_observation_scaling.py --index-size 2000 --star-counts 16 24 32 --trials 2` | 16 observed stars: 32/32 correct, 2.873 s avg query. 24: 48/48, 2.381 s. 32: 64/64, 2.454 s. No wrong assignments. | Current pruning keeps query stable as observed star count increases |
| 2026-05-07 | HYG v4.2 pair-index false scaling | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 2000 --stars 32 --false-counts 0 4 8 12 --trials 2` | False 0/4/8/12 all recovered 64/64 true IDs with 0 wrong assignments; false 12 query avg 2.694 s. | Uniform observation-triangle sampling fixes false-star robustness at high observed star count |
| 2026-05-07 | HYG v4.2 pair-index 4000-star density smoke | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 4000 --stars 32 --false-counts 0 12 --trials 2` | 4000 indexed stars, 3,329,812 pairs, 25.2 MB pickle, 65.443 s index build. False 0 and 12 both recovered 64/64 true IDs with 0 wrong assignments; query avg 9.838 s. Candidate hypotheses 49,359-56,411 before pruning. | Correctness holds at 4000; bottleneck is now candidate generation, not verification. Profile and reduce candidate-set size before scaling further |
| 2026-05-07 | HYG v4.2 pair-index 4000-star full false sweep | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 4000 --stars 32 --false-counts 0 4 8 12 --trials 2` | All four false counts recovered 64/64 true IDs with 0 wrong assignments. Query avg 8.689-9.311 s. Per-stage timing added: candidate generation 7.346-7.932 s, verification 0.382-0.472 s. | Confirms correctness across the full false sweep at 4000 indexed stars. Candidate generation is ~85% of query time, so the next optimization target is pair-list intersection / adjacency reuse, not verification |
| 2026-05-07 | HYG v4.2 pair-index `QueryCache` candidate-generation experiment | Same command as above, plus a per-query edge-bin adjacency cache and pair-list cache | Correctness preserved (64/64, 0 wrong). Candidate generation 7.15-7.91 s — statistically indistinguishable from the no-cache baseline once standalone-rerun variance (10-15%) is accounted for. A separate predicted-edge hoist regressed false=12 by ~4x and was abandoned. | Reverted the cache and the hoist. Kept the per-stage timing instrumentation. Move the next attempt to NumPy vectorization of the inner loop. Archived runs at `outputs/hyg_pair_index_false_scaling_4000_optimized/` and `outputs/hyg_pair_index_false_scaling_4000_cached/` |
| 2026-05-08 | HYG v4.2 pair-index NumPy vectorization of `candidate_mappings()` | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 4000 --stars 32 --false-counts 0 4 8 12 --trials 2` | Bit-exact correctness preserved (64/64, 0 wrong, identical `candidate_hypotheses=46645-57122` and `best_rms_error_arcsec=28.229...` to the baseline). Standalone three-run smokes show candidate generation drop on the same fixture: false_0/1 7.46→7.30 s (-2%), false_4/1 7.49→6.79 s (-9%), false_12/1 7.70→6.54 s (-15%). The benchmark sweep is dominated by system noise and shows mixed per-trial deltas, so the standalone medians are the cleaner comparison. | Keep the vectorization. Bit-exact correctness plus modest reproducible standalone improvement. PLAN's 8000-star smoke condition (cand_gen < 3 s) is not met, so 8000 is deferred. Next algorithmic step is sky-cell / HEALPix partitioning to reduce the per-query candidate density |
| 2026-05-08 | HYG v4.2 pair-index pandas 3-way merge in `candidate_mappings()` phase 1 | Same command as above, plus `pandas.DataFrame.merge` over symmetrized `pairs_ab/ac/bc` to enumerate triples in C rather than Python loops | Bit-exact correctness preserved across the full sweep (64/64, 0 wrong, identical `candidate_hypotheses=46645-57122` and `best_rms_error_arcsec`). Per-trial cand_gen drops 7.9-22.5% relative to the original baseline with no per-trial regression. Sweep avg cand_gen: false=0 7.41→6.26 s (-15.6%), false=4 7.35→6.56 s (-10.7%), false=8 7.42→6.09 s (-17.9%), false=12 7.93→6.37 s (-19.6%). Output at `outputs/hyg_pair_index_false_scaling_4000_pandas/`. | Keep the pandas-merge phase 1 plus the NumPy phase 2. This is the first optimization with consistent across-the-board improvement and bit-exact correctness |
| 2026-05-08 | HYG v4.2 pair-index 8000-star smoke (post-pandas-merge) | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 8000 --stars 32 --false-counts 0 --trials 1` | 8000 indexed stars, 13,320,198 pairs, 101.2 MB pickle, 218.978 s index build. Recovered 32/32 true IDs with 0 wrong assignments. cand_gen 30.861 s, verify 0.553 s, query 34.905 s total. candidate_hypotheses jumped to 365,881. | Correctness still holds at 8000, but cand_gen latency grows ~5x relative to 4000 because the pair count grows 4x and the per-query candidate hypotheses explode 7x. The remaining win has to come from algorithmic candidate-density reduction (sky-cell partitioning), not constant-factor optimization |
| 2026-05-08 | HYG v4.2 pair-index Pyramid mode at 4000 (`--pyramid-size 8`) | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 4000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8` | 64/64 correct, 0 wrong across all four false counts. Cand_gen avg 0.902-1.083 s (-82% to -86% vs pandas-merge baseline). Query avg 2.482-2.829 s. candidate_hypotheses dropped from ~50k to ~6.5-8.0k. | Pyramid mode is the first algorithmic optimization that brings candidate-generation cleanly under the 3 s threshold the PLAN set as a precondition for trying 8000+. Correctness preserved across the false sweep |
| 2026-05-08 | HYG v4.2 pair-index Pyramid full sweep at 8000 (`--pyramid-size 8`) | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 8000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8` | 64/64 correct, 0 wrong across all four false counts. Cand_gen avg 4.425-5.630 s (-82% to -86% vs the 30.861 s single-pass smoke at the same density). Query avg 8.771-10.236 s. candidate_hypotheses ~52k-64k. Index build 198.295 s. | Pyramid mode at 8000 hits roughly the same per-query latency as the pre-pyramid 4000 baseline, doubling the supported indexed-star density without losing correctness. This is the first concrete path past the 4000 wall |
| 2026-05-08 | HYG v4.2 pair-index Pyramid full sweep at 16000 (`--pyramid-size 8`) | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 16000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8` | 64/64 correct, 0 wrong across all four false counts. Index 123.5 MB / 16,249,850 pairs (only 1.2x of 8000 because `--max-edge-deg 80` and `--min-star-separation-arcsec 120` dominate). Index build 248.768 s. Cand_gen 5.069-7.439 s, query 9.882-12.601 s, candidate_hypotheses ~70k-85k. | 16000 indexed stars is the new largest correctness-preserving operating point. Cand_gen scales sublinearly with catalog density thanks to filter saturation. The remaining engineering problem moves to index serialization (pickle is unwieldy at >100 MB) and build time, not query latency |
| 2026-05-08 | HYG v4.2 pair-index `.npz` format at 4000 (`--pyramid-size 8`) | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 4000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8` (after switching `build_star_pair_index.py` to dual-write `.pkl` + `.npz` and updating the loader to prefer `.npz`) | 64/64 correct, 0 wrong across all four false counts. `.npz` size 13 MB vs `.pkl` 26 MB. `assignments.json` reports `index_format: "npz"`. Cand_gen 0.534-0.629 s (vs 0.902-1.083 s on the same pyramid sweep with the old pickle dict-of-tuples loader). Query 1.466-1.653 s. | Keep the dual-write. `.npz` halves on-disk size and reduces cand_gen ~40% by replacing per-pair Python tuple construction with 2D ndarray slices. Backwards-compatible: the loader falls back to `.pkl` if no `.npz` is alongside, so existing benchmark fixtures still work |
| 2026-05-08 | HYG v4.2 pair-index `.npz` 8000 / 16000 pyramid full sweeps | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 8000/16000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8` | 64/64 correct, 0 wrong everywhere. 8000: cand_gen 3.5-4.1 s, query 5.3-5.8 s (vs 4.4-5.6 / 8.8-10.2 with old loader). 16000: cand_gen 3.6-4.4 s, query 5.4-6.1 s (vs 5.1-7.4 / 9.9-12.6). | `.npz` saves 25-50% of cand_gen and ~40-50% of total query at higher densities while keeping correctness bit-exact |
| 2026-05-08 | HYG v4.2 pair-index 16000 smoke on deeper mag&le;8 catalog | `benchmarks/run_hyg_pair_index_false_scaling.py --catalog datasets/star_catalogs/hyg-v42/converted/hyg_v42_mag8p0_unit.csv --index-size 16000 --stars 32 --false-counts 0 --trials 1 --pyramid-size 8` | 32/32 correct, 0 wrong. Indexed pairs 53,279,484 (3.3x of the 16,249,850 reached on mag&le;6.5 at the same nominal index size). Pickle 406 MB / `.npz` 182 MB. Index build 843 s. candidate_hypotheses 451,789 (vs ~85k on mag&le;6.5). Cand_gen 47.1 s, query 51.5 s. | The previous 16000 results on mag&le;6.5 were catalog-saturated. At true 16000-star density the candidate-generation algorithm hits a new bottleneck (~450k hypotheses per query). Algorithmic candidate-density reduction (sky-cell partitioning, tighter pre-filter) is the prerequisite to operate at full HYG mag&le;8 density |
| 2026-05-08 | HYG v4.2 pair-index 16000 mag&le;8 with tight params | `benchmarks/run_hyg_pair_index_false_scaling.py --catalog hyg_v42_mag8p0_unit.csv --index-size 16000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8 --tolerance-arcsec 120 --neighbor-bins 1` | 64/64 correct, 0 wrong across all four false counts. candidate_hypotheses ~33-41k (vs 451,789 on the loose-param smoke). Cand_gen 9.97-11.96 s (-75%). Query 13.99-16.58 s (-72%). The false=12 row reports `assigned 66/64`, which is `count_correct` registering one false-detection observation index that happens to coincide with a catalog star under the recovered attitude — `correct` and `wrong` are unaffected. | Parameter tightening (`--neighbor-bins 1 --tolerance-arcsec 120`) is correctness-safe at the current 0.1 px noise level (~30 arcsec inter-pair error). It closes ~75% of the cand_gen gap between catalog-saturated mag&le;6.5 and honest mag&le;8 16000 regimes. Sky-cell partitioning is deprioritized; it now matters only above 16000 mag&le;8 |
| 2026-05-08 | Pyramid-size tuning at 16000 mag&le;8 (false=12 trial=1, tight params) | `identify_stars_with_pair_index.py` direct invocation with `--pyramid-size {4,6,8,10,12}` against the saved index | size=4: 6/32 assigned, RMS 333 arcsec — fails because picking only 4 observations from a 27% false-rate workload leaves ≤2 valid all-true triangles roughly 30% of the time. size=6: 33/32 assigned, RMS 90.47, cand_gen 4.76-5.25 s. size=8: 33/32, RMS 90.47, cand_gen 12.08-13.43 s. size=10: 33/32, RMS 90.47, cand_gen 24.55-29.56 s. size=12: 33/32, RMS 89.99, cand_gen 45.35-47.70 s. (33 = 32 true + 1 false-coincidence at this fixture density.) | `--pyramid-size 6` is the operational sweet spot for honest-density mag&le;8 work: same correctness as size 8 but ~58% faster cand_gen. Move benchmark default to 6 once the 24000 sweep confirms it scales. Keep size 4 disabled until a more aggressive false-rejection prefilter is added. |
| 2026-05-08 | Vectorize `filter_star_catalog.py` and `build_star_pair_index.py` | Source code change: replace per-pair `acos` loops with NumPy batched operations | Filter 24000 from mag&le;8 catalog: ~30 min (estimated, killed in flight) → 1.3 s. Pair-index build at 4000 stars: 52 s → 5.9 s with bit-exact bucketing (verified by `bin_mismatches: 0/2395`). 24000-star build: 226.8 s. 32000-star build: 430.0 s. | Vectorized index construction is the unlock for 24000+ honest-density experiments. Without it, 24000 build alone would take >30 min, and 32000 would take >1 hour. Bit-exact correctness is preserved because the new code computes the same `np.arccos(np.clip(np.dot(...)))` value as the old scalar path |
| 2026-05-08 | HYG v4.2 pair-index 24000 mag&le;8 with tight params | `benchmarks/run_hyg_pair_index_false_scaling.py --catalog hyg_v42_mag8p0_unit.csv --index-size 24000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 8 --tolerance-arcsec 120 --neighbor-bins 1` | 64/64 correct, 0 wrong across all four false counts. Indexed pairs 119,879,891 (2.25x of 16000 mag&le;8). Pickle 913.6 MB / `.npz` 407 MB. Build 226.8 s. candidate_hypotheses 109k-139k. Cand_gen 34.85-45.41 s. Query 41.76-52.57 s. | 24000 indexed stars on mag&le;8 is the new largest correctness-preserving full-sweep operating point. Per-query latency is ~50 s, which is acceptable for cold-start LIS but slow for online use |
| 2026-05-08 | HYG v4.2 pair-index 32000 mag&le;8 smoke with tight params | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 32000 --stars 32 --false-counts 0 --trials 1 --pyramid-size 8 --tolerance-arcsec 120 --neighbor-bins 1` | 32/32 correct, 0 wrong. Indexed pairs 213,021,414. Pickle 1624 MB / `.npz` 667 MB. Build 430.0 s. candidate_hypotheses 282,826. Cand_gen 155.69 s. Query 167.64 s. | 32000 indexed stars on mag&le;8 is the new correctness ceiling. Query latency at ~3 min is past practical use as a single-pass matcher, but the smoke confirms the pipeline is correct at this density. Going further requires algorithmic candidate-density reduction (sky-cell partitioning, pyramid two-pass with attitude-aware second pass, or a sorted/bitset adjacency intersection) |
| 2026-05-08 | HYG v4.2 pair-index 24000 mag&le;8 with ps=6 | Same as the 24000 ps=8 row but `--pyramid-size 6` | 64/64 correct, 0 wrong across all four false counts. candidate_hypotheses 40-55k (vs 109-139k at ps=8). Cand_gen 13.84-17.95 s (-60%). Query 20.66-25.19 s (-50%). | Confirms the ps=6 sweet spot scales: same correctness, ~60% cand_gen reduction at 24000 honest density |
| 2026-05-08 | HYG v4.2 pair-index 32000 mag&le;8 full sweep with ps=6 | Same as the 32000 smoke row but `--false-counts 0 4 8 12 --trials 2 --pyramid-size 6` | 64/64 correct, 0 wrong across all four false counts. candidate_hypotheses 96-130k (vs 283k at ps=8 smoke). Cand_gen 35.92-43.65 s (-75% vs ps=8 smoke). Query 49.16-55.19 s (-70%). | 32000 indexed stars on mag&le;8 is now operationally workable for cold-start LIS. The ps=6 + tight-params + vectorized-build + .npz combination doubled the largest correctness-preserving full-sweep operating point from 16000 (May 8 morning) to 32000 (same day evening) without any new algorithm |
| 2026-05-08 | `--skip-pkl` mode in `build_star_pair_index.py` | Source change: gate the dict-of-tuples + pickle.dump behind a new `--skip-pkl` flag and derive the `.npz` arrays directly from the sorted numpy buffers. Forward the flag from `run_hyg_pair_index_false_scaling.py` | Bit-exact `.npz` (`bin_keys`/`bin_offsets`/`pair_endpoints`/`vectors`/`magnitudes` all `np.array_equal=True` against the dual-write build at 500 stars). 40000-star initial run died with SIGKILL because the dict-of-tuples representation + pickle pass for 332M pairs needed ~30 GB peak. With `--skip-pkl`, peak memory stays bounded by the sorted int64/int32 numpy buffers (~7 GB at 40k). | Use `--skip-pkl` for any sweep at 40000+. The pickle path remains the default at smaller densities for backwards compatibility with archived fixtures and the loader's `.pkl` fallback |
| 2026-05-08 | HYG v4.2 pair-index 40000 mag&le;8 full sweep with ps=6 (`--skip-pkl`) | `benchmarks/run_hyg_pair_index_false_scaling.py --catalog hyg_v42_mag8p0_unit.csv --index-size 40000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 6 --tolerance-arcsec 120 --neighbor-bins 1 --skip-pkl` | 64/64 correct, 0 wrong across all four false counts. Indexed pairs 332,760,587 (1.56x of 32000). `.npz` 1016 MB. Build 277.4 s (vs 430 s at 32000 — vectorized scaling stays sublinear because filter saturation dominates). candidate_hypotheses 186k-254k. Cand_gen 49.6-78.5 s. Query 61-94 s. false=12 reports `assigned 66/64` (two false-detection observation indices coincide with catalog stars under recovered attitude; `correct=64` `wrong=0` unaffected). | 40000 indexed stars on mag&le;8 is the new correctness-preserving full-sweep ceiling. The mag&le;8 catalog itself caps at 41487 stars, so this is essentially the absolute density ceiling without re-converting HYG to mag&le;9. Worst-case query (~94 s at 50% false rate equivalent) is still under PLAN's 2-minute acceptance threshold |
| 2026-05-08 | HYG v4.2 pair-index 16000 mag&le;8 ps=6 high-false-rate stress (`--false-counts 16 24 32`) | `benchmarks/run_hyg_pair_index_false_scaling.py --catalog hyg_v42_mag8p0_unit.csv --index-size 16000 --stars 32 --false-counts 16 24 32 --trials 2 --pyramid-size 6 --tolerance-arcsec 120 --neighbor-bins 1 --skip-pkl` | 64/64 correct, 0 wrong at every false count. false=16 (33% false rate): query 6.09 s. false=24 (43%): 5.97 s. false=32 (50%): 6.15 s. Query is roughly constant in false rate because the pyramid only takes the first 6 observations and constraint is dominated by catalog density rather than total observation count. `assigned` counts of 65/65/67 are the same false-coincidence semantics as the prior tight-params runs. | ps=6 is robust through 50% false detections at 16000 mag&le;8. No need to fall back to ps=8 for high-false-rate workloads at this density. PLAN step 5 acceptance is met |
| 2026-05-08 | Realism flags added: probabilistic mag-weighted detection + near-real-star false positives | Source change: `generate_star_tracker_observations_from_catalog.py` gets `--limiting-magnitude` (sigmoid `1/(1+exp((mag-limit)/softness))`) and `--mag-softness`. `drop_star_ids.py` gets `--false-near-fraction` and `--false-near-sigma-px` (Gaussian offset around a random real observation). `run_hyg_pair_index_false_scaling.py` forwards both. | Smoke at limiting=7.0 / softness=0.5 selects 32 stars with mean mag 6.46 (vs near-3.85 under top-3N) and tail to 7.75. Smoke at near-frac=0.5 / sigma=20 yields nearest-real-distance p10/p50/p90 of 14.5 / 56.0 / 119.6 px (vs ~100+ uniform). | Both axes wired through. Defaults preserve the previous idealized behavior. |
| 2026-05-08 | HYG v4.2 pair-index 16000 mag&le;8 ps=6 with full realism enabled (initial trials=2) | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 16000 --stars 32 --false-counts 0 4 8 12 --trials 2 --pyramid-size 6 --tolerance-arcsec 120 --neighbor-bins 1 --skip-pkl --limiting-magnitude 7.0 --mag-softness 0.5 --false-near-fraction 0.5 --false-near-sigma-px 20.0` | false 0/4/8: 64/64 correct, 0 wrong (~5-6 s query). false=12: **32/64 correct, 4 wrong** — one trial of two completely failed. First regression observed under realistic camera effects. | The 64/64 numbers on idealized data were too optimistic at high false rates. Need to characterize whether the failure is stochastic or systematic |
| 2026-05-08 | Realism ablations: mag-only vs false-near-only at 16000 mag&le;8 ps=6 trials=2 | mag-only run with `--limiting-magnitude 7.0` (no false-near). false-near-only run with `--false-near-fraction 0.5 --false-near-sigma-px 20.0` (top-3N selection). | mag-only: 64/64 at false 0/8/12, **32/64 at false=4**. false-near-only: 64/64 at false 0/4/8, **32/64 at false=12**. Both ablations exhibit single-trial complete failures, but at different false counts — pattern suggests fixture-specific stochastic failures rather than a systematic break tied to either axis alone. | Single-trial failures cluster around different false counts in different ablations, so the regression is fixture-driven (random attitude landing in confusion regions). Need more trials to characterize the per-trial failure rate |
| 2026-05-08 | Realism trials=6 characterization at 16000 mag&le;8 ps=6 | Same realism config as initial run, but `--trials 6` | false 0/4/8: 192/192 correct (6/6 trials each). **false=12: 160/192, 4 wrong, 5/6 trials correct (~17% per-trial failure rate)**. The single failing trial reports `correct=0, wrong=4, assigned=8` — a confusion-attitude that locks onto wrong matches and rejects most observations. | At ~37.5% false rate plus realistic camera effects, the pyramid identifier hits a non-trivial (~17%) catastrophic-failure rate per trial. Idealized 64/64 / 0 wrong was hiding this regime |
| 2026-05-08 | Realism `--pyramid-size 8` mitigation test, trials=6 | Same realism config but `--pyramid-size 8` | false 0/4/8: 192/192. **false=12: 160/192, 6 wrong, 5/6 trials correct (same ~17% per-trial failure rate)**. The failing trial at ps=8 has `candidate_hypotheses=52616, verified=378, wrong=6` vs ps=6's `16446 / 149 / 4` — ps=8 produces 3.2x more candidates and 2.5x more verified hypotheses but still locks onto the same wrong attitude. Cand_gen ~2x slower than ps=6 (5.6-6.6 s vs 1.9-3.0 s). | Pyramid size does not fix the realism regression. The failure is fundamentally about the matching algorithm picking a confusion attitude that produces a self-consistent (but wrong) match. Algorithmic mitigation (multi-hypothesis verification, observation-count-aware re-pyramid, magnitude-prior in candidate scoring) is needed before claiming flight-grade realism |
| 2026-05-08 | Pyramid-restart implementation in `identify_stars_with_pair_index.py` | Source change: add `--pyramid-restarts`, `--confidence-fraction`, `--pyramid-restart-seed`. Wrap the existing pyramid+verify loop in an outer restart loop. After each attempt, if `assigned/observations < confidence-fraction`, shuffle the observation permutation and retry. The attempt with the most assignments wins regardless. Default `--pyramid-restarts=0` preserves prior behavior bit-exactly. | Code change only; smoke results in the next two rows. | The restart loop is gated by a confidence threshold rather than blindly retrying, so successful trials pay no extra cost. Attempts metadata (`attempts_taken`, `winning_attempt_index`) is now recorded in `assignments.json` |
| 2026-05-08 | Pyramid restart fixes the realism failure: trials=6 + `--pyramid-restarts 3 --confidence-fraction 0.5` | `benchmarks/run_hyg_pair_index_false_scaling.py --index-size 16000 --stars 32 --false-counts 0 4 8 12 --trials 6 --pyramid-size 6 --tolerance-arcsec 120 --neighbor-bins 1 --skip-pkl --limiting-magnitude 7.0 --mag-softness 0.5 --false-near-fraction 0.5 --false-near-sigma-px 20.0 --pyramid-restarts 3 --confidence-fraction 0.5` | false 0/4/8: 192/192 correct (6/6 each). **false=12: 192/192 correct, 0 wrong, 6/6 trials succeed**. Trial 0 (the one that previously failed) used 2 attempts and the winning attempt was index 1 (the first restart). All other trials succeeded on attempt 0 (no restart cost). Average query for false=12 dropped slightly to 5.21 s (vs 5.54 s without restart) because successful trials pay no penalty and only the failing trial's cand_gen doubled (5.0 s vs ~2.0 s). | The restart loop closes the realism robustness gap: 17% per-trial catastrophic failure rate → 0% within these 6 trials. Cost is amortized: only the failing fraction pays the 2x cand_gen tax. ps=6 + restart=3 is now the recommended operational config under realism; ps=8 is no longer needed |
| 2026-05-08 | Realism + restart trials=24 statistical-power tightening | Same realism+restart config but `--trials 24` | **768/768 correct, 0 wrong at every false count**. Attempts distribution across 96 trials: 86/96 attempt=1, 8/96 attempt=2, 1/96 attempt=3, 1/96 attempt=4. Net: 10/96 (~10.4%) trials needed at least one restart. Per-false-count query avg: 4.14 / 4.72 / 5.57 / 7.60 s (false 0/4/8/12). The single 4-attempts trial at false=12 paid ~4x cand_gen tax. | Rule-of-three upper bound on residual catastrophic-failure rate: **`<3.1%` at 95% CI** (vs `<17%` from the no-restart baseline). The restart fix is statistically robust within this realism configuration |
| 2026-05-08 | Magnitude-dependent centroid noise: source change | Add `--noise-mag-reference` and `--noise-mag-cap-px` to `generate_star_tracker_observations_from_catalog.py`. Per-star sigma = `noise_px * 10^(0.4 * (mag - reference))`, capped. Forward through `generate_visible_case` and the false-scaling benchmark. Default disabled. | Smoke at `noise-px=0.1` `noise-mag-reference=6.0` produced per-star residuals: mag 3.85 → 0.013 px, mag 6.30 → 0.170 px, mag 7.45 → 0.399 px — matches `0.1 * 10^(0.4 * (mag - 6))` within stochastic deviation. | Centroid noise scaling models the photon-limited regime: bright stars saturate (low noise), faint stars near limiting magnitude have noise growing as `10^(0.4 * Δmag)`. The third realism axis after detection probability and false-near positioning |
| 2026-05-08 | Realism + restart + mag-scaled noise sweep, trials=6 | Same realism+restart config plus `--noise-mag-reference 6.0` | **192/192 correct, 0 wrong at every false count.** false=12 cand_gen 4.47 s (1.7x of constant-noise baseline) and candidate_hypotheses 24,573 (1.6x of 15,406) — faint-star noise lets more pair edges fall inside the 120 arcsec tolerance window. Attempts distribution across 24 trials: 21/24 attempt=1, 2/24 attempt=2, 1/24 attempt=4 (12.5% needed restart, similar to constant-noise's 10.4%). | The mag-scaled noise axis does not break correctness when stacked on top of the existing realism config (mag-weighted detection + near-real false). It does cost ~1.7x candidate generation at false=12 because the wider effective tolerance produces more candidate triples |
| 2026-05-08 | HYG mag&le;9 catalog conversion | `convert_star_catalog.py --max-magnitude 9.0` | 83,479 stars, 5.4 MB. ~2.0x of mag&le;8's 41,487. | Deeper magnitude catalog needed to push past the mag&le;8 catalog density ceiling. Stars are now in scope for indices up to 80k |
| 2026-05-08 | int32 intermediate buffers in `build_star_pair_index.py` | Source change: switch the `pair_lhs_chunks`/`pair_rhs_chunks`/`pair_bin_chunks` and the concatenated `all_*` / `sorted_*` arrays from int64 to int32. Free the chunks list and the unsorted arrays as soon as they are no longer needed. | First 60000 mag&le;9 smoke died with SIGKILL (OOM) under the prior int64 path even with `--skip-pkl`. With int32: 60000 mag&le;9 builds successfully in 654 s peak ~30 GB. Bit-exact `.npz` equivalence verified on a 500-star fixture (`bin_keys`/`bin_offsets`/`pair_endpoints`/`vectors`/`magnitudes` all `np.array_equal=True` against the prior int64 build). | Catalog indices fit comfortably in int32 (max ~80k vs int32 max 2.1G), bin keys are in the low thousands, so int32 is sufficient. Halves intermediate memory; combined with explicit `del` of the chunks list and unsorted arrays, lets 60k mag&le;9 fit on a 62 GB workstation |
| 2026-05-08 | HYG mag&le;9 60000-star ps=6 smoke (false=0 trials=1) | `benchmarks/run_hyg_pair_index_false_scaling.py --catalog hyg_v42_mag9p0_unit.csv --index-size 60000 --stars 32 --false-counts 0 --trials 1 --pyramid-size 6 --tolerance-arcsec 120 --neighbor-bins 1 --skip-pkl` | **32/32 correct, 0 wrong**. Indexed pairs 747,979,225 (2.25x of 40k mag&le;8's 332M). `.npz` 2195.8 MB. Build 654.5 s (vectorized scaling holds; 40k mag&le;8 was 277 s, so the 2.25x pair growth costs ~2.4x more build time, still sub-linear in catalog density due to filter saturation). candidate_hypotheses 701,199 (4.7x of 40k mag&le;8's ~150k). Cand_gen 269.9 s, query 294.4 s. | 60000 mag&le;9 is the new correctness-extension datapoint: density goes past the mag&le;8 catalog ceiling and the algorithm still finds the right attitude. But query latency (~5 min) is past practical use as a single-pass cold-start matcher. The next ceiling jump requires algorithmic candidate-density reduction — sky-cell / HEALPix partitioning becomes the prerequisite for routine mag&le;9 operation |
| 2026-05-08 | Proper-motion realism axis: pmra/pmdec plumbing + stale-catalog drift | Source change: `convert_star_catalog.py` now emits `pmra_mas_yr`/`pmdec_mas_yr` columns. `generate_star_tracker_observations_from_catalog.py` gets `--apply-proper-motion-years N`: drift each catalog direction's RA/Dec by N years before projecting to the camera. Catalog stored in `catalog.csv` stays at J2000 — only the observed (u,v) reflect the drift, simulating a stale-catalog deployment. Forwarded through `generate_visible_case` and `run_hyg_pair_index_false_scaling.py`. mag&le;8 catalog re-converted with the new columns. | Smoke at years=200: median pixel drift 0.05 px (= ~10 arcsec), top-5 high-pm star drift 0.4 px (Talitha at 80 arcsec). Smoke at years=500: top high-pm stars (Groombridge 1830 at 7058 mas/yr, Lacaille 9352 at 6896 mas/yr) drift 1400+ arcsec, well past the 120 arcsec verification tolerance. | New realism axis tests catalog freshness — what happens when the deployed catalog is N years stale relative to the observed sky |
| 2026-05-08 | Realism + restart + proper-motion=500 years sweep, trials=6 | Same realism+restart config plus `--apply-proper-motion-years 500` | **766/768 correct (99.7%), 0 wrong across 24 trials**. Per-false-count: false=0 190/192, false=4 190/192, false=8 191/192, false=12 192/192. Each "30/32" trial is one fixture where 2 high-pm stars drifted past the per-star verification tolerance, but the recovered attitude is still correct (other 30 stars matched). Cand_gen avg at false=12 4.65 s (1.7x of no-pm baseline) — drifted observation positions widen the effective per-pair tolerance window. Wrong=0 everywhere — no high-pm star was misidentified as another catalog star. | The matcher tolerates ~500 years of catalog staleness with **graceful degradation**: attitude estimation is unaffected, only individual high-pm stars (Groombridge 1830, Lacaille 9352, etc.) get rejected at verify-time. This is the correct behavior for a stale-catalog deployment. The recovered attitude could be re-used to re-project drifted positions and recover the rejected stars in a second pass if needed |
| 2026-05-08 | Hot-pixel false-positive realism axis | Source change: `drop_star_ids.py` gets `--hot-pixel-count`, `--hot-pixel-seed`, `--hot-pixel-fraction`, `--hot-pixel-sigma-px`. K hot pixel positions are sampled deterministically from `--hot-pixel-seed` so the same sensor layout repeats across fixtures. False-detection placement: hot-pixel fraction takes priority, then near-real-star, then uniform — partitioned so the three modes do not double-count. Forwarded through the false-scaling benchmark. | Smoke at hot-fraction=0.5 hot-count=8 sigma=1: 15/32 false detections fell within 5 px of one of the 8 hot pixels (~47%, matches the budget partition). The remaining 17 split between the existing near-real (when active) and uniform-random fallbacks. | Models fixed image-coordinate sensor noise (dead pixels, hot pixels, persistent read-noise spikes) — biases false detections to specific (u, v) regardless of attitude, complementary to `--false-near-fraction` which biases near real stars |
| 2026-05-08 | 5-axis realism + restart sweep, trials=6 | All five realism axes stacked: `--limiting-magnitude 7.0 --mag-softness 0.5 --false-near-fraction 0.5 --false-near-sigma-px 20.0 --noise-mag-reference 6.0 --apply-proper-motion-years 200 --hot-pixel-fraction 0.5 --hot-pixel-count 8 --hot-pixel-sigma-px 1.0 --pyramid-restarts 3 --confidence-fraction 0.5` at 16000 mag&le;8 ps=6 | **767/768 correct (99.87%), 0 wrong across 24 trials**. Per-false-count: false=0 191/192 (one trial 31/32, presumably a pm=200 high-pm-star miss); false 4/8/12 all 192/192. Cand_gen at false=12 4.86 s; candidate_hypotheses 34,281 (1.4x of the 4-axis pm=500 baseline). verified_hypotheses 343 — hot-pixel false clusters generate more triples that pass tolerance. Attempts distribution: 17/24 attempt=1, 4/24 attempt=2, 1/24 attempt=3, **2/24 attempt=4** (used the full restart budget). | All five realism axes stacked still preserve correctness via restart. The 2/24 trials that used all 4 attempts are close to the restart-budget cliff; consider `--pyramid-restarts 4` for production use under aggressive realism stacks. The matcher remains realism-validated within the stronger combined regime |
| 2026-05-09 | C++ port skeleton: binary pair index loader | Python build script gets `--write-bin` that emits a flat zlib-free format alongside `.npz`: 72-byte header (magic / version / counts / bin params), then raw `vectors` / `magnitudes` / `bin_keys` / `bin_offsets` / `pair_endpoints` arrays, then a length-prefixed `star_ids` blob. New C++ loader `astro_localization::localization::load_pair_index_bin` populates a `PairIndex` struct. `apps/lost_in_space_pair_id` CLI loads the index and prints a JSON summary matching the Python build. | 500-star fixture: stars=500 / bins=2394 / pairs=101,124, bin_arcsec=120, edges 0.2/80 — bit-exact match between C++ summary and Python JSON. 16000 mag&le;8 fixture: stars=16000 / bins=2395 / pairs=88,885,824, `.bin` size 712 MB (uncompressed, vs `.npz` 292 MB compressed). C++ loads without issue. | First C++ deliverable in the LIS port. Subsequent sessions add `candidate_mappings`, `verify_rotation`, and the pyramid+restart loop in Eigen, matching the Python output bit-exactly on the same fixtures |
| 2026-05-09 | TRN cycle 3: real LRO WAC mosaic + LOLA LDEM bench | New `scripts/lro_trn_demo.py` reuses the cycle-2 `render_rover_view` + `recover_pose_pnp`. (a) Ortho fetched as 256×256 grayscale JPEG WAC-mosaic tiles from NASA Trek WMTS (`https://trek.nasa.gov/tiles/Moon/EQ/LRO_WAC_Mosaic_Global_303ppd_v02/...`) and stitched. Default 3×3 tiles at zoom 5 → 768×768 ortho at ~660 m/px (~500 km × 500 km coverage), ~600 KB total download. (b) Heightmap fetched as `LDEM_<ppd>.img` from PDS Geosciences (`https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/...`), parsed as PDS3 raw int16 with HEIGHT_M = DN × 0.5. Default LDEM_4 (~2 MB, 7.6 km/px) for 2-second download; flag `--ldem-ppd 64` available for 470 m/px (530 MB). DEM cropped to WAC bbox and bilinear-resampled to ortho grid; mean-subtracted to give relative-height heightmap. (c) Rover camera placed at known (X, Y, Z); same forward-render + AP3P PnP pipeline as cycle 2. Plausibility gate uses ortho extent. | **6-target sweep at 400 km altitude / z=5 ortho**: apollo11 79 matches / 24 inliers / **1383 m err**; apollo12 37 / 16 / 300 m; apollo15 107 / 20 / 574 m; apollo17 89 / 16 / 622 m; **tycho 113 / 24 / 179 m (best)**; copernicus 58 / 17 / 391 m. All 6 succeed, **0 false positives**. Empirical altitude sensitivity (Tycho z=5): 35 km → matches=52 PnP fail; 70 km → 77 / fail; 200 km → 86 / 12 inliers / 98 m err; 400 km → 113 / 24 / 179 m err. Below ~200 km altitude the rover-view sample distance becomes >3x finer than the WAC ortho and SIFT scale-space stops bridging. Total demo run ~5 s after first-time download cache is populated. | TRN third-axis is now validated on **real lunar imagery + LOLA elevation**, not just synthetic craters. Honest envelope: orbital-descent regime (200-400 km altitude) at WAC z=5 sample scale. To bring this down to ~10 km terminal-descent altitude requires WAC z=7 (~165 m/px, 16x download) and LDEM_64 (~470 m/px, 530 MB). Mare targets (Apollo 11/12) have ~10x worse position error than crater rim targets (Tycho, Copernicus) because mare SIFT features are dim and self-similar. Output at `docs/figures/trn_lro_{apollo11,tycho}/{ortho,rover,matches}.png + summary.json`. |
| 2026-05-09 | TRN scaffold cycle 2: heightmap forward render + rotation fix + solvePnPRansac | `scripts/synthetic_trn_demo.py` rewritten end-to-end. (a) `synth_terrain` returns BOTH `heightmap_m` and `intensity` driven by the same crater Gaussians. (b) `render_rover_view` ray-marches each rover pixel through 6 fixed-point iterations against the heightmap (`s_{k+1} = (h(P_xy(s_k)) - cam_z) / d_z`), replacing the cycle-1 `cv2.warpPerspective` flat-Z=0 approximation. (c) `world_camera_rotation(yaw, pitch, roll)` applies a base R0 ("look +Y/north, level") plus extrinsic yaw and intrinsic pitch+roll, so `--pitch-deg -90` is now true nadir (verified visually). (d) `recover_pose_pnp` builds (3D world with heightmap Z, 2D rover pixel) correspondences and calls `cv2.solvePnPRansac(flags=SOLVEPNP_AP3P)` + `solvePnPRefineLM`. Two extra fixes were needed before defaults converged: dedup match list by rover keypoint to kill the "57 inliers all at one rover pixel → 1e16 m off" failure mode; plausibility gate that rejects estimates outside [ortho_extent ± 50%] in X/Y or below max heightmap rim in Z. | Operating envelope (10-seed sweep at top-down altitude 100 m, 800 m × 800 m ortho): **8/10 seeds → position error 0.57-2.88 m**, 2/10 cleanly REJECTED by the plausibility gate, **0 false positives**. Default fixture (seed=7): matches=91, inliers=21, median reproj 1.11 px, truth (200, 200, 100), estimated (197.42, 199.99, 98.72), error 2.881 m. Pitch sweep at altitude 80 m yaw=0 roll=0: pitch ∈ {-90, -85, -80, -75} → errors {0.91, 0.54, 1.36, 0.99} m; pitch -70 / -60 / -45 → REJECTED. PnP solver ablation: SQPNP raises `point_coordinate_variance < threshold` on near-coplanar 3D points (heightmap relief ~5 m vs camera altitude ~100 m); EPNP and ITERATIVE return success on outlier-heavy inlier sets but converge to numerically degenerate poses (positions of order 1e7-1e16 m off truth); only AP3P stays stable. | TRN third-axis is now functional, not just a scaffold. Honest envelope is near-nadir (tilt ≤ 20 deg from straight-down) on the synthetic crater terrain. Tilted views need either a multi-sample anti-aliased renderer (current INTER_LINEAR ray-march smears foreshortened terrain), a denser/crater-specific feature detector (SIFT inlier ratio drops to ~5% at high tilt), or a constrained-rotation prior. Real LRO ortho/DEM tile bench is the next concrete validation. Output at `docs/figures/trn_synthetic/{ortho,rover,matches}.png + summary.json` |

## Next POLAR Traverse Run

1. Download `polar-traverse-view1`.
2. Run `scripts/prepare_polar_traverse.py` for camera `L`, exposure `50`.
3. Execute generated `outputs/polar_view1_left_50ms/run_orb.sh`.
4. Repeat with `--feature sift` or the benchmark runner.
5. Record match count, inlier count, failed frames, runtime, and trajectory plot.

## Stereo Scale Notes

The initial stereo PnP baseline uses ORB left-right matches, simple row/disparity filtering, linear
triangulation, and PnP RANSAC. Raw stereo recovers meter-scale motion but overestimates path length by
about 7.4%. Rectifying POLAR stereo pairs reduces this to about 2.0% on View1 Traverse1 50 ms.

Next experiments:

1. Use full stereo extrinsics directly in the C++ CLI instead of relying on prepared rectified images.
2. Compare ORB stereo PnP with SIFT stereo PnP.
3. Reject unstable far-depth triangulations before PnP.
4. Add runtime and memory metrics for rectification plus VO.

## Multi-Traverse Benchmark

`benchmarks/run_polar_traverse_suite.py` prepares POLAR Traverse sequences, runs monocular ORB/SIFT,
runs rectified stereo PnP, evaluates against refined poses, and writes `summary.csv` plus `summary.md`.

Current POLAR View1 50 ms suite summary:

| Method | Alignment | Frames OK | Failed motions | ATE RMSE median | ATE RMSE mean |
| --- | --- | ---: | ---: | ---: | ---: |
| ORB essential | Sim(3) | 35/66 | 31 | 1.631 m | 1.507 m |
| SIFT essential | Sim(3) | 41/66 | 25 | 1.092 m | 1.283 m |
| ORB rectified stereo PnP | SE(3) | 43/66 | 23 | 1.613 m | 1.451 m |

Takeaway: single-traverse metrics were too optimistic. Traverse1-3 are mostly tractable, while Traverse4-6
break the current feature-only baselines. Next robustness experiments should vary exposure, add image
normalization, and tune feature matching for low-texture/high-shadow frames before adding heavier SLAM.

## Image Normalization: CLAHE

CLAHE preprocessing is available in `lunar_visual_odometry` and `stereo_visual_odometry` via `--clahe`.
It is intentionally an option, not the default, because it can help shadowed traverses but may also alter
feature repeatability.

POLAR View1 Traverse4-6 at 50 ms:

| Method | Frames OK baseline | Frames OK CLAHE | ATE mean baseline | ATE mean CLAHE | Decision |
| --- | ---: | ---: | ---: | ---: | --- |
| ORB essential | 11/33 | 11/33 | 2.195 m | 2.180 m | Little effect |
| SIFT essential | 8/33 | 11/33 | 2.549 m | 1.831 m | Useful for difficult lighting |
| ORB rectified stereo PnP | 12/33 | 15/33 | 2.557 m | 2.155 m | Useful but still not robust enough |

Next step: run an exposure sweep before adding more feature heuristics. Traverse6 remains poor even with
CLAHE, suggesting the 50 ms exposure is not sufficient for that condition.

### SIFT Stereo PnP (2026-05-09)

The stereo VO app was hard-coded to ORB descriptors. Adding `--feature sift` to
`apps/stereo_visual_odometry` (mirroring the mono path's selector) and re-running the
Traverse4-6 50 ms CLAHE suite gives a step-function improvement:

| Sequence | ORB stereo PnP | SIFT stereo PnP | ATE RMSE ORB → SIFT |
| --- | ---: | ---: | ---: |
| View1_Traverse4_L_50ms | 7/11 | **10/11** | 1.55 → **0.37 m** |
| View1_Traverse5_L_50ms | 6/11 | **9/11** | 1.89 → **0.73 m** |
| View1_Traverse6_L_50ms | 2/11 | **6/11** | 3.03 → **1.60 m** |
| **Total** | **15/33** | **25/33** | mean 2.16 → **0.90 m** |

Result directory: `outputs/polar_view1_suite_clahe_t4_t6_sift/`. Command:

```bash
python3 benchmarks/run_polar_traverse_suite.py \
  --output-dir outputs/polar_view1_suite_clahe_t4_t6_sift \
  --traverses Traverse4 Traverse5 Traverse6 \
  --exposure-ms 50 --skip-mono --stereo-features orb sift --clahe
```

Takeaway: SIFT's L2-norm descriptors recover Traverse6 enough to register 6/11 frames where ORB
collapses to 2/11. The robustness story for POLAR is now SIFT (not ORB) on the stereo branch when
illumination is poor; ORB stays as the speed baseline. Traverse6 still degrades vs Traverse4/5
even with SIFT, so further work either widens exposure or replaces RANSAC PnP with a robust
landmark tracker.

### Loosened Lowe Ratio Test (2026-05-09)

Inspecting the T6 SIFT log showed temporal-match counts dropping to 16-22 even on frames that
had 200+ stereo matches. The default ratio test (`0.75`) is too restrictive for low-texture
shadowed terrain. Exposing `--ratio-test` on `apps/stereo_visual_odometry` and rerunning at 0.85
gives a step-function improvement on top of the SIFT-stereo win:

| Sequence | SIFT r=0.75 | SIFT r=0.85 | ATE RMSE r=0.75 → r=0.85 |
| --- | ---: | ---: | ---: |
| View1_Traverse4_L_50ms | 10/11 | **11/11** | 0.37 → **0.069 m** |
| View1_Traverse5_L_50ms | 9/11 | **11/11** | 0.73 → **0.080 m** |
| View1_Traverse6_L_50ms | 6/11 | **10/11** | 1.60 → **0.413 m** |
| **Total T4-T6** | **25/33** | **32/33** | mean 0.90 → **0.187 m** |

Verified that easier traverses do not regress at the looser ratio:

| Sequence | SIFT r=0.85 |
| --- | ---: |
| View1_Traverse1_L_50ms | 11/11 ATE 0.028 m |
| View1_Traverse2_L_50ms | 11/11 ATE 0.037 m |
| View1_Traverse3_L_50ms | 11/11 ATE 0.043 m |

Combined: **65/66 frames OK across all six POLAR Traverses** with mean ATE 0.118 m. Result
directories: `outputs/polar_view1_suite_clahe_t4_t6_sift_r085/` and
`outputs/polar_view1_suite_clahe_t1_t3_sift_r085/`.

Operational recommendation: SIFT + CLAHE + `--ratio-test 0.85` for shadowed lunar terrain;
the default 0.75 is fine for cleaner illumination. Default unchanged so that prior runs stay
reproducible.

### Sky-Cell Phase 2: Post-Merge Cell-Compactness Filter (2026-05-09)

`scripts/build_star_pair_index.py` now stores per-star `sky_cell_ids` (4 lat × 12 lon = 48
equal-area-by-`sin(dec)` cells) in the `.npz` index. `identify_stars_with_pair_index.py
--cell-compactness-deg <deg>` opt-in: after the 3-way edge-tolerance merge produces a list of
catalog triangles `(a, b, c)`, drop triangles whose pairwise sky-cell-center angles exceed
the threshold. Backward-compatible: indices without the field auto-compute cell ids on the
fly so existing `.npz` files keep working.

16k mag≤8 fixture (camera fx=1000 → diagonal half-FOV ≈ 36°):

| --cell-compactness-deg | candidate_hypotheses | cand_gen [s] | RMS [arcsec] | csv vs baseline |
| ---: | ---: | ---: | ---: | --- |
| none (baseline) | 13842 | 2.84 | 26.13 | — |
| 90 | 13842 | 2.71 | 26.13 | identical |
| 75 | 13236 (-4%) | 2.80 | 26.13 | identical |
| 60 | 11542 (-17%) | 2.70 | 26.13 | identical |
| 50 | 8740 (-37%) | 2.73 | 26.13 | identical |
| 40 | 4875 (-65%) | 2.67 | 29.00 | identical |

60k mag≤9 fixture (`outputs/hyg_pair_index_false_scaling_60000_mag9_ps6_smoke/`),
compactness=50: candidate_hypotheses **701199 → 437357 (-38%)**, cand_gen 251.05s (baseline
range 256–270s — within noise), verify 0.412s (baseline 0.451s), 32/32 correct.

**Honest finding**: the post-merge filter is correctness-preserving and cleanly reduces
the *count* of candidate hypotheses, but **does not move the cand_gen wall-time** on either
fixture. The merge itself (pandas 3-way join across 500k-pair lists at 60k mag≤9) is the
bottleneck, and an output-side filter cannot help. The 60k mag≤9 ~250 s `cand_gen` headline
needs the *index itself* re-keyed by `(edge_bin, cell_pair)` so each pair-list lookup loads
only pairs in the relevant cell pair. That is a multi-day refactor (new `.npz` schema +
`load_candidate_pairs` rewrite + C++ reader update) and remains the open phase 3 of this
work. For now, `--cell-compactness-deg` is useful as a hypothesis-count knob (downstream
verify cost scales linearly with the triangle count once `--max-verified-hypotheses` is
unbounded), not as a query-latency lever.

### TRN (Terrain Relative Navigation) Scaffold (2026-05-09)

`scripts/synthetic_trn_demo.py` lands as the project's third localization axis (after star
tracker attitude and lunar VO). The scaffold is a single Python script doing the full
synthetic pipeline:

1. Procedural ortho image: 20 large + 150 medium + 800 small Gaussian crater bowl/rim pairs
   plus low-amplitude pebble noise → ~2000 SIFT keypoints in a 800×800 px image.
2. Rover view: `cv2.warpPerspective` of the ortho onto the rover image plane under a
   flat-plane (Z=0) approximation given (`X, Y, altitude, yaw, pitch, roll`) and intrinsics.
3. SIFT + Lowe-ratio BFMatcher between ortho and rover.
4. `cv2.findHomography(RANSAC)` for the ortho→rover homography.
5. Decompose the homography to recover (X, Y) in ortho world coords; compare to truth.

Output goes to `docs/figures/trn_synthetic/{ortho,rover,matches}.png` plus a `summary.json`
with the recovered homography and position error.

Status at `--pitch-deg -15 --rover-altitude-m 50 --ortho-size-px 800
--rover-x-m 200 --rover-y-m 150`:

- Match count: **99** (Lowe ratio 0.75)
- RANSAC inliers: **69**
- Estimated XY position error: **~250 m** vs truth (200, 150) — too large

The matching half of the pipeline works (69 RANSAC inliers is a plausible inlier set), but
the homography decomposition produces a degenerate position because (a) the rotation
convention in `project_rover_view` interprets `pitch=-15°` differently from what the
recovery step assumes, and (b) the bilinear `warpPerspective` blurs out most fine
features in the foreshortened far-field (rover SIFT keypoints drop to ~10), so the
RANSAC inliers cluster in a small region of the image and underconstrain the homography.

Open follow-ups (next TRN cycle, all incremental):

- Forward-render the rover view by sampling a heightmap per rover pixel instead of warping
  the ortho — fixes feature suppression in the foreshortened far-field.
- Fix the (yaw, pitch, roll) → R_world_camera construction so `--pitch-deg -90` produces
  a true top-down view.
- Switch the recovery step from homography decomposition to `cv2.solvePnPRansac` over
  (ortho 3D world point, rover 2D pixel) once the heightmap renderer is in place.
- Bench against a real LRO ortho/DEM tile once the synthetic path returns sub-metre error.

The CLI shape (`--ortho-size-px / --rover-fx/fy/cx/cy / --rover-x-m/y-m /
--rover-altitude-m / --yaw/pitch/roll / --ortho-px-to-m`) is stable so the renderer and
recovery internals can be swapped without breaking the I/O contract.

### Realism Axis 7: Annual Stellar Aberration (2026-05-09)

`scripts/generate_star_tracker_observations_from_catalog.py --annual-aberration-deg <deg>`
applies first-order stellar aberration for an Earth orbital phase. Earth orbital speed
≈ 29.78 km/s, c = 299792.458 km/s → β ≈ 9.94e-5. Each catalog unit vector `u` becomes
`normalize(u + β * v_hat)` where `v_hat` is the Earth velocity direction in the equatorial
frame (ecliptic obliquity dropped — 23.4° tilt → sub-arcsec error at this scale, well under
the centroid noise floor). Maximum apparent shift ≈ β rad ≈ 20.5 arcsec at 90° to the
velocity vector. truth.json now records `annual_aberration_deg` and
`apply_proper_motion_years` for reproducibility.

16k mag&le;8 fixture, default identifier params, sweep across orbital phases:

| Aberration phase | assigned | RMS [arcsec] | shift vs no-aberration |
| --- | ---: | ---: | ---: |
| none | 32/32 | 18.60 | — |
| 0° | 32/32 | 18.93 | +0.33 |
| 90° | 32/32 | 19.12 | +0.52 |
| 180° | 32/32 | 18.62 | +0.02 |
| 270° | 32/32 | 18.52 | -0.08 |

Identifier absorbs the shift cleanly because the verification tolerance (600 arcsec) is
well above the 20-arcsec maximum aberration. The RMS modulation across phases (~0.5 arcsec
amplitude) reflects the angle between the camera optical axis and the Earth velocity
vector. Realism axis count is now **7**: limiting magnitude, magnitude-dependent centroid
noise, near-real-star false positives, edge-biased false positives, observation-side
magnitude prior, hot-pixel intensity clustering, **annual stellar aberration**.

### Realism Axis 6: Magnitude-Aware False-Positive Intensity (2026-05-09)

`scripts/drop_star_ids.py` gains `--false-mag-hot-mean` / `--false-mag-hot-std`. Hot-pixel
false detections (those generated under `--hot-pixel-fraction`) draw their magnitude from a
*separate* Gaussian, modelling the physical reality that read-noise spikes saturate the
sensor and look much brighter than ambient noise. Default `None` reuses
`--false-mag-mean/std` for all branches (back-compat).

16k mag&le;8 fixture, `pyramid_size=6 --pyramid-restarts 3 --hot-pixel-fraction 0.5
--false-count 12`:

| Hot-pixel mag dist | assigned | RMS [arcsec] |
| --- | ---: | ---: |
| default (Gaussian 5.0±1.0 for everything) | 32/32 | 18.65 |
| `--false-mag-hot-mean 2.0 --false-mag-hot-std 0.3` (saturated/bright) | 32/32 | 18.65 |
| `--false-mag-hot-mean 4.5 --false-mag-hot-std 0.3` (star-like, hardest) | 32/32 | 18.65 |

The default-vs-bright assignments are byte-identical, and the star-like variant (where the
hot-pixel "stars" sit smack in the middle of the real catalog mag distribution) is also
recovered cleanly. The identifier's magnitude-aware verification (`|obs_mag - cat_mag|`)
plus pyramid restart absorbs all three distributions without correctness loss. Realism axis
count is now **6**: limiting magnitude, magnitude-dependent centroid noise, near-real-star
false positives, edge-biased false positives, observation-side magnitude prior,
**hot-pixel intensity clustering**.

### Sky-Cell Phase 1 C++ Port (2026-05-09)

`apps/lost_in_space_pair_id --fov-radius-deg <deg>` mirrors the Python flag in
`pair_id_solver.cpp::verify_rotation`. JSON metadata gains `fov_radius_deg`. Tested on a
2000-star fixture (`/tmp/cpp_fov_bench/idx2k.bin`) against the matching Python `.npz`:

| Config | assigned | verified | RMS [arcsec] | Verify [s] | CSV |
| --- | ---: | ---: | ---: | ---: | --- |
| Python baseline | 32 | 34 | 26.12526431 | 0.0201 | — |
| Python FOV=40 | 32 | 34 | 26.12526431 | 0.0161 (-20%) | — |
| C++ baseline | 32 | 34 | 26.12526396 | 0.00317 | == Python baseline |
| C++ FOV=40 | 32 | 34 | 26.12526396 | 0.00093 (**-71%**) | == C++ baseline |

CSV is byte-identical across Python baseline, C++ baseline, and C++ FOV=40. RMS matches to
~1e-6 (existing Eigen-JacobiSVD vs LAPACK-gesdd numerical noise). Same magnitude of verify
speedup as the Python flag (Python ~50% on 16k mag&le;8, ~70% on 60k mag&le;9), so the C++
port keeps the verification pre-pruning lever available in the deployment binary.

### Sky-Cell Phase 2: Post-Merge Cell-Compactness Filter (2026-05-09)

`identify_stars_with_pair_index.py --fov-radius-deg <deg>` adds an opt-in pre-prune to
`verify_rotation`: when the flag is set, only catalog stars whose inertial unit vector lies
within the cone of half-angle `--fov-radius-deg` of the optical axis (`R^T @ [0,0,1]`)
participate in the per-hypothesis verification. Default `None` keeps the legacy whole-catalog
verification path bit-exact.

For the 60000 mag≤9 fixture (`outputs/hyg_pair_index_false_scaling_60000_mag9_ps6_smoke/`,
camera fx=1000 → diagonal half-FOV ≈ 36°), three configs at `--max-verified-hypotheses 400`:

| FOV [deg] | Verified | RMS [arcsec] | Verify [s] | Cand gen [s] |
| ---: | ---: | ---: | ---: | ---: |
| none (baseline) | 159 | 26.13 | 0.451 | 256.26 |
| 40 | 159 | 26.13 | **0.127** | 245.60 |
| 38 | 159 | 26.13 | **0.128** | 245.30 |

Verify time drops **-72%** at FOV 40°. RMS, verified-hypothesis count, and the assignments
CSV are byte-identical to the no-FOV baseline. Cand-gen variation is single-trial noise.

The 16000 mag≤8 fixture confirms identical behaviour (verify 0.144 → 0.070 s, -51%) and the
same byte-exact CSV.

Headline impact on the 60k mag≤9 query: 0.32s saved out of ~250s. Verify is not the dominant
cost at this catalog density (cand_gen is). The win matters more for:

- narrower-FOV cameras (5-15° half-FOV → 95-99% catalog reduction → near-100% verify cut),
- denser catalogs where verify time grows linearly with N_stars, and
- verify-bound configurations (`--max-verified-hypotheses` tens of thousands).

This change also lays infrastructure for the next step (cell-partitioning the *pair index*
itself by `(cell_a, cell_b)`, which would prune candidate generation rather than verification
— the bigger lever at 60k mag≤9).

### T6 Frame 3 Investigation (2026-05-09)

The single remaining failure (T6 frame 3) was investigated. Mean luminance drops 166 → 118
between frame 2 and frame 3 (~30% darkening), and SIFT only finds 4 temporal matches under
ratio test 0.85. Tried:

- `--ratio-test 0.90` — frame 3 still 4 matches, but other T6 frames now collapse to "pnp failed"
  (more matches, but mostly outliers).
- `--clahe-clip-limit 4.0` — no improvement on frame 3, frame 6 starts failing.
- `--min-pnp-points 4 --min-pnp-inliers 4` — RANSAC PnP itself fails on 4 noisy matches.

Frame 3 is unrescuable in the current frame-to-frame VO architecture. Recovering it would
require a multi-frame map (bundle adjustment over a sliding window of 3+ frames), dense
optical flow, or simply a longer exposure on that frame.

Added a defensive `last_good_frame_` fallback to `StereoVisualOdometry`: on motion failure,
the next frame retries against the most recent successful frame if it has strictly more 3D
points than the failed `previous_frame_`. Did not trigger on this T6 sequence (the failure
pattern here is one isolated frame whose `previous` is itself the last good), but provides a
safety net for "previous + current both weak" patterns in other sequences. T1-T6 results
unchanged at 65/66 with mean ATE 0.118 m.

## Space-Specific Direction

The project should move beyond rover VO into space-native localization. Added first star tracker baseline:
identified star observations plus a star catalog are solved with Wahba/Kabsch to estimate inertial camera
attitude. This provides an absolute attitude source that Earth rover localization usually does not use.

Next experiments:

1. Generate synthetic star tracker observations with known attitude and pixel noise.
2. Add a public star catalog adapter.
3. Combine star tracker attitude with crater/TRN position constraints.

Completed initial synthetic benchmark with 30 identified stars:

| Pixel noise | Mean attitude error | Max attitude error |
| ---: | ---: | ---: |
| 0 px | 0.000015 deg | 0.000015 deg |
| 0.1 px | 0.00459 deg | 0.00728 deg |
| 0.5 px | 0.0229 deg | 0.0364 deg |
| 1.0 px | 0.0458 deg | 0.0729 deg |

Public catalog adapter status:

- HYG v4.2 download works through `scripts/download_star_catalog.py`.
- `mag <= 6.5` conversion produced 8,920 unit-vector stars.
- Catalog-backed synthetic observations from 20 visible HYG stars ran successfully through the C++ star
  tracker with RMS bearing error around `2.34e-4 rad` for `0.2 px` noise in the smoke case.

Lost-in-space prototype status:

| Pixel noise | Correct IDs | Assigned IDs | Notes |
| ---: | ---: | ---: | --- |
| 0 px | 36/36 | 36/36 | synthetic local candidate catalog |
| 0.05 px | 36/36 | 36/36 | synthetic local candidate catalog |
| 0.1 px | 36/36 | 36/36 | synthetic local candidate catalog |
| 0.2 px | 36/36 | 36/36 | synthetic local candidate catalog |

Decision: keep the triangle matcher as a prototype. It validates angular pattern matching, but full-sky use
needs an indexed catalog search instead of brute force over a small candidate set.

Triangle index prototype:

| Catalog | Indexed stars | Indexed triangles | Test observations | Correct IDs |
| --- | ---: | ---: | ---: | ---: |
| HYG v4.2 brightest subset | 120 | 34,858 | 24 | 24/24 |
| HYG v4.2 brightest subset | 180 | 121,705 | 24 | 24/24 |
| HYG v4.2 brightest subset | 240 | 293,642 | 24 | 24/24 |
| HYG v4.2 brightest subset | 360 | 958,157 | 24 | 24/24 |
| HYG v4.2 brightest subset | 500 | 2,524,698 | 16 | 16/16 |

Decision: the indexed approach is viable for small public HYG subsets when triangle candidates are
verified by a Wahba/Kabsch attitude hypothesis and a weak magnitude prior. Larger-catalog partitioning
and faster index construction now matter more than the query step before a C++ port.

Pair-angle index prototype with 120 arcsec minimum catalog star separation:

| Catalog | Indexed stars | Indexed pairs | Index MB | Query avg | Correct IDs |
| --- | ---: | ---: | ---: | ---: | ---: |
| HYG v4.2 brightest resolved subset | 500 | 52,553 | 0.4 | 0.298 s | 24/24 |
| HYG v4.2 brightest resolved subset | 1000 | 208,563 | 1.6 | 0.392 s | 24/24 |
| HYG v4.2 brightest resolved subset | 2000 | 832,038 | 6.3 | 0.727 s | 24/24 |

Decision: pair-angle indexing is the preferred scalable Python prototype. It reduces build time and index
size enough to test 2000 stars. Candidate pruning plus vectorized attitude verification keeps query time
below one second in the current benchmark.

False/missing star robustness, 16 generated stars, 0.1 px noise:

| Dropped | False | Correct IDs | Assigned IDs |
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

Decision: robustness is promising in local-catalog synthetic conditions. Public-catalog ambiguity is now
measured with HYG brightest subsets; the next work is replacing full triangle enumeration with a more
scalable catalog index.

HYG pair-index robustness, 2000 indexed stars, 12 generated stars, 0.1 px noise, 2 trials per setting:

| Dropped | False | Correct true IDs | Assigned IDs | Wrong IDs | Query avg |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | 24/24 | 24/24 | 0 | 1.281 s |
| 0 | 2 | 24/24 | 24/24 | 0 | 2.207 s |
| 0 | 4 | 24/24 | 24/24 | 0 | 2.393 s |
| 2 | 0 | 20/20 | 20/20 | 0 | 0.943 s |
| 2 | 2 | 20/20 | 20/20 | 0 | 1.359 s |
| 2 | 4 | 20/20 | 20/20 | 0 | 2.299 s |
| 4 | 0 | 16/16 | 16/16 | 0 | 0.793 s |
| 4 | 2 | 16/16 | 16/16 | 0 | 1.240 s |
| 4 | 4 | 16/16 | 16/16 | 0 | 1.560 s |

Decision: pair-index robustness is good enough to move next to larger observed star counts and more
realistic false-star distributions.

HYG pair-index observation scaling, 2000 indexed stars, 0.1 px noise, 2 trials per setting:

| Observed stars | Correct IDs | Assigned IDs | Wrong IDs | Candidates avg | Verified avg | Query avg |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 16 | 32/32 | 32/32 | 0 | 7092.5 | 390.5 | 2.873 s |
| 24 | 48/48 | 48/48 | 0 | 6082.0 | 382.5 | 2.381 s |
| 32 | 64/64 | 64/64 | 0 | 7236.0 | 378.0 | 2.454 s |

Decision: observed-star scaling is stable through 32 stars. Next stress tests should combine larger
observed star counts with false detections.

HYG pair-index false detection scaling, 2000 indexed stars, 32 true stars, 0.1 px noise, 2 trials per
setting:

| False detections | Correct true IDs | Assigned IDs | Wrong IDs | Candidates avg | Verified avg | Query avg |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 64/64 | 0 | 6597.0 | 384.0 | 2.426 s |
| 4 | 64/64 | 64/64 | 0 | 6565.0 | 352.5 | 2.686 s |
| 8 | 64/64 | 64/64 | 0 | 6903.5 | 332.0 | 2.664 s |
| 12 | 64/64 | 64/64 | 0 | 7288.0 | 331.5 | 2.694 s |

Decision: false-star scaling is stable through 32 true stars plus 12 false detections. The uniform
observation-triangle sampler should remain the default.

HYG pair-index full false sweep at 4000 indexed stars, 32 true stars, 0.1 px noise, 2 trials per
setting (3,329,812 indexed pairs, 25.2 MB index, 51.591 s build, 120 arcsec minimum catalog star
separation). Per-stage timings from `identify_stars_with_pair_index.py`:

| False detections | Correct true IDs | Assigned IDs | Wrong IDs | Candidates avg | Verified avg | Query avg | Cand gen avg | Verify avg |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 64/64 | 64/64 | 0 | 49359.0 | 378.0 | 8.796 s | 7.413 s | 0.425 s |
| 4 | 64/64 | 64/64 | 0 | 51884.5 | 349.0 | 8.689 s | 7.346 s | 0.382 s |
| 8 | 64/64 | 64/64 | 0 | 52643.5 | 345.0 | 8.889 s | 7.419 s | 0.472 s |
| 12 | 64/64 | 64/64 | 0 | 56411.5 | 330.5 | 9.311 s | 7.932 s | 0.410 s |

Decision: correctness holds at 4000 indexed stars across the full false_counts 0/4/8/12 sweep with
zero wrong assignments. Per-stage timing shows candidate generation accounts for roughly 85% of query
time (~7.4-7.9 s) while vectorized verification is only ~0.4 s. The next engineering target is the
pair-list intersection / adjacency reuse inside `candidate_mappings`, not verification. Defer any
8000-star scaling attempt until candidate generation is reduced.

## 2026-05-09: C++ Identifier Port — Bit-Exact Validation On 500 / 2000 / 16000 Fixtures

`apps/lost_in_space_pair_id` now wraps `localization::identify_lost_in_space()`, the C++ port of
`scripts/identify_stars_with_pair_index.py`. We re-ran the Python reference and the C++ binary on
the same fixtures and compared the assignments CSVs byte-for-byte (`cmp -s`).

Pair indices were rebuilt with `scripts/build_star_pair_index.py --write-bin --skip-pkl` so each
fixture has both an `.npz` (Python loads it) and a `.bin` (C++ loads it). Same `--bin-arcsec 120`,
`--min-edge-deg 0.2`, `--max-edge-deg 80` parameters across all builds.

| Fixture | Indexed stars | Mode | Args extras | C++ vs Py |
| ---: | ---: | --- | --- | --- |
| `outputs/hyg_ambiguity_benchmark_500/index_500/trial_000` | 500 | default | `--tolerance-arcsec 300 --neighbor-bins 2` | byte-exact |
| `outputs/hyg_ambiguity_benchmark_500/index_500/trial_001` | 500 | default | (same) | byte-exact |
| `outputs/hyg_pair_index_benchmark_pruned/index_2000/trial_000` | 1000 (resolved) | default | (same) | byte-exact |
| `outputs/hyg_pair_index_benchmark_pruned/index_2000/trial_001` | 1000 (resolved) | default | (same) | byte-exact |
| (same trial_000 obs) | 1000 | pyramid | `--tolerance-arcsec 120 --neighbor-bins 1 --pyramid-size 6` | byte-exact |
| 16-star synthetic obs vs HYG mag≤6.5 (catalog-saturated to 8920) | 8920 | pyramid | `--pyramid-size 6 --tolerance-arcsec 120 --neighbor-bins 1` | byte-exact |

Metadata fields that depend on Wahba/Kabsch (`best_rms_error_arcsec`, `best_mean_score_arcsec`)
differ between Python (`numpy.linalg.svd` → LAPACK gesdd) and C++ (`Eigen::JacobiSVD`) at the
~1e-6 arcsec level — far below the 600 arcsec verification tolerance and not enough to flip any
greedy assignment. Counts (`assigned_observations`, `triangle_matches`,
`observation_triangles_evaluated`, `candidate_hypotheses`, `verified_hypotheses`,
`attempts_taken`, `winning_attempt_index`) match exactly.

Decision: the C++ identifier is now the deployment path. The Python script remains the reference
implementation and the bit-exact diff is the regression contract — see PLAN.md item 7 for the
proposed CI smoke that runs both on every push.

## 2026-05-09: Optical Distortion Realism Axis (Brown-Conrady k1 Sweep)

`scripts/generate_star_tracker_observations_from_catalog.py` gained four flags
(`--distortion-k1`, `--distortion-k2`, `--distortion-p1`, `--distortion-p2`) that apply forward
Brown-Conrady distortion at projection. The lost-in-space identifier consumes no calibration
information, so the synthesized pixels go in raw — this measures how robust the pair-angle
matcher is to an *uncalibrated* lens.

Sweep configuration: HYG mag&le;6.5 catalog (8920 stars, full mag6.5 file used as the index
source), 16 observations per trial, `--noise-px 0.1`, `--seed 16000`, `--yaw-deg 30 --pitch-deg
15 --roll-deg 5`. Identifier: C++ `apps/lost_in_space_pair_id` with the bit-exact-against-Python
default config (`--tolerance-arcsec 120 --neighbor-bins 1 --verification-tolerance-arcsec 600
--magnitude-prior-arcsec 15 --pyramid-size 6`).

| k1 | Correct | Wrong | Assigned | best_rms_error_arcsec |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 15/16 | 1 | 16 | 27.68 |
| -0.02 | 12/16 | 2 | 14 | 222.49 |
| -0.05 | 0/16 | 5 | 5 | 186.73 |
| -0.10 | 0/16 | 6 | 6 | 264.42 |
| -0.20 | 0/16 | 5 | 5 | 116.15 |
| -0.30 | 0/16 | 6 | 6 | 118.50 |

Order-of-magnitude calibration: at the image corner (u=1024, fx=1000), normalized radius² is
0.524, so k1=-0.05 displaces a star ~13 pixels inward (~2700 arcsec). The default 300 arcsec
edge-tolerance is overwhelmed at any k1 &le; -0.05.

Decision: the pair-angle identifier is **not** distortion-tolerant on its own — uncalibrated
distortion at k1 = -0.02 already drops correctness from 94% to 75%, and k1 = -0.05 collapses it
to zero. For real-camera deployment the consumer must undistort the observed pixels (using a
calibrated `cv::undistortPoints` or equivalent) before handing them to
`identify_lost_in_space()`. Tolerance widening is not a real fix — at the corner the residual
already approaches the verification-tolerance ceiling.

Open follow-up: add a `--distortion-coefficients` JSON input to the C++ CLI that runs
`undistortPoints` over the loaded `(u, v)` pixels before normalisation. This keeps the
identifier algorithm unchanged and folds the calibration knowledge into the front door.

## 2026-05-09: Undistortion Front-Door Restores Correctness Under Distortion

Both `scripts/identify_stars_with_pair_index.py` and `apps/lost_in_space_pair_id` gained
`--distortion-k1/k2/p1/p2` flags that iteratively invert Brown-Conrady at the (u, v) →
bearing step using a fixed-point iteration (8 iterations; converges within float precision
for |k1| ≤ 0.5). This re-runs the previous sweep with the identifier compensating for the
generator-applied distortion.

| k1 (gen = id) | Py correct | Py wrong | C++ correct | C++ wrong | Bit-exact |
| ---: | ---: | ---: | ---: | ---: | --- |
| 0 | 15/16 | 1 | 15/16 | 1 | byte-exact |
| -0.02 | 15/16 | 1 | 15/16 | 1 | byte-exact |
| -0.05 | 15/16 | 1 | 15/16 | 1 | byte-exact |
| -0.10 | 15/16 | 1 | 15/16 | 1 | byte-exact |
| -0.20 | 15/16 | 1 | 15/16 | 1 | byte-exact |
| -0.30 | 15/16 | 1 | 15/16 | 1 | byte-exact |

The single wrong assignment is the same fixture-specific catalog ambiguity that already
appears at k1=0 in the previous (uncalibrated) sweep — it is not introduced by the
undistortion path. The 500-star no-distortion fixture remains byte-exact between the C++
and Python outputs after the changes (regression-checked).

Decision: the C++ identifier now consumes calibration knowledge at the front door (intrinsics
+ distortion). For real-camera deployment the consumer simply provides the calibration JSON
and the algorithm stays untouched. Open follow-up: accept a single `--calibration-json` path
that bundles fx/fy/cx/cy + distortion (matches the `truth.json` schema written by the
observation generator), so wiring downstream tooling becomes a one-liner.

## 2026-05-09: C++ Pair-Index Builder — Bit-Exact + Wall-Time Sweep

`apps/build_star_pair_index` is a C++ port of `scripts/build_star_pair_index.py --write-bin`.
We measured wall time + peak RSS on the same catalogs and `cmp`'d the resulting `.bin` files.

Catalogs:

- 500: `outputs/hyg_ambiguity_benchmark_500/hyg_brightest_500.csv`
- 4000 / 16000: `datasets/star_catalogs/hyg-v42/converted/hyg_v42_bright_mag6p5_unit.csv`

Both runs use `--bin-arcsec 120 --min-edge-deg 0.2 --max-edge-deg 80`.

| Limit | Pairs | Python wall | Python RSS | C++ wall | C++ RSS | `.bin` cmp |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 500 | 52,549 | (negligible) | — | (negligible) | — | byte-exact |
| 4000 | 3,329,812 | 4.33 s | 265 MB | 0.73 s | 130 MB | byte-exact |
| 16000 (sat. 8920) | 16,547,574 | 12.51 s | 748 MB | 2.76 s | 393 MB | byte-exact |

Decision: the C++ builder is the deployment-path index source going forward. The Python
builder stays as the reference / `.npz` source for benchmarks that consume the NumPy index
directly. With the C++ identifier already in place, the full deployment pipeline (catalog
CSV → `.bin` → identifier → assignments CSV) now runs without a Python dependency.

Open follow-ups for very large catalogs:

- mag≤9 80k+ build still allocates the full pair list. Memory peaks ~50 GB extrapolated;
  add a streaming / two-pass path before attempting that scale.
- The Python builder still writes the `.npz` (used by benchmarks) and the optional `.pkl`.
  Whether to add `.npz` writing to the C++ builder is gated on whether the npz format
  becomes a deployment-path requirement (currently no — deployment uses `.bin` only).

## 2026-05-09: C++ Pair-Index Builder — 2-Pass Bucket Fill (Memory + Wall Wins)

`apps/build_star_pair_index` was refactored from a single-pass (collect-then-sort) layout to
a 2-pass bucket fill. Pass 1 walks every (i, j>i) pair and counts pairs per bin; that gives
us `bin_keys` + `bin_offsets` directly. Pass 2 re-walks the same order and writes each pair
into its slot via a per-bin write cursor. Within any bin, pairs land in visitation order
(i ascending, j ascending), which is exactly what `np.argsort(kind="stable")` produced on
the prior sort path — so the `.bin` output is unchanged.

| Limit | Pairs | Sort-path wall | Sort-path RSS | 2-pass wall | 2-pass RSS | `.bin` cmp |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 500 | 52,549 | (negl.) | — | 0.00 s | 5 MB | byte-exact |
| 4000 | 3,329,812 | 0.73 s | 130 MB | 0.49 s | 46 MB | byte-exact |
| 16000 (sat. 8920) | 16,547,574 | 2.76 s | 393 MB | 2.07 s | 134 MB | byte-exact |

Compared to the Python `--write-bin` reference at 16000 stars: 12.51 s / 748 MB → 2.07 s /
134 MB, a ~6× wall and ~5.5× memory reduction. The previous C++ sort-path numbers are
preserved in the prior table for reference.

Decision: 2-pass is now the only build path. The intermediate `(lhs, rhs, bin)` int32
buffers and the index-permutation `stable_sort` are gone, which removes the dominant
memory term for very large catalogs. Extrapolation: at 80k stars (mag≤9), pair count is
~3.2 G, so peak `pair_endpoints` is ~25 GB — still big, but ~3× smaller than the prior
sort-path peak and closer to fitting on a 32-64 GB workstation. Disk-streaming external
sort remains available as a future fix if 80k+ exceeds the in-memory ceiling.

## 2026-05-09: `--calibration-json` Umbrella Flag For Both Identifiers

Both `scripts/identify_stars_with_pair_index.py` and `apps/lost_in_space_pair_id` now
accept `--calibration-json <path>` that fills in any unset
`--fx/--fy/--cx/--cy/--distortion-k1/k2/p1/p2` from a JSON file. Schema matches the
`truth.json` written by `scripts/generate_star_tracker_observations_from_catalog.py`:

```json
{
  "intrinsics": {"fx": 1000.0, "fy": 1000.0, "cx": 512.0, "cy": 512.0},
  "distortion": {"k1": -0.1, "k2": 0.0, "p1": 0.0, "p2": 0.0}
}
```

The C++ identifier uses a single regex per key (no JSON dependency added) — the
generator's schema has each key in a unique location so a substring search is sufficient.

Verification on the k1=-0.1 distortion fixture (16-star × 16000 pair index):

| Path | Py output | C++ output | C++ vs Py |
| --- | --- | --- | --- |
| Individual flags | `py_undistort.csv` | `cpp_undistort.csv` | byte-exact (existing) |
| `--calibration-json truth.json` | `py_calib_json.csv` | `cpp_calib_json.csv` | byte-exact |
| Explicit flag override | `py_override.csv` (`--distortion-k1 0`) | `cpp_override.csv` (`--distortion-k1 0`) | byte-exact, matches uncalibrated baseline |

All three paths between Py and C++ are byte-exact, and `--calibration-json` is byte-exact
against the equivalent individual-flag invocation. Override semantics work in both
directions: passing `--distortion-k1 0` alongside the JSON correctly disables
undistortion, dropping the assigned observation count back to the broken-uncalibrated
baseline (6/16) — the same result as not using either flag set.

Decision: `--calibration-json` is now the recommended way to invoke either identifier on
data accompanied by a `truth.json` (or the production equivalent). The 500-star
no-calibration regression test still passes byte-exact, so the change is fully
backward-compatible.

## 2026-05-09: Edge-Biased False Detections Realism Axis

`scripts/drop_star_ids.py` gained `--false-edge-fraction` and `--false-edge-band-px`
flags. The edge-biased mode samples a side (top/bottom/left/right) uniformly, then
distance-from-edge ∈ [0, band] and along-edge ∈ [0, side_length]. Models lens-flare /
sensor-edge artifacts that cluster around the image perimeter — a more realistic
distribution than the existing uniform-random and near-real-star modes.

The four false-detection modes (`hot`, `edge`, `near`, `uniform`) now form a cumulative
partition: each fraction consumes its share of the *remaining* probability mass after
the higher-priority modes, so they never overlap.

### Sweep — 16 true stars + N false detections, single attempt (no restart)

Source observations: `/tmp/pair_id_compare/distortion_sweep/k1_0/observations.csv` (16
true stars, no distortion). Identifier: C++ `--pyramid-size 6` against the 16000-star
HYG mag≤6.5 index. Edge band = 24 px (1024×1024 image).

| False mode | False count | Correct | Wrong | Assigned-on-false | Assigned total |
| --- | ---: | ---: | ---: | ---: | ---: |
| uniform | 0 | 15/16 | 1 | 0 | 16 |
| edge | 0 | 15/16 | 1 | 0 | 16 |
| uniform | 4 | 15/16 | 1 | 0 | 16 |
| edge | 4 | 15/16 | 1 | 0 | 16 |
| uniform | 8 | 15/16 | 1 | 0 | 16 |
| edge | 8 | 15/16 | 1 | 0 | 16 |
| uniform | 12 | 15/16 | 1 | 0 | 16 |
| edge | 12 | **0/16** | 1 | **4** | 5 |

Uniform false detections are absorbed cleanly through false=12 (44% false rate).
Edge-biased false detections trip the identifier at false=12: only 5 observations get
assigned and 4 of those are false-detection rows landing on real catalog stars — a
catastrophic single-pass failure.

### Recovery with `--pyramid-restarts 3`

Re-running the edge=12 trial with `--pyramid-restarts 3 --confidence-fraction 0.5`:

| Mode | False | Restarts | Correct | Wrong | Assigned-on-false | Attempts |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| edge | 12 | 3 | 15/16 | 1 | 0 | 2 (won on attempt 1) |

The restart strategy recovers full correctness on the second attempt. So
edge-biased false detections are a **restart-fixable failure mode**, consistent with
the documented hypothesis that pyramid restart already covers the practical
catastrophe surface. This is fresh evidence against prematurely implementing top-K
verified-hypothesis ranking (PLAN item 5) — restart is sufficient here.

Open follow-ups:

- Higher edge-band widths (e.g. 64 / 128 px) and combinations with `--false-near-fraction`
  or hot-pixel modes — does the failure surface widen at higher mixed-mode budgets?
- Magnitude-aware false-positive intensity is the remaining single-axis realism gap;
  needs catalog magnitudes plumbed through `drop_star_ids.py` so false detections can
  cluster at brighter (or specifically bright-spike) magnitudes.

## 2026-05-09: Observation-Side Magnitude Realism Axis

The last realism gap closes: observation CSVs now carry a `mag` column populated from
the catalog magnitude of the synthesized star. Both identifiers consume it as a tighter
verification prior — `score = error + magnitude_prior_rad * |obs_mag - cat_mag|` — and
fall back to the legacy `score = error + magnitude_prior_rad * cat_mag` when the input
has no `mag` column.

### Plumbing

- `scripts/generate_star_tracker_observations_from_catalog.py` writes `id,u,v,mag`.
- `scripts/drop_star_ids.py` preserves the mag column when present and injects false
  detection magnitudes from a Gaussian (`--false-mag-mean`, `--false-mag-std`).
- `scripts/identify_stars_with_index.py::load_observations_with_mag` exposes the
  optional column; `verify_rotation` accepts `observation_magnitudes=None` for the
  legacy fallback.
- `apps/lost_in_space_pair_id` mirrors via `LoadedObservations { bearings, magnitudes }`
  and `identify_lost_in_space()` now takes `observation_magnitudes` (empty = legacy).

### Backwards compat

| Fixture | Path | Result |
| --- | --- | --- |
| 500 trial_000 (no mag column) | C++ identifier vs prior reference | byte-exact regression |
| 500 trial_000 (no mag column) | Python identifier vs prior reference | byte-exact regression |

### Correctness wins

| Fixture | Without mag prior (legacy) | With mag prior |
| --- | ---: | ---: |
| 16 truth × 16000 idx (clean) | 15/16, 1 wrong | **16/16, 0 wrong** |
| 16 truth + 12 edge-biased false (single-pass, no restart) | 0/16 (catastrophic) | **16/16, 0 wrong** |

Two qualitative improvements in one axis:

1. The persistent 1-wrong fixture-specific catalog ambiguity that survived through every
   prior sweep (distortion 0, undistorted, calibration-json) is **resolved** by the mag
   prior. A near-twin catalog star at very different magnitude no longer wins on pure
   angular error.

2. The catastrophic single-pass failure under 44%-rate edge-biased false detections —
   previously requiring `--pyramid-restarts 3` to recover — now succeeds on attempt 0
   when observation magnitudes are present. False detections inherit Gaussian mag (mean
   5, std 1) which still differs from real-star mags enough that the prior pushes
   correct (obs_mag, cat_mag) pairings to the front.

### Bit-exact gates

- 500-star regression (no mag column) byte-exact across both Py and C++.
- 16 truth × 16000 idx mag-augmented fixture: Py vs C++ byte-exact assignments CSV.
- Mag-augmented edge=12 fixture: C++ resolves all 16 observations correctly.

Decision: observation-side magnitude is now the recommended invocation when the
upstream pipeline has access to per-observation intensity. The fallback path stays
intact for star-image processing chains that don't yet emit per-detection magnitudes.
This effectively closes the realism-axis open list — remaining items (catalog aberration,
magnitude-aware false-positive *intensity* — false detections clustered specifically
around bright spike pixels rather than uniform Gaussian) are now refinements, not
fundamental gaps.
