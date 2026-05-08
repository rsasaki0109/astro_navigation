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
